# 對話紀錄（摘要）：RPLIDAR 排錯與純光達 SLAM（2026-10-31）

與 Claude Code 的對話整理，依時間順序。前一次的紀錄見 [conversation.md](conversation.md)。

## 1. 光達沒有回應
- 筆電 `lsusb` 有 `10c4:ea60 CP2102`，裝置在 `/dev/ttyUSB0`，沒有其他程式佔用
- `rplidar_ros view_rplidar.launch.py` 啟動後出現 `Error, operation time out. RESULT_OPERATION_TIMEOUT!`，`/scan` 沒有 publisher
- 直接用 Python 對串口送 GET_INFO（`A5 50`），115200／256000／460800／1000000／1500000 都沒有回應；被動監聽 115200–2000000 也收不到資料
- 重新插 USB、壓緊光達和轉接板之間的白色接頭，結果都一樣

## 2. 確認型號：RPLIDAR A2，不是 A1
- 從照片判斷是 **RPLIDAR A2**（外殼整個包住、三角形底座、銅柱），之後 GET_INFO 回報 model `0x18`、韌體 1.29、硬體 rev 7，確認是 **A2M8**，速率 115200
- 光達線一出來就剝皮，7 條細線直接插進轉接板，這段容易接觸不良

## 3. 轉接板短接測試與換板
- 舊轉接板 TX/RX 短接後送 26 bytes 收不到，一度判斷是轉接板故障
- 換新轉接板後短接測試也失敗，兩塊都失敗不太合理，比較可能是短接到錯的腳位
- 之後再測，光達 GET_INFO 有回應了，但 ROS 驅動仍然失敗：
  - 第一次：背景還有之前沒關掉的驅動和 RViz，清掉後重開
  - 第二次：讀到 S/N、韌體和健康狀態，但 `Cannot start scan: '80008000'`、`Failed to set scan mode`

## 4. 找到根本原因：USB 2.0 埠供電不足
- 馬達不轉時連續 30 次 GET_INFO 全部成功，SCAN 指令也有回應，只是沒有掃描資料
- 用 DTR 啟動馬達的瞬間，系統紀錄出現 `cp210x ttyUSB0: failed set request ... status: -19`、`converter now disconnected`，接著自動重新連上
- 原因：A2 馬達啟動電流太大，USB 2.0 埠（500 mA）撐不住，轉接板斷電重開。前面 device 編號從 14 一路跳到 25，很可能大部分也是這樣造成的
- **改插 USB 3.0 埠（900 mA）後就正常了**：馬達啟動不再斷線，`view_rplidar` 顯示 Sensitivity 模式、最遠 12 m，`/scan` 約 6.4 Hz

## 5. 準備接 Nav2
- 光達驅動改用 `rplidar_composition`，`frame_id:=base_scan`，`/scan` 約 6.4 Hz
- 機器人連不到：筆電在「AGI wifi 108 5G」（192.168.0.x），192.168.0.118 沒回應，區網上也沒有樹莓派，Nav2 先不開

## 6. 純光達 SLAM（Cartographer）
- 機器人不在線，改成只用光達建圖，沒有里程計也沒有 IMU，設定檔是 [config/lidar_only_2d.lua](config/lidar_only_2d.lua)
  - `tracking_frame` 和 `published_frame` 都是 `base_scan`，`provide_odom_frame: true`，TF 是 `map → odom → base_scan`
  - 沒有里程計，靠 online correlative scan matching（搜尋範圍 ±0.2 m、±30°）推算移動，所以要慢慢移動
- 執行指令：
  ```bash
  ros2 run cartographer_ros cartographer_node \
    -configuration_directory config -configuration_basename lidar_only_2d.lua
  ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0
  ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/turtlebot3_cartographer/rviz/tb3_cartographer.rviz
  ```
- log 一直出現 `Dropped ~46 earlier points`，是 RPLIDAR 每圈共用一個時間戳記造成的，每圈只丟掉少量點，不影響建圖
- 使用者那邊的存檔指令沒有存成功，關閉 SLAM 前由 Claude 存成 `maps/map_new.pgm` / `maps/map_new.yaml`（173 × 169 格，0.05 m/格，約 8.7 m × 8.5 m，原點 `[-3.05, -1.80]`），原本的 `maps/map.*` 沒有動到
- 這組 SLAM 會自己發布 `map → odom`，機器人上線跑 Nav2 前一定要先關掉，不然 TF 會衝突

## 7. SLAM 與 Nav2 技術細節（重點）
- **TF：** `map → odom`（SLAM 或 AMCL）→ `base_footprint`（輪子里程計）→ `base_scan`（URDF 固定安裝位置）
- **Cartographer：** 前端用 correlative scan matching 加 Ceres 微調，把掃描寫進 submap；後端用位姿圖加閉環偵測（`min_score 0.65`），每 30 個節點做一次最佳化
- **AMCL：** 500–2000 個粒子，里程計雜訊 `alpha1–5 = 0.2`，likelihood field 感測模型；移動超過 0.25 m 或轉超過 0.2 rad 才更新一次
- **Costmap：** static、obstacle/voxel、inflation 三層（`inflation_radius 0.5`、`cost_scaling_factor 5.0`、`robot_radius 0.1`）
- **規劃與控制：** NavFn 規劃全域路徑；DWB 控制器每秒 10 次，最高 0.3 m/s、1.0 rad/s；Velocity Smoother 輸出 `/cmd_vel`（TwistStamped）
- **BT Navigator：** 用行為樹串起重新規劃、控制和失敗時的恢復動作（清 costmap、旋轉、後退、等待）
- **AMCL 位置跳動最可能的原因：** RPLIDAR 的實際安裝位置或朝向和 URDF 的 `base_scan`（原廠 LDS 位置）不一致，下次要優先確認

## 待辦
- 確認 10/28 用的是不是另一顆 A1（README 寫 A1M8、`/scan` 7.8 Hz，今天這顆是 A2M8、6.4 Hz）；如果是同一顆，就把 README 改成 A2M8
- README 註明 A2 要插 USB 3.0 埠，或用有外接電源的 USB Hub
- 機器人上線後：關掉純光達 SLAM → 機器人 bringup（`use_lidar:=false`）→ Nav2 用 `maps/map_new.yaml`
- 確認光達實際安裝位置和朝向，必要時修正 `base_scan` 的 TF
