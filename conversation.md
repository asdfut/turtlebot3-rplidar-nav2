# 對話紀錄（摘要）：TurtleBot3 Nav2 與路線錄製（2026-10-28）

與 Claude Code 的對話整理，依時間順序。完整步驟與問題說明見 [README.md](README.md)。

## 1. 系統與機器人狀態檢查
- 筆電：剛開機，RAM 7.5 GiB、磁碟 55%，連手機熱點
- 從 `~/.bash_history` 找到機器人位址，SSH 金鑰免密碼登入成功
- 機器人：Raspberry Pi 4B、Ubuntu 24.04.5、46.7 °C，bringup 已用 `use_lidar:=false` 啟動，OpenCR 在 `/dev/ttyACM0`
- 舊的區網位址連不到，因為換了網路

## 2. 用現有地圖啟動 Nav2
- 地圖在筆電的 `~/map.yaml`（124 × 92，0.05 m/格）；機器人上沒有安裝 Nav2，所以 Nav2 在筆電上跑
- 發現 RPLIDAR（CP2102）接在**筆電**的 `/dev/ttyUSB0`，在筆電上啟動 `rplidar_composition`，`frame_id:=base_scan`，`/scan` 約 7.8 Hz
- 啟動 `turtlebot3_navigation2 navigation2.launch.py map:=~/map.yaml`

## 3. 第一次啟動失敗：60 秒逾時
- 起始位置沒有在 60 秒內設好，`global_costmap` 等 `map` frame 逾時，`planner_server` 啟動失敗
- 之後在 RViz 按了 Startup、Pause、Reset，所有節點被重置成 unconfigured，使用者設的起始位置沒有送到 AMCL
- 處理：先只啟動定位（`lifecycle_manager_localization` STARTUP），等設好位置再啟動導航

## 4. Reset 後再啟動，Nav2 當機
- 從 Reset 狀態重新 configure 時，`nav2_container` 在 `route_server` 那裡 SIGSEGV
- 處理：整個 Nav2 重開。AMCL 一啟動就自動送出剛才定位到的位置，避開 60 秒限制，所有節點都順利變成 active
- 第一次單點導航：目標在正後方 0.5 m，`Failed to make progress` 後取消，接著又按了 Reset 再 Startup，第二次重現 SIGSEGV

## 5. 用 teleop 錄路線、用 Nav2 跑路線
- 寫了 `record_route.py`（從 TF 取樣並自動記點）和 `follow_route.py`（使用 `nav2_simple_commander`）
- 第一次錄到的路線只有約 0.4 m，而且錄完後 Nav2 被 RViz 的 Pause 暫停了，用 Resume 恢復，沒有當機
- 執行路線時遇到 `Failed to transform from  to map`，原因是 `burger.yaml` 把 through-poses 設成單點的行為樹。改成在腳本中指定正確的行為樹
- 修好後回報 `SUCCEEDED`，但機器人沒動，因為所有點都在 0.7 m／0.25 m 的判定範圍內

## 6. 第二條路線
- 錄製間隔改成 0.7 m，重新錄了約 1 m 的直線（3 點）
- 機器人停在終點，所以 through-poses 又立刻判定成功。把腳本改成先開到起點，再走其餘的點
- 這次機器人開到了起點，但沿路線走時 AMCL 位置大幅跳動，一度跑到 (-0.31, -1.84)，規劃失敗、旋轉恢復也失敗，之後 Nav2 被 Pause
- 判斷根本原因是定位不穩，可能跟光達固定方式、安裝方向或地圖特徵太少有關，待下次確認

## 7. 整理成果並上傳 GitHub
- 建立本 repo，內容包括腳本、機器人端 launch 檔、地圖、路線、預覽圖、README 和本紀錄
