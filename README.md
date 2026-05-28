# 捷運到站偵測融合算法技術規格書

本演算法專為 Android 在地下隧道等無 GPS 訊號環境下設計。核心目標是透過低功耗的**加速度計（運作於時域能量）**與**陀螺儀（運作於姿勢抗噪）**進行感測器數據融合（Sensor Fusion），精準捕捉列車「進站煞車減速 ➔ 完全靜止開門」的斷崖式物理特徵，實現即時到站通知跳出。

---

## 1. 物理學本質與核心邏輯

捷運列車在隧道的運行具有強烈的物理慣性特徵：
* **隧道行駛與煞車**：鋼軌高頻摩擦與煞車減速會釋放持續且顯著的震動能量（$10\text{ Hz} \sim 30\text{ Hz}$）。
* **月台停靠**：車廂完全靜止，震動能量瞬間發生**斷崖式下跌（Cliff Drop）**，進入絕對死寂區間。

本演算法放棄脆弱的單秒絕對門檻判定，改用**「時間對齊雙向滑動視窗積分法」**。定義進站的鋼鐵鐵證為：**「過去短時間內有高強度的行駛煞車動能，且未來短時間內極度死寂。」**

---

## 2. 演算法架構與數學表達

### 2.1 感測器輸入與預處理 (Pre-processing)
系統以常規頻率（如 $50\text{ Hz}$）採集原始數據，並在每 1 秒的區間內進行統計學降採樣（Downsampling to 1Hz）：

1. **加速度總能量標準差 ($accel\_std$)**：
   消除重力變異，計算三軸加速度合向量變異數。
   $$mag_t = \sqrt{x_t^2 + y_t^2 + z_t^2} - 9.81$$
   $$accel\_std = \text{Standard Deviation}(mag) \text{ over 1-second interval}$$

2. **陀螺儀角速度標準差 ($gyro\_std$)**：
   用於監測手機絕對空間旋轉。**注意：不使用易受車廂高壓電磁干擾的電子羅盤（磁力計），改用純慣性元件陀螺儀以阻絕電磁雜訊。**
   $$gyro\_mag_t = \sqrt{gyro\_x_t^2 + gyro\_y_t^2 + gyro\_z_t^2}$$
   $$gyro\_std = \text{Standard Deviation}(gyro\_mag) \text{ over 1-second interval}$$

### 2.2 三大核心防禦特徵工程

#### ① 陀螺儀人體晃動防禦 (Hand-shake Rejection)
* **目的**：消除使用者在站內「滑手機」、「收包包」、「轉身」等自車非相關的人體運動雜訊。
* **邏輯**：當陀螺儀偵測到手機角速度劇烈改變（$gyro\_std > 25.0^\circ/\text{s}$），代表此震動由人體主動產生而非火車釋放，**強制將當秒的加速度能量降至靜止基底值**。
$$\text{If } gyro\_std_t > 25.0 \implies accel\_pure_t = 0.05 \quad (\text{Otherwise } accel\_pure_t = accel\_std_t)$$

#### ② 自適應環境增益控制 (Adaptive Gain Control)
* **目的**：對抗手機放置姿勢不同造成的 Y 軸尺度大幅落差（例如：放在身上動能被大腿吸收上限僅 `0.5`；手拿懸空雜訊大上限達 `2.0`）。
* **邏輯**：動態維護一個 60 秒的環境長窗平均值，自適應計算當前的靜止閾值（Threshold），並實施硬邊界截斷。
$$dynamic\_env\_energy = \text{Mean}(accel\_pure) \text{ over past 60s}$$
$$adaptive\_threshold = \text{Clip}(dynamic\_env\_energy \times 0.50, \text{lower}=0.05, \text{upper}=0.18)$$

#### ③ 雙向時序特徵視窗 (Dual-Window Decision Engine)
透過兩個滑動環形緩衝區（Ring Buffers）在時間軸上同時向前與向後透視：
* **過去行駛窗 ($past\_energy$)**：計算過去 10 秒（`DRIVE_WINDOW_SEC = 10`）的平均能量，必須 $> 0.08$，確保前面列車真的有在隧道開動或煞車（排除中途滑行誤判）。
* **未來安靜窗 ($future\_quiet$)**：計算未來 10 秒（`QUIET_WINDOW_SEC = 10`）的能量**中位數（Median）**，必須 $<$ `adaptive_threshold`，確保車廂確實進入月台靜止期。

## 3. 決策狀態機與標定流程

當時間軸掃描到某一秒 $t$ 時，必須同時滿足以下三大物理條件，才判定為**進站開門瞬間**：

```python
# 核心決策虛擬碼
Condition_1 = future_quiet[t] < adaptive_threshold[t]  # 未來 10 秒極度死寂 (突破閾值)
Condition_2 = past_energy[t] > 0.08                    # 過去 10 秒車子真的有在動
Condition_3 = (t - last_station_time) > 70             # 時間壁壘：距離上一站 > 70 秒

if Condition_1 and Condition_2 and Condition_3:
    # 判定觸發進站！
    # 在當前雙向窗的 [t : t + QUIET_WINDOW_SEC] 區間內
    # 尋找 accel_pure 的絕對波谷最小值作爲紅線精準對齊點 (精準度達 ±1秒)
    best_station_time = find_minimum_wave_trough()
    trigger_notification(best_station_time)
    last_station_time = best_station_time

```

## 4. 實車測試參數配置表 (Configuration)

在 Android/iOS App 實作實車測試中，黃金配置參數如下：

| 參數名稱 | 最佳化設定值 | 物理意義 |
| --- | --- | --- |
| `DRIVE_WINDOW_SEC` | **10 秒** | 隧道行駛驗證長度。縮短至 10 秒可完美相容進站前偶爾的手晃抹平區間。 |
| `QUIET_WINDOW_SEC` | **10 秒** | 月台靜止驗證長度。連續 10 秒安靜即可確認非隧道中途滑行。 |
| `MIN_STATION_INTERVAL_SEC` | **70 秒** | 兩站間冷卻壁壘。防止在同一個月台因微小人體震動重複觸發跳通知。 |
| `GYRO_REJECT_THRES` | **25.0 deg/s** | 手晃過濾門檻。高於此值視為使用者操作手機，啟動訊號淨化。 |
| `THRESHOLD_GAIN` | **0.50** | 門檻縮放增益。動態環境平均能量的 50% 作為靜止判定切點。 |

## 5. 系統優勢

1. **純慣性硬隔離**：完全放棄磁力計，改用陀螺儀與加速度計融合，物理上阻絕捷運高壓電磁與鋼體屏蔽干擾。
2. **零延遲動態 Gain**：引入自適應增益，不論手機明天是拿在手上滑、放在口袋、還是放包包，演算法皆能自動對齊 Y 軸。
3. **雙向視窗安全鎖**：完美過濾了「捷運在隧道中途熄火滑行」的提早誤判，只有真正的動能斷崖跌落才會觸發到站通知。
