# PHOENIX — Part 1: Firmware Review & Calibration Protocol

## 1. Firmware Review (main.c)

### 1.1 Critical issue — burst timestamp is at burst *end*, not start

`BURST_START` is printed only after `esp_wifi_ftm_initiate_session()` returns and
the report event has fired. On the ESP32-S3 a 64-frame FTM session takes roughly
**150–320 ms** (the actual exchange duration is non-deterministic — it depends on
contention and retries). At your top speed (0.6 m/s) the SRV moves
**9–19 cm during one burst**, then the line lands on the Pi and gets timestamped
by `rclpy.time.Time()`. The Pi then looks up TF at the wrong moment in the past.

Consequence for the IMPPF: every burst is associated with a pose offset by 2–4
grid cells from where ranging actually started, which silently corrupts the
LOS/NLOS raycast classification near walls.

**Fix:** emit a marker *before* initiating FTM so the logger captures the SLAM
pose at burst start. Keep the existing `BURST_START` line for the per-frame
loop. Both lines also carry the ESP32 monotonic timestamp so the Pi can detect
clock drift between MCU and Pi.

Add at the top of `main.c`:

```c
#include "esp_timer.h"      // for esp_timer_get_time()
```

In `ftm_task()`, immediately before `esp_wifi_ftm_initiate_session(&fc)`:

```c
/* Tag burst at TX time so the Pi can pin SLAM pose to ranging start. */
int64_t t_tx_us = esp_timer_get_time();
printf("BURST_TX,%" PRIu32 ",%" PRId64 ",%s\n",
       s_seq, t_tx_us, lbl);
fflush(stdout);
```

And in the existing `BURST_START` line, add the MCU timestamp:

```c
printf("BURST_START,%" PRIu32 ",%" PRId64 ",%u,%s\n",
       s_seq, esp_timer_get_time(), s_snap.n, lbl);
```

(Update the schema comment to match.)

### 1.2 Reduce in-burst spatial smear

64 frames at the fastest FTM rate is overkill for an IMPPF: the per-burst median
already converges well before that, and the burst duration is what limits how
finely you can spatially register a measurement to a SLAM pose.

Recommended values:

| #define              | Current | Suggested | Reasoning                                       |
|----------------------|---------|-----------|-------------------------------------------------|
| `FTM_FRMS_PER_BURST` | 64      | **32**    | Median over 32 frames already gives ~0.2 m of burst-level σ; halves burst duration and therefore in-burst smear to ~5–10 cm at 0.6 m/s. |
| `MEASURE_INTERVAL_MS`| 600     | **400**   | 2.5 Hz update rate gives better trajectory coverage during a single room traversal; well within the radio duty-cycle budget. |
| `FTM_BURST_PERIOD`   | 0       | 0         | Leave at "fastest"; the 802.11mc burst period field becomes meaningful only when you want multiple bursts inside one session, which you don't. |

### 1.3 Console throughput

At 32 frames/burst + ~10 CSI lines per burst, each burst emits ~5 KB of ASCII.
At 2.5 Hz that is ~12 kB/s, which **saturates the default 115200 baud UART
(11.5 kB/s)**. You will start losing lines silently. Two options:

- **Best:** use the native USB-CDC console (XIAO ESP32-S3 has built-in USB).
  Add to `sdkconfig.defaults`:
  ```
  CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
  ```
  USB throughput is effectively unlimited for this workload.
- **If you must stay on UART:** raise the baud rate:
  ```
  CONFIG_ESP_CONSOLE_UART_BAUDRATE=921600
  ```
  and update the logger to match.

### 1.4 CSI — keep collecting it, but don't feed it to the IMPPF

The PDF's IMPPF derives LOS/NLOS from the SLAM map raycast, not from CSI. The
CSI stream is useful future fuel for a learned NLOS classifier (it captures
multipath shape), so I'd keep the firmware collecting it and let the logger
store it — just don't make the IMPPF depend on it.

### 1.5 Minor — make sure `BURST_TX` is flushed before FTM

Without the explicit `fflush(stdout)` shown in §1.1 the line can sit in the
stdio buffer until after the session completes, defeating the whole point of
the fix. The existing `fflush(stdout)` after the per-burst block does not help
here because it runs *after* FTM, not before.

---

## 2. Calibration Protocol

The IMPPF has **three** noise parameters and **one** hidden systematic offset
you must measure before deployment:

| Parameter         | Symbol        | Meaning                                   |
|-------------------|---------------|-------------------------------------------|
| LOS noise         | σ_los         | Residual std after removing systematic FTM offset, in clear LOS |
| NLOS noise        | σ_nlos        | Residual std through a thin wall          |
| Thin-wall bias    | Bias_thin_wall| Mean extra range introduced by the wall   |
| FTM offset (hidden)| b            | Systematic per-device range offset (TX/RX hardware delays). Not in the PDF, but you MUST remove it or it will dominate σ_los. |

The PDF assumes `d_rtt` is already de-biased of `b`. Skipping this step is the
single most common reason FTM particle filters fail to converge.

### 2.1 LOS calibration

**Setup**

- Open area, no surface within 3 m perpendicular to the AP–SRV line (a gym or
  empty lab works; outdoors is even better but check Wi-Fi regulations).
- AP and SRV both mounted at 1.0 m above floor. Same height eliminates one
  source of geometric error.
- Use a laser distance meter or steel tape; record true range to ±2 cm.

**Procedure**

1. Place SRV at distances **d_true ∈ {1, 2, 3, 5, 7, 10, 15, 20} m**.
2. At each station: SRV stationary, label `LOS_STATIC`, log for **2 minutes**
   (~300 bursts at 2.5 Hz).
3. For each burst, compute
   `d_meas = median over frames of (RTT_ps · c / 2e12)`.

**Fit**

Fit a linear model across all stations:
```
d_meas = a · d_true + b + ε,  ε ~ N(0, σ_los²)
```
- `a` should be ≈ 1.0 (typically 0.98–1.02). If it isn't, you have a scale
  problem — check that the firmware reports `rtt_ps` in picoseconds.
- `b` is the systematic offset — typically **1.5–3.0 m** for ESP32-S3 pairs.
  Apply at runtime as `d_corrected = (d_meas − b) / a`.
- `σ_los = std(residuals)` after applying the correction. Expect **0.3–0.8 m**.

**Sanity check:** plot d_meas vs d_true. A non-linear shape (e.g. saturation
beyond 10 m) tells you the maximum reliable range — anything beyond it should
be down-weighted or rejected at the logger.

### 2.2 NLOS calibration

**Setup**

- Same vertical geometry as §2.1.
- Insert exactly one thin wall (12.5 mm drywall on metal studs is the typical
  reference; calibrate other materials separately if present).
- Both AP and SRV ≥ 1 m back from the wall to keep the geometry clean.

**Procedure**

1. d_true ∈ {1, 2, 3, 5, 7} m through the wall.
2. Label `NLOS_WALL`. 2 minutes per station.
3. Apply the LOS correction first: `d_los_corrected = (d_meas − b) / a`.
4. Compute NLOS residual: `e_nlos = d_los_corrected − d_true`.

**Estimates**

- `Bias_thin_wall = mean(e_nlos)` — expect **0.5–1.5 m** for drywall. This is
  not a slope; it is treated as an additive delay independent of distance, per
  the PDF's equation (4).
- `σ_nlos = std(e_nlos)` — expect **0.8–2.0 m**.

**Crucial warning.** Plot a histogram of `e_nlos` before trusting `σ_nlos`.
NLOS distributions are almost always heavy-tailed and sometimes bimodal
(reflected path vs. through-wall path). A Gaussian is a working approximation
that the PDF's equation (4) commits to, but its tails are wrong. If your
histogram has a long upper tail, inflate `σ_nlos` by ~1.5× to keep particles
from being prematurely killed.

### 2.3 Closed-loop validation

After calibration, drive Pattern C (see Part 2) once. The IMPPF estimate should
converge to within **1.5 m** of the surveyed AP position by the end of the run.
If it doesn't:

| Symptom                                | Likely cause                                     |
|----------------------------------------|--------------------------------------------------|
| Particles collapse to wrong location early | σ_los too small, or `b` not removed              |
| Particles never collapse                | σ_los or σ_nlos too large                        |
| Estimate biased radially away from SRV | `Bias_thin_wall` underestimated                  |
| Estimate jumps when crossing a doorway | LOS/NLOS raycast misclassifying — check map resolution alignment |

### 2.4 Re-calibration cadence

- Repeat **whenever you swap either ESP32** (the offset `b` is chip-pair-specific).
- Repeat **monthly** during active development — temperature and aging drift `b`
  by ~10–30 cm.
- A 5-minute one-station check at d_true = 5 m is enough as a sanity check
  between full calibrations.
