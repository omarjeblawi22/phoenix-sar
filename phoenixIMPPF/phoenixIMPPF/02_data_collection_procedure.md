# PHOENIX — Part 2a: Data Collection Procedure

The IMPPF's convergence depends almost entirely on **angular diversity** of
the SRV around the AP. A robot that only drives radially toward the AP cannot
distinguish "AP 10 m ahead" from "AP 10 m to the side" — every particle on
the ring at distance 10 m fits the data equally well. The patterns below are
designed to break that ambiguity quickly while exercising the LOS/NLOS
switch.

## 0. Pre-mission checklist

1. **AP surveying.** Place the target AP at a fixed location. Measure
   `(x_ap_true, y_ap_true)` from at least two map landmarks with a laser
   distance meter; record to ±2 cm. Tape a printed label with the
   coordinates onto the AP enclosure — you will need them for error scoring.
2. **Height matching.** AP and SRV antenna at 1.0 m. Keep within ±10 cm.
3. **AP firmware.** Verify FTM responder mode is active and SSID `FTM_AP` is
   broadcasting on the configured channel (default 6).
4. **SRV.** Power on, wait for SLAM to publish `/map` and a stable `map →
   base_link` transform. Drive a small 1 m square as a smoke test —
   trajectory should look like a square in RViz.
5. **Logger.** Start the logger node and confirm `rtt.csv` is growing at
   ~2.5 lines per second:
   ```
   ros2 run phoenix_logger phoenix_logger \
       --ros-args -p output_dir:=/home/pi/datasets/run01 \
                  -p serial_port:=/dev/ttyACM0
   ```
6. **Battery.** 3S LiPo above 11.4 V. Below that the ESP32's RF power sags
   and biases distance estimates upward.

## 1. Driving patterns

### Pattern A — Lawnmower (clean LOS, convergence baseline)

- Single open room with AP in one corner, line-of-sight everywhere.
- Drive a serpentine: 2 m row spacing, full sweep across the room.
- **Constant 0.3 m/s.**
- Label sequence: `LOS_STATIC` (while stopped), then `LOS_DYNAMIC` once moving.
- Aim for ~3 minutes of motion, covering ~50 m of path.
- **Purpose:** verify the filter converges in the easy case before testing
  harder ones.

### Pattern B — Single wall crossing

- Two rooms separated by one thin wall with a doorway.
- AP fixed in room 1; SRV starts in room 2.
- Drive **parallel to the wall** for ~10 m at 1.5 m offset, then through the
  doorway into room 1, then along the wall on the LOS side for another 10 m.
- **0.4 m/s.**
- Label sequence: `NLOS_WALL` → `NLOS_DOOR` (within ±1 m of the doorway) →
  `LOS_DYNAMIC`.
- **Purpose:** validate the LOS/NLOS switch fires correctly at the wall
  boundary; ensure the bias term doesn't cause a discontinuous jump in the AP
  estimate.

### Pattern C — Multi-room circuit (realistic SAR)

- Three or more rooms with the AP in the farthest corner room.
- Drive a figure-8 that visits every room twice, passing every doorway in
  both directions.
- **0.5 m/s.**
- Update labels as you go — the firmware accepts new labels at any time via
  serial:
  ```
  echo NLOS_WALL > /dev/ttyACM0
  ```
  (The logger node also forwards labels you publish on a `/phoenix/label`
  topic if you wire that in.)
- ~5 minutes total.
- **Purpose:** this is the *primary* validation dataset for the IMPPF.

### Pattern D — Static distance set (calibration data)

- Used to fit `(a, b, σ_los)` and `(Bias_thin_wall, σ_nlos)`. See
  `01_firmware_and_calibration.md §2.1–§2.2`.
- 2 minutes stationary at each surveyed distance, in both LOS and through
  one wall.
- Drive slowly between stations to avoid accidentally including motion data.

## 2. While driving — habits that prevent ruined datasets

- **Never drive straight at the AP.** Always include arcs. A simple
  guideline: in any 30-second window the SRV's heading vector should sweep
  through at least 90°.
- **Stop briefly (5 s) at each label change.** It gives you a clean visual
  marker in `rtt.csv` and lets the filter pick up the new measurement model.
- **No one walks between SRV and AP.** A human body in the Fresnel zone adds
  3–8 dB of attenuation and several meters of bias.
- **Avoid driving within 0.5 m of metal cabinets, fridges, etc.** They make
  strong reflectors that the IMPPF's two-mode model cannot accommodate.
- **Single-AP environment.** During data collection, turn off or move out of
  range any other 2.4 GHz APs on channel 6 ±2; co-channel beacons can be
  mistakenly picked up by the FTM initiator's scan if you ever re-associate.
- **Battery floor 11.4 V.** Plug in or swap if you fall below.

## 3. Post-mission verification

Immediately after stopping the logger, before tearing anything down:

1. Check files exist and are non-empty:
   ```
   ls -la /home/pi/datasets/run01/
   # expect: rtt.csv, trajectory.csv, map.npy, map_meta.json, (csi.csv)
   ```
2. **Sanity check distances:**
   ```python
   import pandas as pd
   r = pd.read_csv('rtt.csv')
   print(r['d_median_m'].describe())
   ```
   The min/max/median should be roughly consistent with the geometry you just
   drove. A median of 25 m in a 30×30 m room means the AP went offline or the
   FTM offset `b` is wildly off.
3. **Quick map view:**
   ```python
   import numpy as np, matplotlib.pyplot as plt
   m = np.load('map.npy')
   plt.imshow(m, origin='lower'); plt.show()
   ```
   It should look like the room.
4. **Label coverage:** every label you intended to record should appear in
   `rtt.csv`. If `NLOS_DOOR` is missing, you forgot to send the label
   command — re-do that segment.

## 4. Common pitfalls — quick reference

| Symptom in dataset                              | Cause                                            | Fix                                    |
|-------------------------------------------------|--------------------------------------------------|----------------------------------------|
| `rtt.csv` rate ≪ 2.5 Hz                          | Serial buffer overrun, FTM session failures      | Lower baud throughput or check AP      |
| `pose_ok=0` on many rows                         | TF not flowing, SLAM not running                 | Restart SLAM; check `ros2 topic hz /tf`|
| Distance jumps discontinuously by 5+ m          | Robot re-associated mid-run                      | Check Wi-Fi reconnect logs             |
| Distance noise way above σ_los                   | Someone walking nearby, or metal reflector       | Re-do the segment                      |
| Filter converges to mirror image of AP location | All-radial trajectory — no angular diversity     | Add an arc; re-run Pattern C           |
