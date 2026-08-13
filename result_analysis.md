# Result Analysis: Motion Quality (WMX Lookahead vs JTC)

## Method

The same real pi0.5 chunk stream — harvested from an actual BananaInBowlTask
episode (`CHUNK_LOG_PATH`) — was replayed into each controller and the joint
output was recorded, so any difference is attributable to the controller
(identical input). Controls for methodological validity:

- **Identical input**: the same 450-point command stream fed to both.
- **Unified sampling**: both stacks recorded at 100 Hz.
- **Explicit time alignment**: the feeder logs the stream-start time (t0);
  alignment is done against the point schedule, not progress-normalization.
- **Single publisher verified** on `/joint_states` before each capture.

Metrics (lower = smoother), from position → velocity → acceleration → jerk by
finite difference:
- **RMS acc / RMS jerk**: root-mean-square over the motion window.
- **max acc**: peak absolute acceleration.
- **tracking RMSE**: deviation from the commanded waypoint schedule.

---

## Per-joint results (primary)

Acceleration (rad/s²) and jerk (rad/s³). **WMX is lower than JTC on every one
of the 7 joints**, so the improvement is not an artifact of a single
dominant joint.

| joint | RMS acc WMX | RMS acc JTC | max acc WMX | max acc JTC | RMS jerk WMX | RMS jerk JTC |
|---|---|---|---|---|---|---|
| j1 | 2.07 | 3.28 | 15.5 | 40.5 | 91 | 242 |
| j2 | 6.99 | 12.11 | 68.1 | 250.5 | 306 | 900 |
| j3 | 1.51 | 2.55 | 14.1 | 52.6 | 66 | 187 |
| j4 | 4.94 | 7.83 | 61.2 | 161.0 | 218 | 579 |
| j5 | 3.65 | 5.74 | 41.3 | 106.4 | 160 | 423 |
| j6 | 5.60 | 8.89 | 60.3 | 158.7 | 246 | 658 |
| j7 | 9.52 | 15.21 | 109.0 | 287.6 | 418 | 1130 |

Every joint: WMX RMS acc, max acc, and RMS jerk are all below JTC. The peak
acceleration on JTC is roughly 2.6–3.7× higher joint-by-joint.

---

## Pooled summary (single number, all joints × time)

A convenience aggregate. Note it is dominated by the larger-moving joints
(j7 produces the pooled max), so it is reported **alongside**, not instead of,
the per-joint table.

| metric | **WMX** | JTC | diff |
|---|---|---|---|
| RMS acc | **5.54** | 9.01 | -38 % |
| max acc | **109.0** | 287.6 | -62 % |
| RMS jerk | **243** | 668 | -64 % |
| tracking RMSE | 3.4 mrad | **1.2 mrad** | JTC better |
| command lag | ~70 ms | ~0 ms | JTC better |

---

## Interpretation (trade-off, stated honestly)

- **WMX is far smoother**: lower acceleration and jerk on every joint (pooled
  jerk ≈ 1/2.7 of JTC). This is the multi-stage smoothing (velocity/accel
  limits + double 30 ms moving-average filter) plus upstream temporal
  ensembling acting on the VLA waypoints.
- **JTC tracks more accurately**: linear interpolation passes exactly through
  the waypoints (RMSE 1.2 vs 3.4 mrad) with near-zero lag, but the price is
  velocity discontinuity at every waypoint → high acceleration/jerk spikes.
- **The WMX advantage is a smoothing trade-off**: ~3.4 mrad path deviation and
  ~70 ms lag bought a large reduction in jerk. Worthwhile when the goal is to
  avoid passing VLA waypoint noise (including re-plan jumps) straight to the
  drives; the ~70 ms lag did not harm the task (grasp succeeded, verified).

---

## In-episode cross-check (live banana task)

Recorded during actual task execution (behavior-confounded, so directional
only — one episode each, both succeeded with verdict + video verification):

| metric | **WMX-JIT episode** | JTC episode |
|---|---|---|
| RMS acc | **1.89** | 2.66 (-29 %) |
| max acc | **83.6** | 128.9 (-35 %) |
| RMS jerk | **79** | 192 (-59 %) |

Same direction as the deterministic replay: WMX produces smoother motion
during the real task as well.

---

## Caveats

- 1 replay + 1 episode per controller. The replay is deterministic (repeat
  not required); the in-episode layer would benefit from more episodes (N≥10)
  for statistical claims.
- Sim task success is not used as a motion-quality signal (a prior session
  showed the simulator's success verdict can disagree with the video, so
  episodes here were frame-verified).
- Raw data: `~/recording/vla_jit_2/data/` (replay CSVs + meta + episode CSVs).
