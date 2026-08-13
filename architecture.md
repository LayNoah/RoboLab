# JTC vs WMX Lookahead Controller

Structural comparison of the two motion controllers that execute the pi0.5
VLA action chunks: the standard ros2_control `joint_trajectory_controller`
(JTC, baseline) and the wmx-r2 `lookahead_trajectory_controller` (WMX).
(Source material for architecture diagrams.)

---

## 1. Fundamental character

| | **JTC (ros2_control)** | **WMX Lookahead** |
|---|---|---|
| Command update | **Preemption** — a new goal replaces the running trajectory | **Append-only** — points are queued behind; no cancel |
| Interpolation | **Linear** between position-only waypoints | Time-axis path + velocity/accel limits + double smoothing filter |
| Execution layer | Non-realtime process on a general kernel | WMX3 **RT engine** (EtherCAT cycle determinism) |
| Real-robot path | Needs a separate RT hardware interface | The same binary used in sim drives real EtherCAT |

These character differences drive everything below.

---

## 2. Why the streaming scheme differs (append vs preemption)

The VLA emits a 15-step chunk per re-inference. How that chunk is fed to the
controller is opposite in the two paths.

### JTC — whole chunk (full-chunk)
- Every re-inference (8 steps) sends **all 15 points** as one
  `FollowJointTrajectory` goal.
- JTC **preempts**: a new goal discards the still-executing old one.
- → Stale plans never pile up, so sending the whole chunk is fine
  (the idiomatic ROS 2 usage).

### WMX — just-in-time (JIT) drip
- WMX lookahead has **no preemption API** (append-only). Feeding whole chunks
  lets ~0.5 s of stale plan accumulate in the buffer, so the arm always
  executes past intent — measured: the gripper closed 29–31 cm away from the
  target and every grasp failed.
- Fix: stream only **1–2 points per step**, keeping the un-executed backlog
  ≤ 2 control periods (~130 ms), so a re-plan takes effect within ~130 ms
  even without preemption → grasp succeeds (closes at 14 cm).
- Principle: "if you cannot preempt, keep nothing to discard."

```
JTC :  [15-pt goal] ── new goal ──▶ [old goal dropped, new 15 pts]   (preempt)
WMX :  [pt][pt] ── append 1/step ──▶ [pt][pt]  (buffer ~2 pts always) (JIT drip)
```

---

## 3. Why the interpolation / smoothing differs

### JTC — linear interpolation
Straight lines between position-only points. Velocity is discontinuous at
every waypoint by construction, and goal preemption **resets the velocity
profile** each time. → Tracking is exact (passes through the waypoints) but
acceleration/jerk are high.

### WMX — multi-stage smoothing pipeline
1. **Time-axis linear interpolation** for the skeleton
2. **Velocity/accel limits** (per-joint Franka physical values) → a physically
   executable profile
3. **Double moving-average filter** (30 ms × 2) → removes sharp corners /
   noise, drastically lowering jerk
4. (Upstream, in the client) **temporal ensembling** pre-softens the
   re-plan jumps at chunk boundaries

→ At the cost of a small ~3.4 mrad path deviation and ~70 ms lag, jerk is cut
sharply — a smoothing trade-off. Clear improvement if the goal is to not pass
the VLA waypoint noise straight through to the drives.

---

## 4. WMX internal structure that works around an SDK bug

WMX lookahead (SDK v3.7.0) cannot use the joints directly as the path axes:
- **Multi-axis corner-stop bug**: when several joints form the path, the
  engine decelerates to a stop at every direction change (corner). The
  `angleTolerance` option meant to suppress this does not work.

Workaround: **swap the roles of path axis and joints**.
- Path (main) axis = a **virtual master axis** (a motor-less axis whose
  position = accumulated trajectory time). Time only increases → no corners.
- Joints ride along as **auxiliary axes** attached to the path.
- Auxiliary axes are capped at 3 per segment → the 7 joints are split across
  **3 channels (3+3+1)**.
- All three channels share the **same master-axis time**, so joints 1–7 stay
  synchronized.

(JTC has no such workaround; being a standard stack, it puts all 7 joints
directly into the goal.)

---

## 5. Data flow (shared pipeline, controller swapped)

Both experiments share the client and bridge protocol and swap **only the
controller** (fair comparison). The port selects which controller.

```
Isaac Lab (VLA client, 15 Hz)
  │ chunk (15,8) → take joints[0:7]
  ▼
TCP bridge  (:5555 for WMX  /  :5556 for JTC)
  │ convert to a JointTrajectory message
  ▼
controller  (WMX lookahead  /  ros2_control JTC)
  │ compute commanded joint positions
  ▼
/joint_states (100 Hz) → client reads it, overwrites the arm action → Isaac PD tracks it
```

- WMX path: `--wmx --open-loop-horizon 8` (JIT drip + ensembling)
- JTC path: `--wmx --wmx-port 5556 --wmx-full-chunk` (goal-wise + preemption)

---

## 6. Measured motion quality

Same real pi05 chunk stream replayed into each controller, recorded at a
unified 100 Hz, with explicit stream-start (t0) time alignment.

### Per-joint (this is the primary evidence)

Acceleration (rad/s²) and jerk (rad/s³), lower = smoother. **WMX < JTC on
every single joint** — the improvement is not an artifact of one dominant
joint.

| joint | RMS acc WMX | RMS acc JTC | max acc WMX | max acc JTC | RMS jerk WMX | RMS jerk JTC |
|---|---|---|---|---|---|---|
| j1 | 2.07 | 3.28 | 15.5 | 40.5 | 91 | 242 |
| j2 | 6.99 | 12.11 | 68.1 | 250.5 | 306 | 900 |
| j3 | 1.51 | 2.55 | 14.1 | 52.6 | 66 | 187 |
| j4 | 4.94 | 7.83 | 61.2 | 161.0 | 218 | 579 |
| j5 | 3.65 | 5.74 | 41.3 | 106.4 | 160 | 423 |
| j6 | 5.60 | 8.89 | 60.3 | 158.7 | 246 | 658 |
| j7 | 9.52 | 15.21 | 109.0 | 287.6 | 418 | 1130 |

### Pooled summary (single number over all joints × time)

A convenience summary; note it is dominated by the larger-moving joints
(e.g. j7 produces the pooled max). Reported alongside — not instead of — the
per-joint table.

| metric | **WMX** | JTC | diff |
|---|---|---|---|
| RMS acc | **5.54** | 9.01 | -38 % |
| max acc | **109.0** | 287.6 | -62 % |
| RMS jerk | **243** | 668 | -64 % |
| tracking RMSE | 3.4 mrad | **1.2 mrad** | JTC better |
| command lag | ~70 ms | ~0 ms | JTC better |

- **WMX**: far smoother commands (jerk ~1/2.7) at the cost of small deviation/lag
- **JTC**: passes exactly through waypoints but with velocity discontinuities
  and acceleration spikes
- Same direction in the live banana-task recording too (max acc -35 %, jerk
  -59 %); both controllers completed the task (verdict + video verified)

---

## 7. One-line summary

> **JTC** is a preemptable standard trajectory controller: it runs chunks as
> whole goals and tracks waypoints exactly via linear interpolation (but
> roughly). **WMX Lookahead** is an append-only RT controller: JIT drip
> streaming + time-axis workaround + multi-stage smoothing shape the VLA
> commands smoothly (at a small deviation/lag cost). Choose WMX for
> smoothness, JTC for tracking accuracy.
