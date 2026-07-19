"""Pi0 client variant that executes action chunks through a WMX motion controller.

Instead of applying the raw VLA chunk actions directly to the simulated arm,
this client streams chunks to the wmx-r2 ``lookahead_trajectory_controller``
(via the ``wmx_chunk_bridge.py`` TCP bridge — rclpy cannot be imported here
because Isaac Lab runs Python 3.11 while ROS 2 Jazzy ships Python 3.12
bindings) and returns the WMX *commanded joint positions* as the per-step arm
action. The Isaac Lab arm then tracks WMX output, so the sim shows exactly the
motion the motion controller would produce on real hardware.

Streaming modes (``stream_mode``):

- ``jit`` (default): **drip / just-in-time streaming.** Every step, top the
  WMX buffer up to only ``jit_lead`` (2) points ahead. The un-executed
  backlog is therefore bounded by ~2 control periods (~130 ms at 15 Hz), so
  when the policy re-plans, at most 2 stale points execute before the new
  intent takes effect — preemption-grade responsiveness without ever
  stopping the lookahead stream. (The WMX lookahead module has no
  truncate-pending-points API, and Stop/Clear/Start would decelerate to a
  stop on every re-plan; keeping the buffer nearly empty makes preemption
  unnecessary.)
- ``segment``: stream ``open_loop_horizon`` ensembled points per re-plan
  (the earlier TE behavior; backlog ~0.5 s — measured to break closed-loop
  grasping: the arm executes stale intent while the passthrough gripper
  fires on the VLA's schedule).
- ``full``: send every fresh chunk whole, the way a conventional client
  hands trajectories to a controller. Used for controller-stack baselines
  (e.g. the ros2_control JTC bridge on port 5556).

Temporal ensembling (ACT-style) applies in ``jit`` and ``segment`` modes:
each streamed point is a weighted average of every stored chunk's prediction
for that timestep (newest weight 1, each older chunk decayed by ``te_decay``).

The first ``infer`` after a reset only presets WMX to the sim pose and holds
one step, so Isaac's JIT warm-up stall cannot race the wall-clock WMX stream.

The gripper dimension (binary) bypasses WMX and passes through from the
newest chunk unchanged.

Requires (see wmx-r2/doc/lookahead_trajectory_controller.md):
  1. wmx-r2 Franka stack:  ros2 launch wmx_r2_package wmx_r2_franka_manipulator.launch.py  (sudo)
  2. bridge:               python3 <wmx-r2>/wmx_r2_package/scripts/wmx_chunk_bridge.py
"""

import json
import logging
import socket
import time
from collections import deque

import numpy as np

from policies.pi0_family.client import Pi0DroidJointposClient

logger = logging.getLogger(__name__)


class WmxBridgeConnection:
    """Newline-delimited-JSON request/response over a persistent TCP socket."""

    def __init__(self, host: str, port: int, timeout: float = 5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._buf = b""

    def _connect(self):
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        sock.settimeout(self.timeout)
        self._sock = sock
        self._buf = b""

    def request(self, payload: dict) -> dict:
        for attempt in (0, 1):
            try:
                if self._sock is None:
                    self._connect()
                self._sock.sendall(json.dumps(payload).encode() + b"\n")
                while b"\n" not in self._buf:
                    data = self._sock.recv(65536)
                    if not data:
                        raise ConnectionError("bridge closed connection")
                    self._buf += data
                line, self._buf = self._buf.split(b"\n", 1)
                return json.loads(line)
            except (OSError, ConnectionError) as e:
                self._sock = None
                if attempt == 1:
                    raise ConnectionError(f"WMX bridge unreachable at {self.host}:{self.port}: {e}") from e

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None


class WmxPi0DroidJointposClient(Pi0DroidJointposClient):
    """Route pi0-family chunks through the WMX lookahead controller.

    Only ``env_id == 0`` is supported (one WMX engine drives one arm); run
    with ``--num-envs 1``. For ``jit``/``segment`` modes pass
    ``--open-loop-horizon 8`` (below the model's 15-step chunk) so
    consecutive chunks overlap for temporal ensembling.
    """

    def __init__(
        self,
        *args,
        wmx_host: str = "127.0.0.1",
        wmx_port: int = 5555,
        te_decay: float = 0.5,
        stream_mode: str = "jit",
        jit_lead: int = 2,
        stream_full_chunk: bool = False,   # back-compat alias for stream_mode="full"
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        if stream_full_chunk:
            stream_mode = "full"
        if stream_mode not in ("jit", "segment", "full"):
            raise ValueError(f"unknown stream_mode {stream_mode!r}")
        self.bridge = WmxBridgeConnection(wmx_host, wmx_port)
        self.stream_mode = stream_mode
        self.te_decay = float(te_decay)
        self.jit_lead = int(jit_lead)
        # Pacing. segment/full modes: EMA of the wall time spanned by one
        # streamed segment. jit mode: EMA of the wall time per sim step.
        self._nominal_dt = 1.0 / 15.0
        self._chunk_dt_ema = self._nominal_dt
        self._last_chunk_time: float | None = None
        self._step_dt_ema: float | None = None
        self._last_step_time: float | None = None
        self._step = 0                      # global step index (this episode)
        self._streamed_until = -1           # last step index streamed (jit)
        self._history: deque = deque(maxlen=4)  # (start_step, chunk) pairs
        self._warmup_done = False
        reply = self.bridge.request({"cmd": "state"})
        if not reply.get("ok"):
            logger.warning("[WMX] bridge reachable but no /joint_states yet: %s", reply)
        print(
            f"[{self.__class__.__name__}] Connected to WMX bridge at {wmx_host}:{wmx_port} "
            f"(mode={self.stream_mode}, horizon={self.open_loop_horizon}, "
            f"te_decay={self.te_decay}, jit_lead={self.jit_lead})."
        )

    # ------------------------------------------------------------------

    def infer(self, obs, instruction: str, *, env_id: int = 0) -> dict:
        if env_id != 0:
            raise ValueError("WMX mode supports a single env (run with --num-envs 1)")

        extracted = self._extract_observation(obs, env_id=env_id)

        if not self._warmup_done:
            # First step after reset: Isaac's JIT warm-up may stall this
            # env.step for seconds while WMX runs on the wall clock. Align
            # WMX with the sim pose and hold still for one step; streaming
            # begins on the next call.
            reply = self.bridge.request({
                "cmd": "preset",
                "positions": [float(v) for v in extracted["joint_position"][:7]],
            })
            if not reply.get("ok"):
                logger.warning("[WMX] preset failed: %s", reply)
            self._warmup_done = True
            hold = np.zeros(8)
            hold[:7] = np.asarray(extracted["joint_position"][:7], dtype=np.float64)
            hold[7] = 1.0 if float(np.ravel(extracted["gripper_position"])[0]) > 0.5 else 0.0
            return {"action": hold, "viz": self._build_visualization(extracted)}

        self._update_step_pacing()

        if self._needs_refresh(env_id):
            request = self._pack_request(extracted, instruction)
            response = self._query_server(request)
            chunk = self._postprocess_chunk(self._unpack_response(response))
            self._set_chunk(env_id, chunk)
            self._history.append((self._step, np.asarray(chunk, dtype=np.float64)))
            if self.stream_mode in ("segment", "full"):
                self._stream_segment(self._step)

        if self.stream_mode == "jit":
            self._stream_jit()

        # Advance the chunk counter (drives re-inference cadence) and keep the
        # newest VLA action as fallback + gripper source.
        vla_action = self._next_action(env_id)
        action = np.array(vla_action, dtype=np.float64, copy=True)
        self._step += 1

        # Arm follows the WMX commanded position instead of the raw VLA action.
        reply = self.bridge.request({"cmd": "state"})
        if reply.get("ok"):
            action[:7] = np.asarray(reply["positions"], dtype=np.float64)
        else:
            logger.warning("[WMX] no joint state (%s); falling back to raw VLA action", reply)

        return {"action": action, "viz": self._build_visualization(extracted)}

    # ------------------------------------------------------------------

    def _update_step_pacing(self) -> None:
        """EMA of the wall time per sim step, excluding extreme stalls."""
        now = time.monotonic()
        if self._last_step_time is not None:
            dt = now - self._last_step_time
            if self._step_dt_ema is None:
                self._step_dt_ema = max(dt, self._nominal_dt)
            elif dt <= 3.0 * self._step_dt_ema:
                self._step_dt_ema = 0.8 * self._step_dt_ema + 0.2 * dt
        self._last_step_time = now

    def _ensembled_position(self, step: int) -> np.ndarray | None:
        """Weighted average of all stored chunks' predictions for ``step``."""
        total_w = 0.0
        acc = np.zeros(7)
        # newest chunk first -> weight 1, each older chunk decayed by te_decay
        for rank, (t0, chunk) in enumerate(reversed(self._history)):
            i = step - t0
            if 0 <= i < len(chunk):
                w = self.te_decay ** rank
                acc += w * chunk[i, :7]
                total_w += w
        if total_w == 0.0:
            return None
        return acc / total_w

    def _stream_jit(self) -> None:
        """Top the WMX buffer up to ``jit_lead`` points ahead of the sim."""
        points = []
        s = self._streamed_until + 1
        while s <= self._step + self.jit_lead:
            q = self._ensembled_position(s)
            if q is None:
                break
            points.append([float(v) for v in q])
            self._streamed_until = s
            s += 1
        if not points:
            return
        self.bridge.request({
            "cmd": "chunk",
            "positions": points,
            "dt": self._step_dt_ema or self._nominal_dt,
        })

    def _stream_segment(self, start_step: int) -> None:
        """Stream one segment (ensembled horizon or full chunk) per re-plan."""
        if self.stream_mode == "full":
            _, chunk = self._history[-1]
            points = [[float(v) for v in q[:7]] for q in chunk]
        else:
            points = []
            for i in range(self.open_loop_horizon):
                q = self._ensembled_position(start_step + i)
                if q is None:  # cannot happen while the newest chunk covers i
                    break
                points.append([float(v) for v in q])
        if not points:
            return

        now = time.monotonic()
        if self._last_chunk_time is not None:
            dt = (now - self._last_chunk_time) / max(len(points), 1)
            if dt <= 4.0 * self._nominal_dt:
                # Exclude extreme stalls (sim hiccups) from the pacing EMA.
                dt = max(dt, 0.25 * self._nominal_dt)
                self._chunk_dt_ema = 0.7 * self._chunk_dt_ema + 0.3 * dt
        self._last_chunk_time = now

        self.bridge.request({
            "cmd": "chunk",
            "positions": points,
            "dt": self._chunk_dt_ema,
        })

    # ------------------------------------------------------------------

    def reset(self, *, env_id: int | None = None) -> None:
        try:
            self.bridge.request({"cmd": "stop"})
        except ConnectionError:
            logger.exception("[WMX] failed to stop lookahead stream on reset")
        self._last_chunk_time = None
        self._chunk_dt_ema = self._nominal_dt
        self._step_dt_ema = None
        self._last_step_time = None
        self._step = 0
        self._streamed_until = -1
        self._history.clear()
        self._warmup_done = False
        super().reset(env_id=env_id)

    def close(self) -> None:
        try:
            self.bridge.request({"cmd": "stop"})
        except ConnectionError:
            pass
        self.bridge.close()
        super().close()
