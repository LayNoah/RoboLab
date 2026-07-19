"""Pi0 client variant that executes action chunks through a WMX motion controller.

Instead of applying the raw VLA chunk actions directly to the simulated arm,
this client streams chunks to the wmx-r2 ``lookahead_trajectory_controller``
(via the ``wmx_chunk_bridge.py`` TCP bridge — rclpy cannot be imported here
because Isaac Lab runs Python 3.11 while ROS 2 Jazzy ships Python 3.12
bindings) and returns the WMX *commanded joint positions* as the per-step arm
action. The Isaac Lab arm then tracks WMX output, so the sim shows exactly the
motion the motion controller would produce on real hardware.

Latency/smoothness refinements over naive full-chunk streaming:

- **Partial chunk streaming**: only the first ``open_loop_horizon`` points of
  each chunk are streamed, so re-inference happens before the previous chunk
  is exhausted and the WMX queue depth (and therefore the control lag) stays
  at roughly one horizon instead of one full chunk.
- **Temporal ensembling** (ACT-style): with re-inference every H < chunk_len
  steps, consecutive chunks overlap; each streamed point is a weighted average
  of every stored chunk's prediction for that timestep (newest weight 1, each
  older chunk decayed by ``te_decay``). This removes re-planning jumps at the
  source, allowing a shorter WMX smoothing filter.
- **Warm-up hold**: the first env step after a reset triggers Isaac's JIT
  warm-up and can stall for seconds while WMX keeps running on the wall
  clock. The first ``infer`` therefore only presets WMX to the sim pose and
  holds; streaming starts from the second step. Extreme stalls are also
  excluded from the chunk-pacing EMA.

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
    with ``--num-envs 1``. Pass ``--open-loop-horizon 8`` (or similar, below
    the model's 15-step chunk) to enable overlap for temporal ensembling and
    to halve the streaming lag.
    """

    def __init__(
        self,
        *args,
        wmx_host: str = "127.0.0.1",
        wmx_port: int = 5555,
        te_decay: float = 0.5,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.bridge = WmxBridgeConnection(wmx_host, wmx_port)
        self.te_decay = float(te_decay)
        # Adaptive chunk point spacing: each streamed segment should span the
        # wall time the sim takes to consume it (open_loop_horizon steps).
        # EMA of the observed inter-chunk wall time, seeded at nominal 15 Hz.
        self._nominal_dt = 1.0 / 15.0
        self._chunk_dt_ema = self._nominal_dt
        self._last_chunk_time: float | None = None
        self._step = 0                      # global step index (this episode)
        self._history: deque = deque(maxlen=4)  # (start_step, chunk) pairs
        self._warmup_done = False
        reply = self.bridge.request({"cmd": "state"})
        if not reply.get("ok"):
            logger.warning("[WMX] bridge reachable but no /joint_states yet: %s", reply)
        print(
            f"[{self.__class__.__name__}] Connected to WMX bridge at {wmx_host}:{wmx_port} "
            f"(horizon={self.open_loop_horizon}, te_decay={self.te_decay})."
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

        if self._needs_refresh(env_id):
            request = self._pack_request(extracted, instruction)
            response = self._query_server(request)
            chunk = self._postprocess_chunk(self._unpack_response(response))
            self._set_chunk(env_id, chunk)
            self._history.append((self._step, np.asarray(chunk, dtype=np.float64)))
            self._stream_segment(self._step)

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

    def _stream_segment(self, start_step: int) -> None:
        """Stream the next ``open_loop_horizon`` ensembled points to WMX."""
        horizon = self.open_loop_horizon
        points = []
        for i in range(horizon):
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
        self._step = 0
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
