"""Pi0 client variant that executes action chunks through a WMX motion controller.

Instead of applying the raw VLA chunk actions directly to the simulated arm,
this client streams each chunk to the wmx-r2 ``lookahead_trajectory_controller``
(via the ``wmx_chunk_bridge.py`` TCP bridge — rclpy cannot be imported here
because Isaac Lab runs Python 3.11 while ROS 2 Jazzy ships Python 3.12
bindings) and returns the WMX *commanded joint positions* as the per-step arm
action. The Isaac Lab arm then tracks WMX output, so the sim shows exactly the
motion the motion controller would produce on real hardware.

The gripper dimension (binary) bypasses WMX and is passed through from the
chunk unchanged.

Requires (in separate terminals, see wmx-r2/doc/lookahead_trajectory_controller.md):
  1. wmx-r2 Franka stack:  ros2 launch wmx_r2_package wmx_r2_franka_manipulator.launch.py  (sudo)
  2. bridge:               python3 <wmx-r2>/wmx_r2_package/scripts/wmx_chunk_bridge.py
"""

import json
import logging
import socket
import time

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
    with ``--num-envs 1``.
    """

    def __init__(self, *args, wmx_host: str = "127.0.0.1", wmx_port: int = 5555, **kwargs):
        super().__init__(*args, **kwargs)
        self.bridge = WmxBridgeConnection(wmx_host, wmx_port)
        # Adaptive chunk point spacing: each chunk should span the wall time
        # the sim takes to consume it (open_loop_horizon steps), so WMX
        # neither starves nor lags when the sim runs off real time. EMA of
        # the observed inter-chunk wall time, seeded with the nominal 15 Hz.
        self._nominal_dt = 1.0 / 15.0
        self._chunk_dt_ema = self._nominal_dt
        self._last_chunk_time: float | None = None
        # WMX simu axes wake up at zero, not at the sim arm's initial pose;
        # teleport them to the observed pose before the first chunk of every
        # episode, otherwise the arm is commanded to jump to the stale WMX pose.
        self._needs_preset = True
        # Fail fast if the bridge is down (state may legitimately be empty).
        reply = self.bridge.request({"cmd": "state"})
        if not reply.get("ok"):
            logger.warning("[WMX] bridge reachable but no /joint_states yet: %s", reply)
        print(f"[{self.__class__.__name__}] Connected to WMX bridge at {wmx_host}:{wmx_port}.")

    def infer(self, obs, instruction: str, *, env_id: int = 0) -> dict:
        if env_id != 0:
            raise ValueError("WMX mode supports a single env (run with --num-envs 1)")

        extracted = self._extract_observation(obs, env_id=env_id)

        if self._needs_preset:
            reply = self.bridge.request({
                "cmd": "preset",
                "positions": [float(v) for v in extracted["joint_position"][:7]],
            })
            if not reply.get("ok"):
                logger.warning("[WMX] preset failed: %s", reply)
            self._needs_preset = False

        if self._needs_refresh(env_id):
            request = self._pack_request(extracted, instruction)
            response = self._query_server(request)
            chunk = self._postprocess_chunk(self._unpack_response(response))
            self._set_chunk(env_id, chunk)
            self._send_chunk_to_wmx(chunk)

        # Advance the chunk counter (drives re-inference cadence) and keep the
        # VLA action as fallback + gripper source.
        vla_action = self._next_action(env_id)
        action = np.array(vla_action, dtype=np.float64, copy=True)

        # Arm follows the WMX commanded position instead of the raw VLA action.
        reply = self.bridge.request({"cmd": "state"})
        if reply.get("ok"):
            action[:7] = np.asarray(reply["positions"], dtype=np.float64)
        else:
            logger.warning("[WMX] no joint state (%s); falling back to raw VLA action", reply)

        viz = self._build_visualization(extracted)
        return {"action": action, "viz": viz}

    def _send_chunk_to_wmx(self, chunk: np.ndarray) -> None:
        now = time.monotonic()
        if self._last_chunk_time is not None:
            span = now - self._last_chunk_time
            dt = span / max(len(chunk), 1)
            # Clamp to a sane range around nominal before smoothing, so one
            # hiccup (e.g. episode reset) cannot poison the pacing.
            dt = min(max(dt, 0.25 * self._nominal_dt), 4.0 * self._nominal_dt)
            self._chunk_dt_ema = 0.7 * self._chunk_dt_ema + 0.3 * dt
        self._last_chunk_time = now

        self.bridge.request({
            "cmd": "chunk",
            "positions": [list(map(float, q[:7])) for q in chunk],
            "dt": self._chunk_dt_ema,
        })

    def reset(self, *, env_id: int | None = None) -> None:
        try:
            self.bridge.request({"cmd": "stop"})
        except ConnectionError:
            logger.exception("[WMX] failed to stop lookahead stream on reset")
        self._last_chunk_time = None
        self._chunk_dt_ema = self._nominal_dt
        self._needs_preset = True
        super().reset(env_id=env_id)

    def close(self) -> None:
        try:
            self.bridge.request({"cmd": "stop"})
        except ConnectionError:
            pass
        self.bridge.close()
        super().close()
