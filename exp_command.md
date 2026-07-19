# VLA + WMX Experiment Commands
Run pi0-family policies with a motion controller in the loop (WMX lookahead or ros2_control JTC), harvest/replay real action chunks, and record motion quality.

## Execution Procedure

### Step 1: Run policy server
```
cd ~/test/RoboLab/openpi
XLA_PYTHON_CLIENT_MEM_FRACTION=0.5 uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi05_droid_jointpos \
    --policy.dir=gs://openpi-assets-simeval/pi05_droid_jointpos
```
wait for `server listening on 0.0.0.0:8000`

### Step 2a: Run WMX stack (sudo)
check `~/workspaces/movensys_ws/src/wmx-r2/doc/lookahead_trajectory_controller.md`
```
sudo --preserve-env=PATH --preserve-env=AMENT_PREFIX_PATH --preserve-env=COLCON_PREFIX_PATH \
     --preserve-env=PYTHONPATH --preserve-env=LD_LIBRARY_PATH --preserve-env=ROS_DISTRO \
     --preserve-env=ROS_VERSION --preserve-env=ROS_PYTHON_VERSION --preserve-env=ROS_DOMAIN_ID \
     --preserve-env=RMW_IMPLEMENTATION \
     bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && source $HOME/workspaces/movensys_ws/install/setup.bash && \
     ros2 launch wmx_r2_package wmx_r2_franka_manipulator.launch.py"
```
```
python3 ~/workspaces/movensys_ws/src/wmx-r2/wmx_r2_package/scripts/wmx_chunk_bridge.py --port 5555
```

### Step 2b: Run ros2_control JTC stack (baseline, no sudo)
```
ros2 launch wmx_r2_package ros2c_baseline.launch.py
```
```
python3 ~/workspaces/movensys_ws/src/wmx-r2/wmx_r2_package/scripts/ros2c_chunk_bridge.py --port 5556
```

### Step 3: Run evaluation

#### WMX lookahead (JIT streaming, default)
```
uv run python policies/pi0_family/run.py --policy pi05 --task BananaInBowlTask \
    --num-envs 1 --num-runs 1 --enable-subtask --headless \
    --wmx --open-loop-horizon 8
```

#### ros2_control JTC baseline
```
uv run python policies/pi0_family/run.py --policy pi05 --task BananaInBowlTask \
    --num-envs 1 --num-runs 1 --enable-subtask --headless \
    --wmx --wmx-port 5556 --wmx-full-chunk
```

#### Direct (no controller)
```
uv run python policies/pi0_family/run.py --policy pi05 --task BananaInBowlTask \
    --num-envs 1 --num-runs 1 --enable-subtask --headless
```
stream mode variants: `--wmx-stream jit|segment|full`

## Motion Quality Measurement

### Record controller output during an episode (100 Hz)
```
python3 ~/workspaces/movensys_ws/src/wmx-r2/wmx_r2_package/scripts/record_joint_states.py --output episode.csv
```
check single publisher first:
```
ros2 topic info /joint_states
```

### Harvest real policy chunks from an episode
```
CHUNK_LOG_PATH=/tmp/chunks.jsonl uv run python policies/pi0_family/run.py --policy pi05 \
    --task BananaInBowlTask --num-envs 1 --num-runs 1 --enable-subtask --headless
```

### Replay identical chunks into a controller
```
python3 ~/workspaces/movensys_ws/src/wmx-r2/wmx_r2_package/scripts/replay_chunks.py \
    --log /tmp/chunks.jsonl --port 5555 --mode drip --meta-out wmx_meta.json
```
```
python3 ~/workspaces/movensys_ws/src/wmx-r2/wmx_r2_package/scripts/replay_chunks.py \
    --log /tmp/chunks.jsonl --port 5556 --mode chunk --meta-out jtc_meta.json
```

## Results
episode outputs: `~/test/RoboLab/output/<timestamp>_pi05/` (`episode_results.jsonl`, `*.mp4`, `run_N.hdf5`)
```
python3 - <<'EOF'
import json, glob
d = sorted(glob.glob("/home/noah/test/RoboLab/output/*_pi05"))[-1]
for line in open(f"{d}/episode_results.jsonl"):
    r = json.loads(line)
    print(r["run"], "SUCCESS" if r["success"] else "FAIL", r["episode_step"], r["events"])
EOF
```

## Cleanup
```
sudo pkill -9 wmx3_engine; sudo pkill -9 -f wmx_r2_package; pkill -f chunk_bridge
```
if `Maximum number of channels reached` on next launch:
```
for id in $(ipcs -m | awk '$1 ~ /^0x0005/ {print $2}'); do sudo ipcrm -m $id; done
sudo rm -f /dev/shm/*WMX3* /dev/shm/sem.Im*
```
