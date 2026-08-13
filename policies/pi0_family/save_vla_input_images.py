"""Save the two images that actually go into the VLA, from a banana episode.

Runs the BananaInBowlTask sim (no policy server needed), steps until the arm
is near the banana, and dumps the exact images the client feeds pi0.5:
  - over_shoulder_left_camera  -> observation/exterior_image_1_left
  - wrist_cam                  -> observation/wrist_image_left
both as raw sensor frames AND as the 224x224 resize_with_pad the model sees.
"""

import os

import cv2  # noqa: F401 -- import before isaaclab
from isaaclab.app import AppLauncher

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--out", default="/home/noah/recording/example")
parser.add_argument("--step", type=int, default=40, help="sim step to capture at")
from robolab.eval.runner import add_common_eval_args  # noqa: E402
add_common_eval_args(parser)  # provides --task (nargs="+"), --task-dirs, etc.
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

import robolab.constants  # noqa: E402
from robolab.registrations.droid.auto_env_registrations_jointpos import auto_register_droid_envs  # noqa: E402
from robolab.core.environments.runtime import create_env  # noqa: E402
from robolab.core.environments.factory import get_envs  # noqa: E402
from openpi_client import image_tools  # noqa: E402

robolab.constants.ENABLE_SUBTASK_PROGRESS_CHECKING = getattr(args_cli, "enable_subtask", False)

auto_register_droid_envs(task_dirs=args_cli.task_dirs, task=args_cli.task)

# use RoboLab's own env creation path (same as the eval runner)
task_env = get_envs(task=args_cli.task)[0]
env, env_cfg = create_env(task_env, num_envs=1, policy="pi05")
env = env.unwrapped
obs, _ = env.reset()
obs, _ = env.reset()

import omni.timeline  # noqa: E402
timeline = omni.timeline.get_timeline_interface()

# step a bit so the arm approaches the banana (deterministic zero action)
import torch  # noqa: E402
action_dim = env.action_space.shape[-1]
for i in range(args_cli.step):
    while not timeline.is_playing():
        app_launcher.app.update()
    obs, *_ = env.step(torch.zeros(env.num_envs, action_dim, device=env.device))

os.makedirs(args_cli.out, exist_ok=True)


def to_uint8(arr):
    arr = np.asarray(arr)
    if arr.dtype != np.uint8:
        arr = (np.clip(arr, 0, 1) * 255).astype(np.uint8) if arr.max() <= 1.0 else arr.astype(np.uint8)
    return arr


shoulder = to_uint8(obs["image_obs"]["over_shoulder_left_camera"][0].detach().cpu().numpy())
wrist = to_uint8(obs["image_obs"]["wrist_cam"][0].detach().cpu().numpy())

for name, raw in [("exterior_over_shoulder_left", shoulder), ("wrist", wrist)]:
    Image.fromarray(raw).save(f"{args_cli.out}/{name}_raw_{raw.shape[1]}x{raw.shape[0]}.png")
    model_in = image_tools.resize_with_pad(raw, 224, 224)
    Image.fromarray(to_uint8(model_in)).save(f"{args_cli.out}/{name}_vla_input_224x224.png")
    print(f"saved {name}: raw {raw.shape} + 224x224 model input")

simulation_app.close()
