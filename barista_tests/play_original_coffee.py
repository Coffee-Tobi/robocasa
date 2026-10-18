"""Download and replay ONE ORIGINAL NVIDIA RoboCasa PrepareCoffee demonstration.

Uses RoboCasa's existing official playback_dataset.py, not handmade robot actions.
Run: MUJOCO_GL=osmesa NUMBA_DISABLE_JIT=1 OPENBLAS_NUM_THREADS=1 python barista_tests/play_original_coffee.py
"""
from pathlib import Path
import json
import shutil
import subprocess
import urllib.request

ROOT = Path("/tmp/robocasa_coffee_original")
EXTRAS = ROOT / "extras"
EP = EXTRAS / "episode_000000"
OUTPUT = Path(__file__).resolve().parent / "original_prepare_coffee.mp4"
BASE = ("https://huggingface.co/datasets/"
        "nvidia/PhysicalAI-Robotics-Manipulation-Kitchen-Demos/resolve/main/"
        "target/composite/PrepareCoffee/20250812/lerobot/extras/")
FILES = (
    ("dataset_meta.json", EXTRAS / "dataset_meta.json"),
    ("episode_000003/ep_meta.json", EP / "ep_meta.json"),
    ("episode_000003/model.xml.gz", EP / "model.xml.gz"),
    ("episode_000003/states.npz", EP / "states.npz"),
)

for source, dest in FILES:
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.is_file() and dest.stat().st_size > 100:
        print("Bereits vorhanden:", dest.name, flush=True)
        continue
    print("Lade original:", source, flush=True)
    req = urllib.request.Request(BASE + source + "?download=true",
                                 headers={"User-Agent":"Mozilla/5.0"})
    tmp = dest.with_suffix(dest.suffix + ".download")
    try:
        with urllib.request.urlopen(req, timeout=120) as response, open(tmp,"wb") as out:
            shutil.copyfileobj(response,out)
        tmp.replace(dest)
    except Exception as exc:
        tmp.unlink(missing_ok=True)
        raise RuntimeError("Download fehlgeschlagen (" + source + "). " + str(exc)) from exc
    print("  Groesse:", round(dest.stat().st_size / 1e6, 2), "MB", flush=True)

import numpy as np
import robocasa
from robocasa.scripts.dataset_scripts.playback_dataset import playback_dataset

with np.load(EP / "states.npz") as d:
    print("Originale Simulationsframes:", d["states"].shape, flush=True)
with open(EXTRAS / "dataset_meta.json") as f:
    metadata = json.load(f)
print("Originale Umgebung:", metadata["env_args"]["env_name"], flush=True)
print("Starte RoboCasa-Originalwiedergabe ...", flush=True)

# Uses exact original recorded Mujoco states and XML; not env.step(actions).
playback_dataset(
    dataset=ROOT, use_actions=False, use_abs_actions=False,
    use_obs=False, filter_key=None, n=1, render=False,
    render_image_names=["robot0_agentview_right"],
    camera_height=480, camera_width=640,
    video_path=str(OUTPUT), video_skip=4,
    extend_states=False, first=False, verbose=True,
)
print("Demo-Video:", OUTPUT, flush=True)
