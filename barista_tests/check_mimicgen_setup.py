"""Check exact prerequisites for RoboCasa MimicGen before big downloads.
Run: python barista_tests/check_mimicgen_setup.py
No packages installed, no assets downloaded.
"""
from pathlib import Path
from importlib.util import find_spec
import urllib.request
import json
import shutil

ROOT = Path("/workspaces")
print("MimicGen quick check", flush=True)
for pkg in ("robocasa", "robosuite", "mimicgen", "robomimic", "h5py"):
    try:
        found = find_spec(pkg)
        print(f"{pkg:12s}: {'installed' if found else 'MISSING'}", flush=True)
    except Exception as exc:
        print(f"{pkg:12s}: error {exc}", flush=True)
for repo in ("mimicgen", "robomimic"):
    p = ROOT / repo
    print("Local repo", repo, "present" if p.is_dir() else "MISSING", flush=True)
free = shutil.disk_usage(ROOT).free / 1024**3
print(f"Free Codespace disk: {free:.2f} GiB", flush=True)
dataset_candidates = list((ROOT / "robocasa/datasets").rglob("*.hdf5")) if (ROOT / "robocasa/datasets").exists() else []
print("Available HDF5 datasets:", len(dataset_candidates), flush=True)
for p in dataset_candidates[:5]:
    print("  ", p, flush=True)
repo = "robocasa-benchmark/mimicgen"
paths = (
    "mimicgen/scripts/prepare_src_dataset.py",
    "mimicgen/scripts/generate_dataset_multicore.py",
    "mimicgen/exps/templates/robocasa/single_stage/kitchen_pnp/PnPCounterToSink.json",
    "mimicgen/exps/templates/robocasa/single_stage/kitchen_pick_place/PickPlaceCounterToSink.json",
)
print("Official RoboCasa MimicGen GitHub file availability:",flush=True)
for path in paths:
    req = urllib.request.Request(
        "https://api.github.com/repos/" + repo + "/contents/" + path,
        headers={"User-Agent": "robocasa-mimicgen-preflight", "Accept":"application/vnd.github+json"})
    try:
        with urllib.request.urlopen(req, timeout=12) as response:
            print(" OK     ", path, flush=True)
    except Exception as exc:
        print(" MISSING/UNKNOWN ", path, type(exc).__name__, str(exc)[:70], flush=True)
print("Check complete. No installation or simulation attempted.",flush=True)
