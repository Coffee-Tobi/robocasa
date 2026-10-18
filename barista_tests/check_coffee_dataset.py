"""Inspect Hugging Face PrepareCoffee metadata without downloading large data.
Run: python barista_tests/check_coffee_dataset.py
"""
import json
import urllib.request
from urllib.parse import quote

REPO = "jellyho/robocasa365-PrepareCoffee"
BASE = f"https://huggingface.co/api/datasets/{REPO}"
def get_json(url):
    req = urllib.request.Request(url, headers={"User-Agent": "robocasa-audit/1.0"})
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.load(resp)

print("Analysiere LeRobot-Datensatz (nur Metadaten) ...", flush=True)
items = []
url = BASE + "/tree/main?recursive=true"
try:
    while url:
        req = urllib.request.Request(url, headers={"User-Agent": "robocasa-audit/1.0"})
        with urllib.request.urlopen(req, timeout=30) as response:
            data = json.load(response)
            link = response.headers.get("Link", "")
        items += data
        next_part = next((v.strip().split(";")[0].strip("<>") for v in link.split(",") if 'rel="next"' in v), None)
        url = next_part
        if len(items) > 50000:
            break
except Exception as exc:
    print("Dateiliste nicht vollständig abrufbar:", repr(exc), flush=True)

files = [x for x in items if x.get("type") == "file"]
print("Dateien gesamt:", len(files), flush=True)
for x in files:
    p = x.get("path", "")
    if any(token in p.lower() for token in ("meta/", "xml", "state", "parquet", "episode")):
        print(f"  {p} ({x.get('size', 0)/1e6:.1f} MB)", flush=True)

for path in ("meta/info.json", "meta/episodes.jsonl", "meta/tasks.jsonl"):
    url = f"https://huggingface.co/datasets/{REPO}/resolve/main/{quote(path)}"
    try:
        req = urllib.request.Request(url, headers={"Range": "bytes=0-32767"})
        with urllib.request.urlopen(req, timeout=20) as res:
            data = res.read(32768).decode("utf-8", errors="replace")
        print("\n" + path + " (Anfang):", flush=True)
        print(data[:7000], flush=True)
    except Exception as exc:
        print(path, "nicht lesbar:", str(exc), flush=True)

print("\nEinordnung:", flush=True)
paths = [x.get("path", "").lower() for x in files]
print("Explizite XML-Dateien:", sum(p.endswith(".xml") for p in paths), flush=True)
print("Explizite state-Dateien:", sum("state" in p for p in paths), flush=True)
print("WICHTIG: Diese Dateiliste allein beweist keine vollständigen MuJoCo-Zustände.", flush=True)
