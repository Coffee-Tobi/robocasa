"""Minimal headless MuJoCo rendering test for GitHub Codespaces.

Run from the repository root:
    MUJOCO_GL=osmesa python barista_tests/render_check.py
"""
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

XML = """
<mujoco model="barista_render_check">
  <visual>
    <headlight ambient="0.7 0.7 0.7" diffuse="0.8 0.8 0.8" specular="0 0 0"/>
    <global offwidth="640" offheight="480"/>
  </visual>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.7 0.7 0.7 1"/>
    <body name="cube" pos="0 0 0.15">
      <geom type="box" size="0.15 0.15 0.15" rgba="0.1 0.4 1 1"/>
    </body>
    <light pos="1 -1 3" dir="-0.3 0.3 -1" directional="true"/>
    <camera name="test_camera" pos="1.5 -1.5 1.2"
            xyaxes="0.7071068 0.7071068 0 -0.3313 0.3313 0.8835"/>
  </worldbody>
</mujoco>
"""

def main():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    with mujoco.Renderer(model, height=480, width=640) as renderer:
        renderer.update_scene(data, camera="test_camera")
        pixels = renderer.render().copy()

    output = Path(__file__).with_name("render_check.png")
    Image.fromarray(pixels).save(output)
    print(f"Bild gespeichert: {output}")
    print(f"Pixelwerte: min={pixels.min()}, max={pixels.max()}, mittel={pixels.mean():.1f}")
    print(f"Anteil fast schwarzer Pixel: {np.mean(np.all(pixels < 5, axis=2)):.1%}")
    if pixels.max() == 0:
        print("WARNUNG: Renderer liefert nur schwarze Pixel.")
    else:
        print("Rendering abgeschlossen. Bitte Bild im VS-Code-Explorer öffnen.")

if __name__ == "__main__":
    main()
