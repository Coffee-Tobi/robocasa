# IFM vs RealSense D455: improved synthetic AprilTag/depth comparison (22 Oct 2026)

The Inlux 3D kit stock code is `IVISIX1000O3D302`, suggesting ifm O3D302 (176×132 pixels); a different O3D303 version would be 352×264. **Confirm exact delivered model before purchasing.**

The RealSense D455 is a representative Intel RealSense RGB-D camera. Reports associate Artly with the RealSense family, **not a verified D455 SKU**.

New local reproducible test in this conversation: synthetic 6D tag-corner projection **plus simulated depth at each visible tag center**, three 90-mm tags with orientations from our existing RoboBarista pitcher fixture, actual planned camera positions, field of view and nominal resolutions. Trajectory is 20s sampled at **60 pose timestamps**, 2 random seeds. This is *not* the original pixel-rendered MuJoCo pipeline or real camera acquisition; all noise and calibration parameters are **assumptions**, not measured device performance.

| Camera | Median of per-run p95 spout error | p95 orientation |
|---|---:|---:|
| 1× ifm O3D302 close | 5.96 mm | 2.59° |
| 1× ifm O3D303 close | 5.44 mm | 1.49° |
| 1× RealSense D455 close | 3.03 mm | 0.48° |
| 1× RealSense D455 from existing right viewpoint | 4.00 mm | 0.52° |
| 3× RealSense D455 existing camera mounts | 1.29 mm | 0.38° |

Depth observation noise, systematic bias, AprilTag infrared contrast and corner noise are *not directly available* from manufacturer's AprilTag benchmarks. Numeric values can shift substantially if assumptions change.

With a more cautious **24-pixel minimum** tag-side criterion, the chosen O3D302 mount at ~0.8 m yields no valid tags; moving it closer (~0.6m target distance) increases tag pixel coverage but still yields illustrative ~9.7mm p95 spout error in one trial. Pixel threshold is an arbitrary assumption, not a certified decoder threshold.

If one of 3 camera streams is deliberately blocked for 3 seconds, 3-camera coverage in this synthetic model remains at 100%; a sole camera has an unobservable 3-second window (85% coverage).

Sources:
- https://inluxrobotics.de/products/3d-vision-kit-fairino-ifm
- https://www.ifm.com/de/en/product/O3D302
- https://www.ifm.com/de/de/product/O3D303
- https://www.realsenseai.com/products/real-sense-depth-camera-d455f/
- https://armdevices.net/2026/01/11/artly-barista-bot-imitation-learning-motion-capture-training-autonomous-latte-art/

Do not claim sub-mm or camera-certified errors. To validate purchasing, get the exact kit SKU and raw recorded synchronized amplitude+depth samples from manufacturer, then rerun our AprilTag detector and robot tracking on measured frames.
