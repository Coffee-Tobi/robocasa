# FAIRINO / IFM 3D Vision Kit: preliminary sensor sensitivity study

This is **not a physically rendered IFM camera test**. It is a stochastic 20-second sensor-error sensitivity benchmark run locally on 2026-10-22 (500 trials per scenario, 25 Hz). The script and chart are supplied in the conversation as separate artifacts.

Actual IFM O3D303 specifications from the manufacturer:
- 352×264 depth pixels, max 25 Hz, nominal FoV 60×45°, working distance 0.3–8 m.
- Near field 0.3–1.0 m: single-pixel depth repeatability ±8 mm (1σ) on 18% reflectivity gray targets; typical accuracy ±7 mm; relative accuracy ±4 mm.
  https://www.ifm.com/in/en/product/O3D303.html
- FAIRINO kit may vary: confirm precise SKU and calibration options first.
  https://fairino.es/productos/kit-vision-3d-ifm/

**Synthetic assumptions (NOT IFM specifications)**:
- 20 s, 25 Hz synthetic 3D path (4 / 2.4 / 1.5 mm axis amplitudes).
- Optimistic independent-pixel averaging reduces 8-mm standard deviation by sqrt(effective pixels). Re-use that conservative depth-derived scale on all axes ONLY to stress-test a 3D location estimator, not to predict true lateral error.
- Independent common-mode frame noise and fixed bias Gaussian standard deviations shown below.
- No rigid-object geometry fitting, time filter, camera image, multipath/reflection simulation, robot dynamics, or liquid flow.

| Scenario | Median maximum error | Median RMS error | Fraction of 500 trials with max <1 mm |
| --- | ---: | ---: | ---: |
| 100 independent pixels, zero common noise/bias | 3.19 mm | 1.38 mm | 0 |
| 100 pixels, 1-mm common noise and 1-mm calibration bias | 5.91 mm | 2.70 mm | 0 |
| 25 pixels, 2-mm common noise and 2-mm bias | 11.82 mm | 5.34 mm | 0 |
| One pixel, 2-mm common noise and 2-mm bias | 33.08 mm | 14.60 mm | 0 |

Interpretation: with this *specific illustrative* noise/geometry model, instantaneous camera-only tracking does not meet the <1-mm worst-case goal. This is **NOT an empirical claim that IFM cannot achieve sub-mm object localization** with optimized fitting, averaging, target choice, or calibration. Object-detection software performance is unknown. Test the kit for cup/pitcher initial pose and workspace detection; retain encoders/robot inverse dynamics for 250–500Hz pouring control.

Next proper experiment: download measured IFM range frames or acquire manufacturer sample point cloud, implement actual O3D camera intrinsics, realistic object geometry, robot-camera extrinsics, target fitting and temporal filtering. Compare matched-camera target poses with existing 3-camera RGB-D setup on same scene with manufacturer-defined noise.
