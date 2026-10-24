# Revised fixture concept: shared grasp geometry

User decision 2026-10-24: **replace** original screw-on portafilter handle; for the Decent pitcher use the **small thermometer hole**, not a clamp around the handle.

## Shared IR40-50 grip
- Same external parallel-jaw datum, keyed orientation and positive longitudinal stop on both tools.
- Size jaw contact span/opening and finger stroke from verified IR40-50 drawing; no assumed dimensions.
- Keep the same tool-center reference and define separate calibrated spout/basket transforms.

## Portafilter
- Replace original screw-on handle with a purpose-made rigid, heat-resistant handle.
- Derive thread pitch, depth, shoulder, torque and tightening direction from manufacturer CAD or measurement.
- Add anti-loosening/anti-rotation solution validated for repeated portafilter lock/unlock cycles.

## Milk pitcher
- Through-hole pin/fastener in thermometer opening with a broad load-spreading backing plate, sealed hygienically if it contacts milk.
- Thermometer function is lost if the hole is occupied unless relocated.
- **HOLD POINT:** thermometer hole and thin pitcher wall may be structurally inadequate for a filled pitcher. Do not treat this as load-rated without manufacturer confirmation or destructive/proof testing. Consider secondary load path to handle if needed.
- Record hole diameter, distance from rim/spout, wall thickness, shape, internal clearance and total wet mass.
- Use food-contact and cleaning-safe materials, inspect crevice/seal and thermal expansion.

## CAD and test gates
1. Obtain exact vendor STEP for portafilter handle and inspect pitcher.
2. Parameterize identical outer grip geometry, different mechanical mount.
3. Export two printable prototype assemblies plus STEP and collision meshes.
4. Test slip, torque, 20s pouring, hot/wet cleaning, fatigue and repeated placement accuracy.

The earlier grasp_fixtures.scad is **superseded conceptually** and must not be used as a final model.
