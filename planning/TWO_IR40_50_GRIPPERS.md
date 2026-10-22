# Confirmed configuration: two Inlux IR40-50 grippers

- Gripper A: IR40-50 with replaceable 3D-printed straight/form-fitted fingers for pitcher handle and portafilter (tamper integrated into grinder).
- Gripper B: IR40-50 with replaceable 3D-printed contoured cup fingers; must verify 40 mm advertised travel can accommodate cup diameter via lever/offset/geometry and if jaw opening required is available.
- Dual mount: verify supplier flange, reachable tool orientations, total payload, and combined wrist torques.
- Splitter / addressable independent controller channels: confirm from Inlux.
- Force/torque sensor sits between FR5 and dual-mount if mechanically compatible.
- Real cup-taper geometry, material, compliance, grasp force and no-spill transport MUST be tested.
- Vendor STEP file is available on https://inluxrobotics.de/products/fairino-gripper but not yet imported.
- Supplier website has no publicly verified cup-specific STL download; request from Inlux before constructing printable production jaws.

### Current code
`robobarista/dual_ir40_concept.py`: separate MJCF *visual* example of 2 grippers with generic rectangular fingers; not physically actuated and not a precision or gripping benchmark. To inspect use `build_dual_gripper_scene(vendor_dir)` then load with MuJoCo. Next iteration: import vendor STEP dimensions, develop compliant cup jaw mesh and collision geoms, two independently actuated gripper slide joints and pickup tests.

This file documents a user decision, not successful hardware validation.
