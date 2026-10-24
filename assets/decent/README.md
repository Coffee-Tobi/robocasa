# Decent original CAD and robot-grasp fixture prototype

This branch adds **parametric OpenSCAD prototypes**, not validated fixtures or finished collision meshes. All fit dimensions in the SCAD file are placeholders. Do not print or use on a hot/filled container without measurement and mechanical testing.

## Vendor CAD
- Bengle original Onshape and STEP: https://decentespresso.com/overview (CAD drawings > Bengle).
- Bengle overall drawing: https://decentespresso.com/img/bengle_dimensions.pdf . Check actual revision against STEP: published drawings and descriptive webpages are not necessarily identical.
- Decent portafilter handle STEP/DXF/PDF: https://decentespresso.com/docs/drawings_of_the_portafilter_and_group_head_handles
- Milk jug thermometer-hole 350/600ml product listings: https://decentespresso.com/parts?filter=DE-MILKJUGALL3CLASSICV2-75705 . Capacity/package size is **not** a fully dimensioned pitcher CAD drawing.

## Workflow
1. Download original STEP locally; preserve the manufacturer's author/license notice. Verify redistribution rights **before committing binary CAD into this repository**.
2. Import STEP into FreeCAD and export *separate* simplified meshes, with origin, units (m for MuJoCo) and collision geometry documented.
3. Measure the particular milk-jug handle at several sections, curvature and attachment points, thermometer hole position, spout-tip pose, pitcher mass and full-fill mass. The pitcher handle is **not** a verified rectangle.
4. For the portafilter, use the vendor STEP to measure a grip section, taper, mass, rotation / locking torque, and an axial stop feature. Verify that the handle STEP matches the chosen portafilter.
5. Edit `grasp_fixtures.scad` parameters; export each half separately using OpenSCAD. The current negative cavities illustrate fit only; align and redesign screw bosses and robot contact geometry after inspecting real dimensions.
6. Add soft washable heat-resistant liners, positive axial stops and keyed orientation, not friction alone. Keep the gripper away from hot surfaces and the thermometer opening.
7. Validate static torque, wet slip, pickup repeatability, 20-second pouring trajectory, tool-camera occlusion, IR40-50 opening/force, and tip pose after repeated removals.
8. **Important**: the current assembly is an FR5 + 90-degree dual adapter + two IR40-50 grippers, with an unverified F/T proxy; do not replace current side-grasp orientation.

## Known limits
No vendor original CAD binaries have been copied here. This model makes no dimensional, food safety or mechanical performance claim. For a real hot milk pitcher use a positively retained attachment rather than a general squeeze-only jaw.
