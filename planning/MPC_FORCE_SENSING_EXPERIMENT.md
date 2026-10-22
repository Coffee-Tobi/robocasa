# MPC / dampflanze contact – isolated experiment

Branch: experiment/mpc-force-sensing

Current status: independent MPC optimizer and MuJoCo contact-force reader created. Three MPC numerical tests pass locally. NOT integrated into the robot motor loop; NOT verified on full FR5 simulation and no sub-mm claim.

MPC: linear double-integrator, 12-step horizon at 20 ms, acceleration bounds (virtual limits). Use the existing full-dynamics mj_inverse torque command downstream. Future integration must preview the reference joint path at each MPC step, enforce joint/torque limits, compare same trajectories with computed_torque_robust and test motor saturation. The MPC is not full model predictive control of nonlinear manipulator/contact dynamics.

Sensor: read_pitcher_wand_contacts uses mj_contactForce on MuJoCo physics contacts. The current pitcher visual clone explicitly has collision disabled: contype=conaffinity=0. Before testing steam-wand contact, create separate collision-enabled thin-wall pitcher geometry and steam-wand geometry, then set up a safe force-limited approach. No force feedback result exists yet.

Proposed future 20 s pour: track spout tip max <1 mm, RMS <0.5 mm, orientation separately, test 0.0/0.5/1.0 Nm and variable payload mass. Camera recording at 60 FPS is different from 1 kHz physics and 250-500 Hz motor control. Aim for 60 FPS playback, keep controls independent. Check actual achieved Ruckig re-time, not nominal input duration.

To run tests: python -m unittest -v tests.test_mpc_control
