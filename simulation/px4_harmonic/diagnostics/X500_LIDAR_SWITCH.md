# Prepared X500 LiDAR model switch — not executed

Existing simulation is airborne in Hold in the latest telemetry. Obtain explicit
user approval before landing. No flight-mode commands have been issued.

## Proposed landing action (requires user approval)

In the existing PX4 SITL console, issue `commander land` once. This sends
NAV_LAND; confirmed in the installed PX4 source Commander.cpp. Do not disarm in
the air. Observe the landing, then run `bash diagnostics/check_grounded.sh` from
a normal WSL terminal. It only reads PX4 topics and requires fresh landed=true
and DISARMED status for three seconds. Wait for automatic disarming if necessary.
If the check does not pass, do not stop or replace the simulator.

## Model-switch plan after verified landing

1. Stop only the existing PX4 SITL and Gazebo instances after landed+disarmed is
   confirmed. Keep the agent and ROS telemetry bridge terminals available.
2. Start the separately prepared world `gazebo_worlds/hacktx_forest_x500_sando.sdf`.
   It copies the working forest and includes the additive `x500_sando` model as
   `x500_0` at (0,0,0.3), with a 3D LiDAR fixed 0.30 m above base_link.
3. Attach PX4 to that already spawned model using PX4_GZ_MODEL_NAME=x500_0 with
   the existing gz_x500 airframe. PX4's installed px4-rc.gzsim explicitly supports
   this path and skips spawning a second vehicle. Do not arm or take off.
4. Verify ordinary PX4 sensors and LiDAR publishing on the ground; bridge to
   preview topics, filter self returns, and verify sensor extrinsics and origin
   against simulation poses before connecting live clouds to SANDO.

Both additive model and world pass gz sdf -k with model resources resolved.
Neither has been runtime tested as a PX4-attached vehicle. Original world,
PX4 models, working planner YAML, and original launch commands are preserved.
Rollback uses the original forest world and original make px4_sitl gz_x500
startup after the modified simulation has been stopped while landed/disarmed.

No autonomous-flight approval is implied by landing or model-switch approval.
