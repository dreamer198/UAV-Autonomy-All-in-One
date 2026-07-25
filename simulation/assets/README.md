# Simulation image assets

This directory contains the local PX4/Gazebo customizations that previously
existed only inside the host-mounted `/root` of the `ros_noetic` container.

The large upstream source trees are fetched by `simulation/Dockerfile` at the
exact commits recorded in `simulation/versions.env`:

- PX4-Autopilot v1.14.3
- Tfly6/Mid360_px4_sim_plugin at commit `44bcd80...`

The Mid360 meshes and scan-pattern CSV files remain in the pinned upstream
repository instead of being duplicated as roughly 180 MB of opaque assets in
this repository. The Docker image copies the required `Mid360` model into the
PX4 Gazebo model path while building.

`px4/models/Mid360/Mid360.sdf` overrides only the small model definition. It
keeps the pinned 18000-ray scan pattern and disables Gazebo ray rendering,
which otherwise makes `gzclient` unnecessarily CPU-bound.
