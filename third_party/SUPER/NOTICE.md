# SUPER vendored source notice

This directory contains a selected source snapshot from the SUPER project:

- Upstream: <https://github.com/hku-mars/SUPER>
- Commit: `2ad3419c127a617c6d7df6925e81a14175a9c096`
- Snapshot paths: `super_planner`, `rog_map`, and
  `mars_uav_sim/mars_quadrotor_msgs`
- Snapshot date in this project: 2026-07-29

The snapshot was produced from the committed Git tree. Uncommitted files in
the source checkout, including its locally modified `README.md` and untracked
`docs/`, were not copied.

Only the ROS 1 Noetic planning core is retained. ROS 2 adapters and messages,
the upstream simulator, mission planner, controller/demo launch files, RViz
profiles, tuning programs, generated logs, and plotting scripts are excluded.
The source layout and ROS package name `quadrotor_msgs` are intentionally
preserved because SUPER's message wire definitions must remain isolated in
its own Catkin workspace.

## Local integration changes

The selected snapshot carries small integration patches:

- thread-safe committed-trajectory clearing and monotonically assigned native
  trajectory IDs;
- synchronous reset and FSM lifecycle epochs so a canceled in-flight replan
  cannot later publish or restore its old trajectory;
- a command-publication barrier that keeps 100 Hz sampling independent of a
  slower replan while still making reset wait for all old command publishers;
- independent heartbeat/progress topics, immediate publication of every FSM
  transition, effective-goal acknowledgement, and a map-backed native
  goal-validation service;
- zero quaternion support for unconstrained yaw;
- a configurable close-to-goal threshold so the native FSM and public
  measured-arrival contract use the same endpoint tolerance;
- configurable ROG-Map TF publication;
- ROS 1 dependency metadata cleanup, including removal of the unused
  `cmake_utils` dependency.

See the Git history of this project for the exact patch relative to the
upstream commit.

## Licensing

SUPER and ROG-Map source files identify themselves as
LGPL-3.0-or-later. A copy of LGPL 3.0 is provided at
`LICENSES/LGPL-3.0.txt`; copyright and attribution headers remain in each
source file.

Some incorporated utility files carry additional MIT or other permissive
notices in their own headers. The embedded cereal/rapidjson/rapidxml license
files are retained under `super_planner/include/cereal/external/`.

The upstream `quadrotor_msgs/package.xml` declares BSD, but the selected
upstream commit does not contain a separate package-level BSD license text or
complete copyright statement. Its BSD declaration and attribution metadata
are preserved. This notice does not replace or reinterpret any file-level
license and does not apply one unified license to the whole snapshot.
