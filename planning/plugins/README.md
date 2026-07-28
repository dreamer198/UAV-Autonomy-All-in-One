# Planner plugin manifests

Each direct child contains one strictly validated `planner.plugin.yaml`.
Planner IDs are stable public identifiers. Because ROS 1 graph names cannot
contain `-`, `ros_namespace` uses the corresponding underscore spelling.

The built-in manifests point at repository-local isolated workspaces:

- `planning/workspaces/interfaces_ws`
- `planning/workspaces/control_ws`
- `planning/workspaces/diff_ws`
- `planning/workspaces/fast_ws`

`list` and schema-only `validate` work before those workspaces are built.
Runtime validation and launch additionally require the selected workspace
`devel/setup.bash`, ROS package, and launch file to exist.

External plugin directories can be appended with the read-only,
colon-separated `SIM2REAL_PLANNER_PLUGIN_PATH` environment variable. Duplicate
IDs are always rejected; external manifests cannot override built-ins.
