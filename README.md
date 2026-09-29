# RS1-Gr25

RS1 Forest Management Project (ROS 2 Humble + Ignition Gazebo Fortress).

## Workspace hygiene

`build/`, `install/`, and `log/` are **local-only**. They are gitignored because they contain machine-specific absolute paths/symlinks. After every `git pull`, rebuild on your machine.

```bash
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
cd ~/git/RS1-Gr25   # your clone path
colcon build --symlink-install --packages-select beer_fire_detection 41068_ignition_bringup
source install/setup.bash
```

See `src/41068_ignition_bringup_v1/41068_ignition_bringup/simulation_workflow_and_implementation_overview.md` for the full simulation runbook.
