# Maps

This directory contains the task 1 navigation map.

The current task 1 scene reuses the calibrated field coordinate system from
`ros2_ws/src/ican_land_sim`. Ground patrol points, window barriers, aerial
inspection points, and the command center are aligned to that map.

`inspection_task1_map.yaml` covers the full task field used by RViz/Nav2:

- resolution: `0.025 m/pixel`
- origin: `[-0.9, -3.5, 0.0]`
- image size: `212 x 176`
- world coverage: about `5.30 m x 4.40 m`

The static map marks the outer walls and window barriers as occupied. Task
markers remain free space so Nav2 can plan to them.
