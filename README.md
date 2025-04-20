# px4-offboard
This `repository` contains a python examples for offboard control on ROS2 with [PX4](https://px4.io/)

The `px4_offboard` package contains the following nodes
- `offboard_control.py`: Example of offboard position control using position setpoints
- `visualizer.py`: Used for visualizing vehicle states in Rviz

The source code is released under a BSD 3-Clause license.

- **Author**: Jaeyoung Lim
- **Affiliation**: Autonomous Systems Lab, ETH Zurich

## Modification Note
- **Modified by**: Jinrae Kim, kjl950403@gmail.com

The [upstream repo](https://github.com/Jaeyoung-Lim/px4-offboard) provides a concise example of offboard control in Python.
While following up the [upstream repo](https://github.com/Jaeyoung-Lim/px4-offboard), I found that the default implementation results in a huge tracking error.

For future followers who are interested in tracking control design, I compared the following three tracking modes for position tracking:
    - `precise` (designed here): This controller uses acceleration as a control input, and exploits feedforward terms with exponential stability. 
    - `with_ff`: Injecting feedforward terms in velocity and acceleration; feedforward terms are critical for tracking control performance.
    - `default` ([upstream repo](https://github.com/Jaeyoung-Lim/px4-offboard) implementation): No feedforward term presented.

### Additional Notes
- I am not familiar with PX4 offboard control. Please feel free to let me know if the implementation is incorrect particularly for `with_ff` and `default`.
- This is only tested for SITL.
- Please visit the [upstream repo](https://github.com/Jaeyoung-Lim/px4-offboard) for setup.


## Results at a glance

### `precise` (designed here)
![Alt text](./gifs/precise.gif)

### `with_ff` (with feedforward terms)
![Alt text](./gifs/with_ff.gif)

### `default` (original implementation in the upstream repo)
![Alt text](./gifs/default.gif)
