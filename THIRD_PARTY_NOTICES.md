# Third-Party Notices

`ax-ros2-ref` bundles third-party source code. This file records where that code came
from, under what license, and how it has been changed. Nothing here overrides the license
text shipped inside each bundled component — where the two disagree, the bundled license
file governs that subtree.

---

## Niryo NED ROS2 Driver — `ros2_ws/src/ned-ros2-driver/`

| | |
| :--- | :--- |
| Upstream | [NiryoRobotics/ned-ros2-driver](https://github.com/NiryoRobotics/ned-ros2-driver) |
| Version | v1.0.3 (27 October 2025), per the bundled [`CHANGELOG`](ros2_ws/src/ned-ros2-driver/CHANGELOG) |
| Copyright | © Niryo Robotics |
| License | BSD 3-Clause — [`LICENSE`](ros2_ws/src/ned-ros2-driver/LICENSE) |
| Modified | Yes — see below |

Bundled into this repository on 29 March 2026 as a source copy rather than a git
submodule. The exact upstream commit was not recorded at import time; the version above
is read from the bundled changelog.

### Modifications

The bundled copy is **not** stock upstream. Relative to the imported tree, changes are
confined to the following files:

- `niryo_ned_moveit_configs/niryo_ned2_moveit_config/` — RViz, MoveIt controller and
  `ros2_controllers` configuration retuned; `launch/ned2_moveit_launch.py` reworked and
  `launch/ned2_moveit_real.launch.py` added to drive real hardware over ROSBridge
- `niryo_ned_ros2_driver/config/whitelist.yaml` — interface whitelist widened to expose
  the gripper and tool services used by `metis_moveit2_perception`
- `niryo_ned_ros2_driver/config/drivers_list.yaml` — default robot hostname

Driver bugs should be reported upstream only after confirming they reproduce against an
unmodified upstream checkout.

### Notes

- Upstream's `LICENSE` file carries the BSD 3-Clause terms but no copyright line. The
  attribution recorded above is taken from the upstream project's README and repository.
- Upstream's `.gitlab-ci.yml` and `.gitlab/` directory are retained so the copy stays
  faithful. They target Niryo's own GitLab instance and play no part in this repository.

---

## Licensing of Axelera-authored code

The root [`LICENSE`](LICENSE) (MIT, © Axelera AI) covers repository-level files. Each
ROS2 package under `ros2_ws/src/` declares its own license in its `package.xml`:

| Package | License |
| :--- | :--- |
| `metis_ros2_inference` | Apache-2.0 ([`LICENSE`](ros2_ws/src/metis_ros2_inference/LICENSE)) |
| `metis_pose_inference` | Apache-2.0 |
| `axros_helper_nodes` | Apache-2.0 |
| `metis_moveit2_perception` | MIT |

## Upstream dependencies not bundled here

The Voyager SDK, ROS2 Jazzy, MoveIt2 and `vision_msgs` are external dependencies
installed separately. They are not redistributed by this repository and are governed by
their own licenses.
