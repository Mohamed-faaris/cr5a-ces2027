# Gazebo pick/place simulation

The PGEA Gazebo stack is separate from the fake-hardware MoveIt demo. Stop the `moveit-demo` window before launching this stack; both publish `/move_group`, `/controller_manager`, and robot state in the same ROS domain.

```bash
cd ~/cr5a_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dobot_gazebo pick_place_gazebo.launch.py shelf_profile:=standard object_kind:=coke shelf_level:=1
```

Supported shelf profiles: `compact`, `standard`, `wide`, `deep`. Supported dynamic objects: `coke`, `pringles`, `wood_box`. The world generator creates shelf boards, back panel, platform, and an object model with mass and inertia. It uses the same baseline dimensions and object mass estimates as the task planner. The default shelf is placed at X=0.55 m and rotated 90° so its open front faces the robot; the place platform is at X=0.35 m, Y=-0.35 m.

The launch file uses the custom PGEA URDF and resolves the `gazebo_ros2_control` parameter file from the installed `cr5_moveit` package. The joint-state broadcaster filters the GazeboSystem-only `right_finger_joint_mimic` state name; MoveIt derives the physical right finger from the URDF mimic relation on `left_finger_joint`. It starts the joint-state broadcaster, arm trajectory controller, gripper action controller, simulation telemetry adapter, MoveIt and RViz. The physical controller's `SetPayload` service is simulated by `dobot_simulation`; this does not prove the real robot's payload or tool-frame calibration.

Gazebo Classic verifies rigid-body contact and the MoveIt planning scene verifies collision-free robot paths. A successful MoveIt plan does not prove stable grasp retention, finger force, friction, or real hardware safety. Do not execute the generated task on hardware without a guarded validation procedure.

A tmuxp session is provided at `.tmuxp/pick-place-gazebo.yaml`:

```bash
cd ~/cr5a_ws/src/cr5a-ces2027
tmuxp load -d .tmuxp/pick-place-gazebo.yaml
```

It opens a Gazebo/MoveIt launch pane, a controller/action monitor, a ROS service monitor and a shell. Keep the existing bridge and frontend windows running; stop the fake-hardware `moveit-demo` and simulated gripper windows first to avoid duplicate controllers/action servers.
