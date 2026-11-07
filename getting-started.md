# Getting Started — CR5A with MoveIt 2 Humble

This guide starts from the MoveIt 2 Humble release Docker image and prepares it to build and run this repository.

- Host: Ubuntu Linux
- Container: Ubuntu 22.04 (Jammy) + ROS 2 Humble
- MoveIt image: moveit/moveit2:humble-release
- Robot: Dobot CR5A
- Shell examples: Bash inside the Docker container

> Run the ROS commands in the Humble container, not in the host's ROS Jazzy environment. Do not source two ROS distributions in the same terminal.

## 1. Pull and start the MoveIt container

Pull the official MoveIt image:

~~~bash
docker pull moveit/moveit2:humble-release
~~~

Clone the repository on the host if you have not already done so. If it is already cloned, skip the `git clone` command and enter your existing checkout:

~~~bash
mkdir -p "$HOME/Projects/econ-systems"
cd "$HOME/Projects/econ-systems"
git clone https://github.com/Mohamed-faaris/cr5a-ces2027.git
cd cr5a-ces2027
~~~

Before starting Docker, make sure your current directory is the repository root (the directory containing `cr5_moveit/`):

~~~bash
pwd
ls
~~~

For RViz GUI access, allow the local container root user to connect to the host X server:

~~~bash
xhost +local:root
~~~

Start the container from the repository root. The `$PWD` bind mount uses the current directory, so this works regardless of the host username or where the repository is cloned.

This version runs the container in the background. That keeps it alive when you close a terminal and lets you open multiple independent shells with `docker exec`:

~~~bash
docker run -d \
  --name cr5a-moveit \
  --network host \
  --ipc host \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD:/root/cr5a_ws/src/cr5a-ces2027" \
  -w /root/cr5a_ws \
  moveit/moveit2:humble-release \
  sleep infinity
~~~

The `sleep infinity` process keeps the container running. It does not start ROS nodes by itself.

Inside the container, the host repository is available at `/root/cr5a_ws/src/cr5a-ces2027`. Source files are edited on the host and immediately visible inside the container. Build output (`build/`, `install/`, and `log/`) is stored in the container workspace; it remains while this container exists, but is removed if you delete the container. The source checkout remains on the host.

### Container lifecycle and terminals

**Open a terminal inside the running container**

Run this from any new host terminal:

~~~bash
docker exec -it cr5a-moveit bash
~~~

Then source ROS 2 and your workspace in that shell:

~~~bash
source /opt/ros/humble/setup.bash
source /root/cr5a_ws/install/setup.bash
~~~

You can repeat `docker exec -it cr5a-moveit bash` in multiple host terminals. Each command opens a separate shell inside the same container. ROS nodes started in one shell are visible to the other shells because they share the container's ROS environment and host network.

For RViz GUI access, run `xhost +local:root` on the host before opening the container shell that will launch RViz. The container already has the host `DISPLAY` value and X11 socket mounted.

**Start the existing stopped container**

~~~bash
docker start cr5a-moveit
~~~

Then open a shell with `docker exec` as shown above.

**Check whether the container is running**

~~~bash
docker ps --filter name=cr5a-moveit
~~~

**Stop the container**

First stop ROS launch processes cleanly with `Ctrl+C` in their terminals and make sure the robot is in a safe state. Then, from the host:

~~~bash
docker stop cr5a-moveit
~~~

This stops the container but keeps it and its build output. Start it again later with `docker start cr5a-moveit`.

**Remove the container**

~~~bash
docker rm cr5a-moveit
~~~

Removing the container deletes files stored only in its writable layer, including this workspace's `build/`, `install/`, and `log/` directories. The repository source remains on the host because it is bind-mounted.

**If you previously created the old interactive container**

The earlier command used `-it ... bash`. To switch that existing container to this background workflow, recreate it once:

~~~bash
docker rm -f cr5a-moveit
~~~

Then run the new `docker run -d ... sleep infinity` command above. This removes the old container and its container-only build output, but not your host repository.

When finished using the GUI, you can revoke the X server permission on the host:

~~~bash
xhost -local:root
~~~

Do not add --privileged for ordinary MoveIt use. Pass through only the specific camera or USB devices needed by a later perception workflow.

## 2. Verify the ROS environment

Run these commands inside the container:

~~~bash
echo "$ROS_DISTRO"
lsb_release -ds
source /opt/ros/humble/setup.bash
ros2 pkg prefix moveit_ros_move_group
~~~

Expected environment:

- ROS_DISTRO should be humble.
- The container OS should be Ubuntu 22.04.
- ros2 pkg prefix should return a path for moveit_ros_move_group.

The image already provides the MoveIt 2 underlay. Do not clone and rebuild the entire MoveIt 2 source tree for this repository.

## 3. Install development tools and CR5A dependencies

Install the workspace build tools, ROS control packages used by the CR5 configuration, and tools for the tmuxp profiles:

~~~bash
apt-get update
apt-get install -y \
  build-essential \
  cmake \
  git \
  curl \
  wget \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-setuptools \
  python3-pip \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  tmux \
  pipx
~~~

Install tmuxp:

~~~bash
pipx install tmuxp
export PATH="$PATH:/root/.local/bin"
~~~

The bridge also uses FastAPI, Uvicorn, WebSockets, YAML, rosapi, and rosbridge_server. Install its ROS and Python runtime packages:

~~~bash
apt-get install -y \
  ros-humble-rosbridge-suite \
  python3-yaml \
  python3-fastapi \
  python3-uvicorn \
  python3-websockets
~~~

Most package dependencies are resolved from the repository's package.xml files in the next step. The explicit packages above cover the container tooling and important runtime components used by the CR5A configuration.

## 4. Initialize rosdep

The MoveIt image may already have rosdep initialized. Initialize it only if the default source list is missing:

~~~bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init
fi

rosdep update
~~~

Source the Humble underlay:

~~~bash
source /opt/ros/humble/setup.bash
~~~

## 5. Verify the mounted repository

The repository was bind-mounted from the host when the container was created. Do not clone it again inside Docker.

~~~bash
cd /root/cr5a_ws
ls src/cr5a-ces2027
~~~

The repository contains the CR5A packages directly; no separate `vcs import` step is required.

Update the checkout before installing dependencies. This includes the ROS dependency-manifest fixes referenced below:

~~~bash
cd /root/cr5a_ws/src/cr5a-ces2027
git pull origin main
cd /root/cr5a_ws
~~~

The relevant package layout is:

~~~text
cr5a-ces2027/
├── cr5_moveit/             # CR5 MoveIt configuration and launch files
├── cra_description/        # Robot description assets
├── dobot_bringup_v4/       # Physical Dobot driver
├── dobot_msgs_v4/          # Dobot ROS messages and services
├── dobot_kinematics_plugin/
├── dobot_moveit/
├── dobot_rviz/
├── dobot_demo/
├── servo_action/
├── dobot_gazebo/           # Optional Gazebo Classic integration
├── cr5a_ros_bridge/        # REST + WebSocket bridge
├── cr5a_web/               # React web console
└── .tmuxp/                 # Hardware and demo terminal layouts
~~~

## 6. Install dependencies declared by the core packages

Run rosdep against the packages needed for the MoveIt demo, physical robot driver, and bridge. This deliberately leaves the Gazebo Classic package out of the core dependency install.

The repository manifests have been corrected for ROS 2 Humble:
- Removed the unused ROS 1 `message_generation` dependency from `dobot_bringup_v4`.
- Removed the optional `warehouse_ros_mongo` dependency from `cr5_moveit`. Normal MoveIt planning-scene collision objects do not require MongoDB warehouse persistence.
- Kept `ament_python` as the `cr5a_ros_bridge` package build type under `<export>`, but removed it as a `<buildtool_depend>`. It is a build-type identifier, not a rosdep key to install.

If you previously cloned the repository before these fixes, run the `git pull` command in section 5 before continuing.

~~~bash
cd /root/cr5a_ws

rosdep install \
  --from-paths \
    src/cr5a-ces2027/cr5_moveit \
    src/cr5a-ces2027/cra_description \
    src/cr5a-ces2027/dobot_bringup_v4 \
    src/cr5a-ces2027/dobot_demo \
    src/cr5a-ces2027/dobot_kinematics_plugin \
    src/cr5a-ces2027/dobot_moveit \
    src/cr5a-ces2027/dobot_msgs_v4 \
    src/cr5a-ces2027/dobot_rviz \
    src/cr5a-ces2027/servo_action \
    src/cr5a-ces2027/cr5a_ros_bridge \
  --ignore-src \
  --rosdistro humble \
  -r -y
~~~

The distro messages printed by `rosdep update` (for example, adding Jazzy/Kilted/Rolling rules or skipping end-of-life distributions) are informational. They do not mean that your Humble environment has changed: the install command explicitly selects `--rosdistro humble`.

If rosdep reports an unresolved key or an APT package that cannot be located, do not guess a replacement package. Confirm that the checkout is up to date with `git pull origin main`, then inspect the package manifest that declares the key.

## 7. Build the CR5A workspace overlay

Build the core CR5A packages and their workspace dependencies:

~~~bash
cd /root/cr5a_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-up-to \
    cr5_moveit \
    dobot_bringup_v4 \
    cr5a_ros_bridge \
    dobot_demo \
    dobot_moveit \
    servo_action \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
~~~

Source the overlay after the build:

~~~bash
source /root/cr5a_ws/install/setup.bash
~~~

For every new terminal, source both layers in this order:

~~~bash
source /opt/ros/humble/setup.bash
source /root/cr5a_ws/install/setup.bash
~~~

Check that the main packages are discoverable:

~~~bash
ros2 pkg prefix cr5_moveit
ros2 pkg prefix dobot_bringup_v4
ros2 pkg prefix cr5a_ros_bridge
~~~

## 8. Run the MoveIt fake-hardware demo

This is the safest first launch because it does not connect to the physical CR5A.

~~~bash
source /opt/ros/humble/setup.bash
source /root/cr5a_ws/install/setup.bash

ros2 launch cr5_moveit demo.launch.py
~~~

The repository's CR5 URDF configuration uses mock_components/GenericSystem for this demo. MoveIt and RViz should start with a simulated joint state. You can test planning without commanding the real robot.

Useful checks from another terminal:

~~~bash
source /opt/ros/humble/setup.bash
source /root/cr5a_ws/install/setup.bash

ros2 node list
ros2 topic list
ros2 action list
~~~

Stop the launch with Ctrl+C.

## 9. Connect to the physical CR5A

Only use this section when you are connected to the actual robot controller network and the robot's physical safety systems are available.

### 9.1 Check the controller IP

The current repository default is in:

~~~text
dobot_bringup_v4/config/param.json
~~~

The checked-in default IP is 192.168.5.1. Confirm that it matches your controller network before launching. The driver reads IP_address from the environment when provided; otherwise it uses the value in param.json.

Example override (replace this address with the actual controller IP):

~~~bash
export IP_address=192.168.5.1
export DOBOT_TYPE=cr5
~~~

### 9.2 Start the physical Dobot driver

In terminal 1:

~~~bash
source /opt/ros/humble/setup.bash
source /root/cr5a_ws/install/setup.bash

ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
~~~

### 9.3 Start MoveIt for the robot

In terminal 2:

~~~bash
source /opt/ros/humble/setup.bash
source /root/cr5a_ws/install/setup.bash

ros2 launch cr5_moveit dobot_moveit.launch.py
~~~

This launch starts the robot state publisher, MoveGroup, and RViz.

### 9.4 Start the CR5A bridge

In terminal 3:

~~~bash
source /opt/ros/humble/setup.bash
source /root/cr5a_ws/install/setup.bash

ros2 launch cr5a_ros_bridge bridge.launch.py start_rosbridge:=false
~~~

The custom bridge is available at:

- REST health: http://127.0.0.1:8000/health
- Application WebSocket: ws://127.0.0.1:8000/ws/v1

The React console uses the custom /ws/v1 WebSocket. The optional standard rosbridge WebSocket is not needed for this frontend workflow.

### Hardware safety notes

- Do not start the fake-hardware demo and physical hardware stack in the same ROS domain at the same time.
- Start the driver and verify its connection and status before testing MoveIt.
- Begin with observation and planning only.
- In cr5a_ros_bridge/config/bridge.yaml, command and execution permissions are disabled by default. Do not enable them until controller behavior, planning-scene correctness, limits, and physical safety have been validated.
- The software bridge is not an emergency stop. Keep the robot's safety-rated physical E-stop available and functional.

## 10. Use the tmuxp profiles

The repository includes two separate terminal layouts:

- .tmuxp/cr5a-demo.yaml — fake-hardware MoveIt demo.
- .tmuxp/cr5a-hardware.yaml — physical driver, MoveIt, bridge, and node monitor.

From /root/cr5a_ws, after building and sourcing the workspace:

~~~bash
tmuxp load src/cr5a-ces2027/.tmuxp/cr5a-demo.yaml
~~~

For the real robot, stop the demo session first and load the hardware profile:

~~~bash
tmux kill-session -t cr5a-demo
tmuxp load src/cr5a-ces2027/.tmuxp/cr5a-hardware.yaml
~~~

To switch back, stop the hardware stack and load the demo profile:

~~~bash
tmux kill-session -t cr5a-hardware
tmuxp load src/cr5a-ces2027/.tmuxp/cr5a-demo.yaml
~~~

Ctrl+B then D detaches from tmux without stopping its processes. Reattach using tmux attach -t cr5a-demo or tmux attach -t cr5a-hardware.

Always stop the active stack safely before switching profiles. Killing a tmux session is not a safety stop for a robot that is already moving.

## 11. Optional: build the Gazebo Classic package

The repository's dobot_gazebo package depends on Gazebo Classic and gazebo_ros2_control. These are not required for the MoveIt fake-hardware demo or the physical CR5A workflow above.

Only install these dependencies if you specifically need the legacy Gazebo Classic launch files:

~~~bash
cd /root/cr5a_ws

rosdep install \
  --from-paths src/cr5a-ces2027/dobot_gazebo \
  --ignore-src \
  --rosdistro humble \
  -r -y

colcon build \
  --symlink-install \
  --packages-up-to dobot_gazebo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
~~~

Gazebo Classic reached end of life in January 2025. Keep this optional environment separate from the basic MoveIt setup.

## 12. Optional: run the React web console

The web console is separate from the ROS build. It requires Node.js and pnpm.

~~~bash
cd /root/cr5a_ws/src/cr5a-ces2027/cr5a_web

pnpm install
pnpm dev
~~~

Open http://127.0.0.1:5173 in a browser. Start the physical driver, MoveIt, and bridge first. The frontend connects to the bridge's /ws/v1 endpoint.

## Troubleshooting checks

| Symptom | Check |
|---|---|
| ROS_DISTRO is empty or wrong | Run source /opt/ros/humble/setup.bash in that terminal. |
| Package cr5_moveit not found | Source /root/cr5a_ws/install/setup.bash after a successful build. |
| rosdep reports `message_generation`, `ament_python`, or `warehouse_ros_mongo` as unresolved | Pull the latest `main` branch in section 5, then rerun the dependency command. These obsolete/unneeded dependency declarations were removed from the manifests. |
| rosdep prints distro messages about Iron/Jazzy/Kilted/Rolling | These are rosdep rule-cache status messages. The install command still targets Humble through `--rosdistro humble`. |
| rosdep cannot resolve another key | Check the package manifest that declares it; do not install a guessed package name. Run `rosdep update` and retry after verifying the source checkout. |
| RViz cannot open a display | Check DISPLAY, the X11 socket mount, and the host xhost permission. |
| Driver cannot connect | Confirm controller IP, host network connectivity, Ethernet route, and IP_address. |
| MoveIt has no current joint state | Check that the correct driver or fake-hardware demo is running and publishing joint states. |
| Bridge cannot find MoveIt actions | Start MoveIt first and check ros2 action list. |

## References

- [MoveIt 2 Docker image and tags](https://hub.docker.com/r/moveit/moveit2/tags)
- [MoveIt 2 Humble source and dependency installation](https://moveit.ai/install-moveit2/source/)
- [MoveIt 2 Humble Docker guide](https://moveit.picknik.ai/humble/doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu.html)
- [ROS 2 Humble workspace tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- [ros2_control Humble installation](https://control.ros.org/humble/doc/getting_started/getting_started.html)
- [Gazebo Classic ros2_control documentation](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html)
