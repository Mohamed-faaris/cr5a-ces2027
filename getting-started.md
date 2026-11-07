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
  python3-venv \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  tmux
~~~

Install tmuxp in an isolated Python virtual environment. This works with the older pip version shipped in the Humble container and avoids installing Python packages into Ubuntu's system Python:
~~~bash
python3 -m venv /opt/tmuxp-venv
/opt/tmuxp-venv/bin/pip install tmuxp
ln -sf /opt/tmuxp-venv/bin/tmuxp /usr/local/bin/tmuxp
~~~

Verify that the installed binary is on PATH:

~~~bash
ln -sf /opt/tmuxp-venv/bin/tmuxp /usr/local/bin/tmuxp
hash -r
command -v tmuxp
tmuxp --version
~~~

The symlink makes `tmuxp` available in every new container terminal without changing PATH. If `tmuxp` still reports `command not found`, confirm that `/usr/local/bin` is on PATH and that the symlink points to an existing file:

~~~bash
echo "$PATH"
ls -l /usr/local/bin/tmuxp
/opt/tmuxp-venv/bin/tmuxp --version~~~


### Install Node.js and pnpm (for the React web console)

The MoveIt Docker image does not include Node.js or pnpm. Install Node.js 22 LTS and pnpm inside the container:

~~~bash
apt-get install -y ca-certificates
curl -fsSL https://deb.nodesource.com/setup_22.x | bash -
apt-get install -y nodejs
npm install --global pnpm
~~~

Verify the installations:

~~~bash
node --version
npm --version
pnpm --version
~~~

These tools are installed inside the persistent container, so they remain available in every new `docker exec` terminal. If you remove and recreate the container, repeat this installation section.

The bridge also uses FastAPI, Uvicorn, WebSockets, YAML, rosapi, and rosbridge_server. Install its ROS and Python runtime packages:

~~~bash
apt-get install -y \
  ros-humble-rosbridge-suite \
  python3-yaml \
  python3-fastapi \
  python3-uvicorn \
  python3-wsproto \
  python3-websockets
~~~

Most package dependencies are also declared in the repository's package.xml files and are resolved in section 6. The bridge installs both `python3-wsproto` and `python3-websockets`: the custom FastAPI/Uvicorn API explicitly selects Uvicorn's `wsproto` backend, avoiding older `websockets` releases that pass the removed `loop=` argument to `asyncio.Lock()` on Python 3.10. Do not patch `asyncio` or files under `/usr/lib/python3/dist-packages`.

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

### Automatically configure every new terminal

Configure the container user's `/root/.bashrc` once. This gives every new interactive Bash terminal:

- A readable colored root prompt: red username, magenta hostname, cyan working directory.- ROS 2 Humble and the workspace overlay automatically sourced.
- CR5A controller defaults.
- Useful navigation, listing, grep, ROS and colcon shortcuts.
- A larger, cleaner command history.

The controller defaults in `dobot_bringup_v4/config/param.json` are IP `192.168.5.1` and robot type `cr5`. These exports make the values explicit in every new shell.

Run this once inside the container. The marker check prevents appending the block more than once:

~~~bash
if ! grep -q '# >>> CR5A shell configuration >>>' /root/.bashrc; then
  cat >> /root/.bashrc <<'EOF'

# >>> CR5A shell configuration >>>

# CR5A controller defaults
export IP_address="192.168.5.1"
export DOBOT_TYPE="cr5"

# ROS 2 Humble underlay and this workspace overlay
source /opt/ros/humble/setup.bash
if [ -f /root/cr5a_ws/install/setup.bash ]; then
  source /root/cr5a_ws/install/setup.bash
fi

# Colored prompt: red user, magenta host, cyan current directory
PS1='\[\e[1;31m\]\u\[\e[0m\]@\[\e[1;35m\]\h\[\e[0m\]:\[\e[1;36m\]\w\[\e[0m\]\$ '

# History
HISTSIZE=10000
HISTFILESIZE=20000
HISTCONTROL=ignoreboth:erasedups
shopt -s histappend checkwinsize
PROMPT_COMMAND='history -a'

# Basic shell conveniences
alias ll='ls -alF --color=auto'
alias la='ls -A --color=auto'
alias l='ls -CF --color=auto'
alias grep='grep --color=auto'
alias cls='clear'
# CR5A workspace navigation
alias cws='cd /root/cr5a_ws'
alias csrc='cd /root/cr5a_ws/src/cr5a-ces2027'

# ROS 2 shortcuts
alias rnodes='ros2 node list'
alias rtopics='ros2 topic list'
alias rservices='ros2 service list'
alias ractions='ros2 action list'

# Build shortcuts (run from /root/cr5a_ws)
alias cb='colcon build --symlink-install'
alias cbrel='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

# <<< CR5A shell configuration <<<
EOF
fi
~~~

Apply it to the current terminal immediately:

~~~bash
source /root/.bashrc
~~~

Check the result:

~~~bash
echo "$ROS_DISTRO"
echo "$IP_address"
echo "$DOBOT_TYPE"
printf '%s\\n' "$PS1"
~~~

Your prompt should look similar to:

~~~text
root@ECON003855L:~/cr5a_ws#
~~~

The username is red, hostname magenta, and working directory cyan. The prompt colors are only visual; they do not change permissions or robot state.

### New-terminal and build workflow

After the initial configuration, open a new terminal inside the container. ROS 2 Humble, the workspace overlay (if it has been built), and CR5A defaults load automatically.

For an existing package, build from the workspace root:

~~~bash
cws
cb --packages-up-to cr5a_ros_bridge
~~~

Or use the Release build shortcut:

~~~bash
cbrel --packages-up-to cr5a_ros_bridge
~~~

For a newly added package, install its dependencies first:

~~~bash
cws
rosdep install --from-paths src/<path-to-package> --ignore-src --rosdistro humble -r -y
~~~

Then build it:

~~~bash
cb --packages-up-to <package_name>
~~~

Replace `<package_name>` with the ROS package name from its `package.xml`, not necessarily its folder name. New packages must be inside the workspace `src/` directory and contain a valid `package.xml`.

After a successful build, the current shell does not automatically re-source the overlay. Run:

~~~bash
source /root/cr5a_ws/install/setup.bash
~~~

New terminals will load the updated overlay automatically.

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

ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py~~~

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

### Rebuild the bridge after pulling changes (inside the running container)

These commands are run **inside the existing Humble container**. Do not run `docker` or `docker compose` commands from this shell.

~~~bash
cd /root/cr5a_ws/src/cr5a-ces2027
git pull origin main

apt-get update
apt-get install -y python3-wsproto

cd /root/cr5a_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cr5a_ros_bridge
source install/setup.bash
~~~

Stop the old bridge process with `Ctrl+C`, then start it again:

~~~bash
ros2 launch cr5a_ros_bridge bridge.launch.py start_rosbridge:=false
~~~

From another shell inside the container, check the API and selected WebSocket backend dependency:

~~~bash
curl --fail --silent http://127.0.0.1:8000/health
python3 -c "import wsproto, uvicorn; print('wsproto', wsproto.__version__, '| uvicorn', uvicorn.__version__)"
~~~
The bridge's `main.py` pins Uvicorn to `ws="wsproto"`. Rebuilding is required after pulling that source change; merely installing `python3-wsproto` does not change an already-running Uvicorn process.

### Hardware safety notes

- Do not start the fake-hardware demo and physical hardware stack in the same ROS domain at the same time.
- Start the driver and verify its connection and status before testing MoveIt.
- Begin with observation and planning only.
- In cr5a_ros_bridge/config/bridge.yaml, command and execution permissions are disabled by default. Do not enable them until controller behavior, planning-scene correctness, limits, and physical safety have been validated.
- The software bridge is not an emergency stop. Keep the robot's safety-rated physical E-stop available and functional.

## 10. Run the complete demo stack with tmuxp

The demo profile opens each part of the development stack in its own tmux window:

| Window | What it starts |
|---|---|
| `moveit-demo` | MoveIt fake-hardware demo and RViz |
| `gazebo` | Gazebo Classic CR5 simulation |
| `bridge` | CR5A REST/WebSocket bridge and rosbridge |
| `frontend` | React/Vite web console |
| `monitor` | ROS node and topic monitors |
| `shell` | Interactive ROS 2 terminal |

### One-time setup

Install tmux and tmuxp if you have not already completed section 3:

~~~bash
apt-get install -y tmux python3-venv
python3 -m venv /opt/tmuxp-venv
/opt/tmuxp-venv/bin/pip install tmuxp
ln -sf /opt/tmuxp-venv/bin/tmuxp /usr/local/bin/tmuxp
tmuxp --version
~~~

Enable mouse support so you can click window names in the tmux status bar, click panes to focus them, and scroll terminal output:

~~~bash
grep -qxF 'set -g mouse on' ~/.tmux.conf 2>/dev/null ||
  echo 'set -g mouse on' >> ~/.tmux.conf

tmux source-file ~/.tmux.conf
~~~

Install frontend dependencies once:

~~~bash
cd /root/cr5a_ws/src/cr5a-ces2027/cr5a_web
pnpm install
~~~

The demo profile also starts Gazebo Classic. Install and build its optional dependencies as described in section 11 before using the complete profile.

### Start the full demo stack

Run from the workspace root inside the Humble container:

~~~bash
cd /root/cr5a_ws
tmuxp load src/cr5a-ces2027/.tmuxp/cr5a-demo.yaml
~~~

The tmux session is named `cr5a-demo`. Click the window names in the bottom status bar to switch between MoveIt, Gazebo, bridge, frontend, monitor, and shell.

### Automatic session logs

Both tmuxp profiles automatically capture each pane's terminal output. Every new tmux session gets its own timestamped directory under:

~~~text
/root/cr5a_ws/logs/tmux/YYYYMMDD-HHMMSS/
~~~

The demo profile creates separate files for MoveIt, Gazebo, bridge, frontend, both monitor panes, and the interactive shell. The hardware profile creates separate files for the driver, MoveIt, bridge, and node monitor.

List saved sessions:

~~~bash
ls -lt /root/cr5a_ws/logs/tmux
~~~

Inspect the newest session and follow a log:

~~~bash
LOG_DIR="$(find /root/cr5a_ws/logs/tmux -mindepth 1 -maxdepth 1 -type d | sort | tail -n 1)"
ls -lh "$LOG_DIR"
tail -f "$LOG_DIR/moveit-demo.log"
~~~

Replace the filename with the pane you want to inspect, such as `moveit-demo.log` or `gazebo.log`. Logs remain after the tmux session exits. They are stored in the container workspace, so copy them to the host or a persistent volume if you need them after removing the container.

The profiles use `.tmuxp/log-pane.sh` to attach tmux's pane-output pipe when each pane starts. If logging does not start, update the checkout with `git pull origin main` and confirm that this helper script exists.

### If a window exits immediately

tmuxp can create the session successfully while one pane command fails. Inspect the newest pane logs first:

~~~bash
LOG_DIR="$(find /root/cr5a_ws/logs/tmux -mindepth 1 -maxdepth 1 -type d | sort | tail -n 1)"
ls -lh "$LOG_DIR"
tail -n 100 "$LOG_DIR/frontend.log"
~~~

Known first-run failures and fixes:

- `frontend.log` contains `cd: cr5a_web: No such file or directory`: the profile is out of date. Pull the latest `main` branch and confirm the frontend pane runs `cd src/cr5a-ces2027/cr5a_web`. Then install the web dependencies once:

  ~~~bash
  cd /root/cr5a_ws/src/cr5a-ces2027/cr5a_web
  pnpm install
  ~~~

- `gazebo.log` contains `executable 'joint_state_relay.py' not found`: the installed relay is a symlink to a source file without execute permission. Pull the latest `main` branch, where the source file is committed as executable, or repair the mode and restart:

  ~~~bash
  chmod +x /root/cr5a_ws/src/cr5a-ces2027/dobot_rviz/dobot_rviz/joint_state_relay.py
  test -x /root/cr5a_ws/install/dobot_rviz/lib/dobot_rviz/joint_state_relay.py
  ~~~

- `bridge.log` contains `AttributeError: 'dict' object has no attribute 'append'`: the bridge build is stale. Pull the latest `main` branch, rebuild the bridge, and source the overlay:

  ~~~bash
  cd /root/cr5a_ws
  colcon build --symlink-install --packages-select cr5a_ros_bridge
  source install/setup.bash
  ~~~

- `bridge.log` contains `Fields of type "<class 'types.UnionType'>" are not supported`: the bridge build is stale. Pull the latest `main` branch and rebuild it with the same command. Humble ships pydantic 1.x, so bridge API annotations must use `typing.Optional` instead of `X | None`.

If the `cr5a-demo` session is left in a broken state, remove it before loading the profile again:

~~~bash
tmux kill-session -t cr5a-demo
~~~

Useful tmux controls:

| Action | Control |
|---|---|
| Switch windows | Click a window name in the status bar |
| Focus a pane | Click inside the pane |
| Next / previous window | `Ctrl+B`, then `n` / `p` |
| Choose a window by number | `Ctrl+B`, then its number |
| Detach without stopping processes | `Ctrl+B`, then `d` |
| Reattach | `tmux attach -t cr5a-demo` |
| List sessions | `tmux ls` |

The frontend is served at http://127.0.0.1:5173. The bridge API is at http://127.0.0.1:8000 and its application WebSocket is at ws://127.0.0.1:8000/ws/v1.

**Important simulation distinction:** `cr5_moveit demo.launch.py` starts MoveIt with fake hardware. The `dobot_gazebo` launch starts Gazebo separately. This profile opens both, but does not connect MoveIt's fake controller to Gazebo's simulated robot. Do not interpret the two windows as a single MoveIt-controlled Gazebo simulation.
To stop the stack, use `Ctrl+C` in each running launch window, then exit the tmux session. Do not use this demo profile alongside the physical hardware profile.

### Physical hardware profile

The separate hardware profile starts the physical Dobot driver, MoveIt, bridge, and a node monitor. It does not start Gazebo or the fake-hardware demo.

~~~bash
cd /root/cr5a_ws
tmuxp load src/cr5a-ces2027/.tmuxp/cr5a-hardware.yaml
~~~

The hardware profile uses the same automatic timestamped logging. Its pane logs are saved as `hardware-driver.log`, `moveit.log`, `bridge.log`, and `monitor-nodes.log` inside that session's log directory.

Stop the active demo stack before starting the hardware profile. Verify the controller network and physical safety systems before launching the real robot.

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

The web console is separate from the ROS build. Complete the Node.js and pnpm installation in section 3 before running it.

~~~bash
cd /root/cr5a_ws/src/cr5a-ces2027/cr5a_web

pnpm install
pnpm dev
~~~
Open http://127.0.0.1:5173 in a browser. Start the physical driver, MoveIt, and bridge first. The frontend connects to the bridge's /ws/v1 endpoint.

## Troubleshooting checks

| Symptom | Check |
|---|---|
| `tmuxp: command not found` | Recreate the symlink with `ln -sf /opt/tmuxp-venv/bin/tmuxp /usr/local/bin/tmuxp`, run `hash -r`, then verify with `tmuxp --version`. |
| No tmux pane log files appear | Pull the latest repository version and confirm `.tmuxp/log-pane.sh` exists. Check that the session was started with the updated tmuxp profile. |
| Frontend pane exits with `cd: cr5a_web: No such file or directory` | Pull the latest `main` branch so the profile uses `src/cr5a-ces2027/cr5a_web`, then run `pnpm install` there. |
| Gazebo pane exits with `executable 'joint_state_relay.py' not found` | Pull the latest `main` branch, or run `chmod +x src/cr5a-ces2027/dobot_rviz/dobot_rviz/joint_state_relay.py` and confirm `test -x install/dobot_rviz/lib/dobot_rviz/joint_state_relay.py`. |
| Bridge pane exits with `'dict' object has no attribute 'append'` | Pull the latest `main` branch, rebuild with `colcon build --symlink-install --packages-select cr5a_ros_bridge`, and source `install/setup.bash`. The old build shadowed rclpy's internal publisher list. |
| Bridge pane exits with `Fields of type "<class 'types.UnionType'>" are not supported` | Pull the latest `main` branch and rebuild `cr5a_ros_bridge`. Humble ships pydantic 1.x, which does not support `X \| None` annotations. |
| Bridge WebSocket crashes with `TypeError: As of 3.10, the *loop* parameter was removed from Lock()` | Pull the latest `main`, install `python3-wsproto`, rebuild `cr5a_ros_bridge`, and restart it. The current bridge pins Uvicorn to `wsproto`; follow the inside-container commands in section 9.4. Do not modify Python `asyncio` or system `websockets` files. |
| `node`, `npm`, or `pnpm` command not found | Complete the Node.js and pnpm installation in section 3. |
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