# 01 - Fresh machine setup

Bring a clean Ubuntu host to a state where the `lbr_med_stack` overlay builds and the mock bringup runs.
No robot required for any step here.

## Scope and assumptions

- Ubuntu **24.04 (Noble)**.
- **ROS 2 Jazzy** (the only ROS 2 release this overlay targets).
- **FRI client 1.15**, paired with **Sunrise.OS Med 1.15** on the cabinet.
- You build an **overlay** workspace; you do not modify `/opt/ros/jazzy`.

## 1. Install ROS 2 Jazzy

Follow the official guide: [https://docs.ros.org/en/jazzy/Installation.html](https://docs.ros.org/en/jazzy/Installation.html). The apt path in brief:

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install -y ros-jazzy-desktop
```

> [VERIFY ON FIRST RUN] The apt key URL and source-list snippet track the ROS 2 docs and change
> occasionally. If apt rejects the key or repo, use the exact commands from the installation page above.

Source it (and add to your shell rc so every shell has ROS 2):

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

## 2. Install build tooling

```bash
sudo apt update && sudo apt install -y \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-rosdep

sudo rosdep init 2>/dev/null || true
rosdep update
```

## 3. Install MoveIt 2 and ros2_control

```bash
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-servo \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers
```

`mock_components` (the fake hardware interface the mock bringup uses) ships with `ros-jazzy-ros2-control`.
`moveit_servo` is an upstream runtime dependency of `lbr_bringup`, even though the basic mock and
go-to-pose flow does not launch Servo.

## 4. Set the FRI client version

The FRI client version must match the cabinet OS. Export it for the build and persist it:

```bash
echo 'export FRI_CLIENT_VERSION=1.15' >> ~/.bashrc
export FRI_CLIENT_VERSION=1.15
```

## 5. Clone the overlay workspace and import sources

```bash
git clone https://github.com/rotematari/lbr_med_stack.git
cd lbr_med_stack

# The manifest contains exact revisions for lbr-stack, FRI 1.15, and the matching IDL.
vcs import src < lbr_med_stack.repos
vcs status src
```

The committed `src/lbr_med_examples` package remains in place; `vcs` creates the three ignored
upstream checkouts beside it.

## 6. Resolve dependencies and build

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --event-handlers console_direct+
```

Run the build from the repository root without an activated virtual environment. This keeps ROS
entry-point shebangs on Ubuntu's `/usr/bin/python3` and avoids binding the install tree to a moved
virtual environment.

## 7. Create the Python tooling environment

ROS Jazzy extensions are built for Ubuntu's Python 3.12. Create a system-Python virtual environment
that can see those packages, then install the locked development tools with `uv`:

```bash
uv venv --clear --python /usr/bin/python3.12 --system-site-packages .venv
source .venv/bin/activate
uv sync --active --locked
source /opt/ros/jazzy/setup.bash
python -c "import rclpy, geometry_msgs, moveit.planning"
```

Recreate `.venv` after moving the checkout; virtual-environment scripts contain absolute paths.
ROS packages continue to come from apt and the colcon overlay, while `uv.lock` covers Python-only
development tools.

## 8. Source the overlay and sanity check

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg prefix lbr_bringup
ros2 pkg prefix med7_moveit_config
ros2 pkg prefix lbr_med_examples
```

All three commands should resolve inside this checkout's `install/` directory.

## Next

Mock bringup and the full mock -> real path are in
[`04_hardware_bringup_checklist.md`](04_hardware_bringup_checklist.md) (start at Phase 0).
