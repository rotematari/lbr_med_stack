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
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers
```

`mock_components` (the fake hardware interface the mock bringup uses) ships with `ros-jazzy-ros2-control`.

## 4. Set the FRI client version

The FRI client version must match the cabinet OS. Export it for the build and persist it:

```bash
echo 'export FRI_CLIENT_VERSION=1.15' >> ~/.bashrc
export FRI_CLIENT_VERSION=1.15
```

## 5. Create the overlay workspace and import sources
in your repo directory:
```bash
mkdir -p lbr_med_ws/src && cd lbr_med_ws

# pinned stack first, then the FRI SDK manifest the stack ships
git clone https://github.com/lbr-stack/lbr_fri_ros2_stack.git -b jazzy src/lbr_fri_ros2_stack
vcs import src < src/lbr_fri_ros2_stack/lbr_fri_ros2_stack/repos-fri-${FRI_CLIENT_VERSION}.yaml
rosdep install --from-paths src -i -r -y
```

> [VERIFY ON FIRST RUN] The path `src/lbr_fri_ros2_stack/lbr_fri_ros2_stack/repos-fri-1.15.yaml`
> is where the FRI manifest lives in lbr-stack's documented layout. If the second import fails on a
> missing file, locate it with `find src -name 'repos-fri-1.15.yaml'` and import that path.

## 6. Resolve dependencies and build

```bash
rosdep install --from-paths src -i -r -y
colcon build --symlink-install
```

## 7. Source the overlay and sanity check

```bash
source install/setup.bash
ros2 pkg list | grep lbr
```

You should see `lbr_bringup`, `lbr_description`, and the med7 MoveIt config among the listed packages.

> [VERIFY ON FIRST RUN] The exact med7 MoveIt config package name is provided by the pinned tag and is
> not hard-coded here; the bringup is driven by `lbr_bringup` launch files with `model:=med7`.

## Next

Mock bringup and the full mock -> real path are in
[`04_hardware_bringup_checklist.md`](04_hardware_bringup_checklist.md) (start at Phase 0).