# Leap Motion + Unitree Z1 ROS Container

## Overview
This container image provisions a ROS Noetic workspace with the Unitree `z1_ros` stack and the Leap Motion C API Python bindings from `leapc-python-bindings`. It is intended for running mixed ROS/Leap Motion development so you can stream arm state from the Z1 while processing pointing gestures provided by `src/pointing.py`.

## Prerequisites
- Docker Engine 20.10+ on an x86_64 host (Linux or WSL2).
- Unitree Z1 arm with official firmware and the accompanying ROS licenses.
- Leap Motion/Ultraleap controller with the Linux Leap SDK (2.3+), extracted on the host.
- USB access to the Leap Motion sensor and network reachability (wired or Wi-Fi) to the Z1 control box.

## Build the Image
From the repository root (where `Dockerfile` lives):

```bash
docker build -t leap-z1-dev .
```

If using WSL and docker refuses the first step, use:
```bash
sudo bash -c 'echo "nameserver 8.8.8.8" > /etc/resolv.conf'
``` 
And then restart wsl to fix DNS connectivity. If docker is running on another port, change 8.8.8.8

The image build clones `unitreerobotics/z1_ros`, installs MoveIt support, ros_control controllers, and sets up the editable Leap bindings.

## Prepare the Leap SDK Assets
1. Download the Linux Leap Motion SDK from Ultraleap.
2. Extract it on the host, e.g. `~/Downloads/LeapSDK`.
3. Confirm the folder contains `libLeap.so` under `lib/x64/` and headers inside `include/`.

You will bind-mount this directory into the container at `/opt/leap-sdk` so the bindings and Leap runtime can discover it.

## Run the Container
Use a privileged run so the container can talk to USB and ROS devices:

```bash
docker run -it --rm \
  --name leap-z1 \
  --net host \
  --privileged \
  -v ~/Downloads/LeapSDK:/opt/leap-sdk:ro \
  -v /dev/bus/usb:/dev/bus/usb \
  leap-z1-dev
```

- `--net host` allows ROS topics to flow directly to and from the Z1 control box.
- Mounting `/dev/bus/usb` exposes the Leap Motion controller (adjust udev permissions on the host as required).
- The `ros_entrypoint.sh` script automatically sources `/opt/ros/noetic/setup.bash` and `/root/z1_ws/devel/setup.bash`, and exports `LEAP_SDK_ROOT` / library paths.

You can optionally mount your development workspace (e.g. `-v $(pwd):/workspace`) if you need to edit code from the host.

## Workspace Layout
- Catkin workspace: `/root/z1_ws` (standard `src`, `build`, `devel` structure).
- Unitree packages: `/root/z1_ws/src/z1_ros` (contains `z1_bringup`, `z1_controller`, `z1_moveit_config`, `z1_examples`, etc.).
- Leap bindings: `/opt/leap/leapc-python-bindings/` (editable installs for `leapc-python-api` and `leapc-cffi`).
- Gesture prototype: `/opt/leap/leapc-python-bindings/src/pointing.py`.

## Typical Development Flow
1. Start the container (see previous section).
2. In the container shell, verify the Leap SDK and bindings can talk to the sensor:
   ```bash
   python3 /opt/leap/leapc-python-bindings/src/pointing.py
   ```
   A window labelled "Leap Motion Direction Detector" should mirror your pointing direction once the sensor is detected.
3. Bring up the Unitree Z1 ROS stack:
   ```bash
   roslaunch z1_bringup bringup.launch robot_ip:=<Z1_IP>
   ```
   Replace `<Z1_IP>` with the control box address; ensure the host interface is on the same subnet.
4. Launch MoveIt or other control interfaces as needed, for example:
   ```bash
   roslaunch z1_moveit_config demo.launch
   ```
5. Develop integration nodes (e.g. convert Leap pointing vectors into Z1 trajectories) under a new package inside `/root/z1_ws/src`. After adding packages run `catkin build` and reopen shells or `source /root/z1_ws/devel/setup.bash`.

## Running Custom Nodes
- Create packages with `catkin create pkg <pkg_name> --catkin-deps rospy std_msgs geometry_msgs` inside `/root/z1_ws/src`.
- Use the Leap API (`import leap`) from Python nodes. The environment already exposes the SDK libraries via `LD_LIBRARY_PATH`.
- Publish Leap-derived targets as `geometry_msgs/PoseStamped` or custom `z1_controller` commands based on the requirements.
- Remember to rebuild (`catkin build <pkg_name>`) and source the workspace after code changes.

## Troubleshooting
- **Leap device not detected**: ensure the USB bus is mounted and the host user grants read permissions (e.g. `sudo usermod -aG plugdev $USER` and re-login).
- **rosdep failures during build**: run `docker build` again with internet access; the Dockerfile initializes `rosdep` but requires online package indexes.
- **Missing Unitree packages**: confirm your account has access to the private parts of `z1_ros` where applicable and update `Z1_ROS_VERSION` build ARGs if you need a specific branch.
- **Network latency**: prefer wired Ethernet and disable firewalls blocking ROS TCP/UDP ports when controlling the physical arm.

## Next Steps
- Prototype Leap-to-trajectory translators by reading the pointing direction from `pointing.py` logic and publishing to `/z1_controller/command`.
- Integrate additional sensors or RViz visualization by installing more ROS packages (use `apt-get` inside the running container or extend the Dockerfile).
- Snapshot the container state with `docker commit` or use Docker volumes for persistent catkin workspaces if required.
