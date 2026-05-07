# Hiwonder Robot Plays Fetch

**Joseph Donaldson, Brenna O'Donnell, Chiang Hui Zhi**

A 5DOF Hiwonder robotic arm that autonomously scans for, detects, picks up, and delivers a marked block. The robot uses ArUco marker detection, analytic inverse kinematics, cubic trajectory generation, and a two-part laptop/Pi server architecture to complete the full fetch sequence.

---

## Demo

> [Watch the demo video here](#) *www.insert_link.com*

---

## Technical Report

[Read the full technical report here](#) *(replace with link to uploaded PDF)*

---

## How It Works

The system is split into two parts. A path planner brain runs on the laptop and handles all computation, including frame transforms, IK, and trajectory generation. A thin server runs on the Raspberry Pi and just executes joint commands on the hardware servos and triggers the camera when asked.

The fetch sequence:
1. Robot homes and opens gripper
2. Scans through preset joint positions, triggering ArUco detection at each one
3. On detection, the brain transforms the camera-frame position into robot base frame coordinates
4. Brain runs IK and generates a cubic trajectory to the block
5. Joint stream is sent to Pi, robot moves to block and closes gripper
6. Robot moves to dropoff, opens gripper, and returns home

---

## Setup and Running

### On the Raspberry Pi

**Navigate to the project directory:**
```bash
cd ~/funrobo_kinematics_final
```

**Activate the Conda environment:**
```bash
conda activate funrobo_hw
```

**Install the package:**
```bash
pip install -e .
```

**Add the project to your Python path:**
```bash
export PYTHONPATH=$PYTHONPATH:/home/pi
```

**Make it permanent across sessions:**
```bash
echo 'export PYTHONPATH=$PYTHONPATH:/home/pi' >> ~/.bashrc
source ~/.bashrc
```

**Start the Pi server:**
```bash
python /home/pi/funrobo_kinematics_final/project/path_planner_server.py
```

The Pi server will now listen for connections from the laptop. It accepts joint commands, gripper commands, and detection requests over TCP.

### On the Laptop

Run the GUI and path planner brain. Once the Pi server is running and the IP is configured, connect through the GUI, then hit **Run MVP** to execute the fetch sequence.

For simulation without the physical robot, run `pp_local_server.py` locally, which spins up a 3D matplotlib visualizer to preview trajectories before running them on hardware.

---

## Repo Structure

```
project/
├── mvp.py                  # Top-level fetch sequence (version 1)
├── mvp2.py                 # Extended fetch sequence with celebrate/dance behaviors
├── path_planner_brain.py   # Laptop-side: IK, trajectory gen, frame transforms
├── path_planner_gui.py     # Tkinter GUI for controlling the robot
├── path_planner_server.py  # Pi-side: hardware interface server
├── pp_local_server.py      # Local sim visualizer server
└── object_detection.py     # ArUco CV pipeline
```

---

## Dependencies

- OpenCV (with ArUco support)
- NumPy
- Matplotlib
- `funrobo_kinematics` (included, install with `pip install -e .`)