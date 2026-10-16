# InvestiGator
MIL's UAV Drone and software for the RobotX competition.

<img width="1092" height="547" alt="image" src="https://github.com/user-attachments/assets/792a302b-4b28-4f99-a625-012084a189be" />


## Major Features
InvestiGator is flying!
- Implementation of reception and transmission of major MAVLink messages using a multi-threaded approach for subscribing to and publishing messages with python decorators
- TextualUI based terminal GUI for commanding the UAV from a ground control station
- Connectivity of companion computer script and ground control script to both hardware flight controller and ArduPilot simulation
- Command retries and timeouts for messages that expect an acknowledgement to be received
- Mission development uses python decorators to automatically register functions in GUI drowpdown list and companion selection list
- Support for python multiprocess camera pipeline
- Support for Raspberry Pi GPIO in rpigpio branch
- Simple API for commanding the UAV in missions
- Example missions implemented to show how to write missions


# Getting Started
## Prerequisites
- UV: https://docs.astral.sh/uv/getting-started/installation/

## Simulation
- Gazebo and ArduPilot Plugin: https://ardupilot.org/dev/docs/sitl-with-gazebo.html
- ArduPilot Simulation: https://ardupilot.org/dev/docs/SITL-setup-landingpage.html#sitl-setup-landingpage
  
Recommended:
- WSL2: https://learn.microsoft.com/en-us/windows/wsl/install

## Configuration
- A `config.toml` is required in the base folder of the repository to define the addresses for the companion computer and the ground control
- Make a copy of `example.config.toml` and enter in the required addresses as prompted
- You can find the COM port of attached devices in Windows by checking the `Device Manager`
- You can find the 

## Running scripts
- Use `uv sync` to download the required libraries and create the virtual environemnt
- Use `uv run path/to/script.py` to run a script
- Example: `uv run src/main.py --sim` will run the companion computer script using the simulation configuration as defined in config.toml
- Example: `uv run src/ground_control.py` will run the ground control script using the hardware configuration in config.toml

# Definitions
- UAV: Uncrewed Arial Vehicle
- Companion or Onboard Computer: Raspberry Pi on the UAV that handles communication with the Ground Control script and other robots. Commands the flight controller directly
- Flight Controller/CubeOrange+: Runs the ArduPilot firmware and handles control of the UAV's motors and sensors
- Ground Control: Main computer that observes MAVLink network traffic and optionally sends commands to directly control the UAV. Acts as a stand-in for other robots and as a way of unit testing the UAV's behavior


# Architecture
## Hardware
<img width="1160" height="813" alt="image" src="https://github.com/user-attachments/assets/b32702a3-6670-4752-9a14-14ee3d769b7a" />

- InvestiGator UAV consists of two computers, a Raspberry Pi 5 and an Orange Cube+ flight controller. The flight controller receives commands from the Pi which communicates with a ground control station or other robot. 

- The flight controller is connected to a ground station with a radio modem connected to a third computer, the ground station. The ground station sends and receives messages from the UAV to start missions, update status, and return points of interest as the UAV completes its tasks.

- Battery: 10000 mAHr 6S batteries
- GPS: Here4 GPS module
- Radio: RFD900x Modem
- Main camera: DepthAI OAK-D Wide

## Software
### General
<img width="953" height="542" alt="image" src="https://github.com/user-attachments/assets/5f556260-944d-4a72-a58e-29d3fc213656" />

### Simulation Communication
<img width="567" height="209" alt="image" src="https://github.com/user-attachments/assets/b8434fe1-e287-4d92-bf68-6440b9dd1aeb" />

### Hardware Communication
<img width="560" height="196" alt="image" src="https://github.com/user-attachments/assets/537d9b2f-6a8e-4c56-865d-4d116f43d79b" />

# References and Libraries Used
- [MAVLink](https://mavlink.io/en/)
- [ArduCopter Wiki](https://ardupilot.org/copter/)
- [RFD900x Documentation](https://files.rfdesign.com.au/docs/)
- [RFD900x Tools](https://files.rfdesign.com.au/tools/)
- [Textual Library](https://textual.textualize.io/)
- [Mission Planner](https://ardupilot.org/planner/)
