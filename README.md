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

- UV: <https://docs.astral.sh/uv/getting-started/installation/>

## Simulation

- Gazebo and ArduPilot Plugin: <https://ardupilot.org/dev/docs/sitl-with-gazebo.html>
- ArduPilot Simulation: <https://ardupilot.org/dev/docs/SITL-setup-landingpage.html#sitl-setup-landingpage>
  
Recommended:

- WSL2: <https://learn.microsoft.com/en-us/windows/wsl/install>

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

<img width="570" height="209" alt="image" src="https://github.com/user-attachments/assets/b8434fe1-e287-4d92-bf68-6440b9dd1aeb" />

### Hardware Communication

<img width="560" height="196" alt="image" src="https://github.com/user-attachments/assets/537d9b2f-6a8e-4c56-865d-4d116f43d79b" />

### Code API: Drone Control

The main file for the project is /src/InvestiGator/vehicle.py. This defines the class VehicleManager which provides the API for controlling the drone through the MAVLink protocol. It takes in a MAVConnection class to handle the MAVLink protocol and instantiates Location and Status classes to store vehicle state.

#### MAVConnection

MAVConnection handles packing and unpacking MAVLink messages from the connected radio. MAVLink is a publish-subscribe protocol, meaning each message has a type, source, and destination. Functions can subscribe to these message types from a specified source, and can publish messages to a destination. MAVLink message definitions can be found [at the MAVLink website](https://mavlink.io/en/messages/common.html).

MAVConnection takes an address, either an IP for simulation or serial port for the radio, and system settings. The system settings can be left to the defaults if only controlling the drone and should be set to a unique source component/system if multiple robots are on the MAVLink network. The relevant parameters are:

- `mav_type`: Either mavlink.MAV_TYPE_GCS for the ground control or mavlink.MAV_TYPE_ONBOARD_CONTROLLER for the companion computer on the drone.

- `source_statem`: A unique value for each endpoint on the network. 1 for the drone, 254 for the ground control station. Other robots on the network should have their own unique value as well, configurable through MissionPlanner ArduPilot parameters.

- `source_component`: Should be mavlink.MAV_COMP_ID_ONBOARD_COMPUTER for companion computers on the network, otherwise can be left default for ground control stations.

- `baud`: Safe to keep as default, though you can match it to whatever baud rate is being used with the radios. Baud is ignored for IP connections.

MAVConnection starts two main threads and instantiates two classes, PublicationManager and SubscriptionManager, which start their own threads, for four threads in total. Two threads are responsible for sending messages and two are responsible for reading messages. Message objects are passed between these threads via thread safe `queues`. The message objects are classes with fields corresponding to the message definition found at the link above. The `Pymavlink` library provides functions for sending messages of different types where the function parameters fill in the fields for the function type. Here is an example for the [`command_long message` type](https://mavlink.io/en/messages/common.html#COMMAND_LONG):

```Python
self.mav.command_long_send(
                target_system=target_system,
                target_component=target_component,
                command=command,
                confirmation=attempt,
                param1=param1,
                param2=param2,
                param3=param3,
                param4=param4,
                param5=param5,
                param6=param6,
                param7=param7
            )
```

The definition for the fields are given on the MAVLink website. The communication goes through the threads and queues like this:

- `mav_sender` thread: When a message is sent via `MAVConnection.mav.x_send` it is pushed to the `send_queue`. This thread's job is to redirect messages sent with Pymavlink's internal .send commands to `send_queue` so the queue can be used to collect outgoing messages from different threads.

- `publication` thread: Functions can register to be published at a given frequency. The `PublicationManager` class keeps a dictionary of subscribed functions and their requested frequency. It cycles through the list and constantly checks if enough time has elapsed to call the functions. The functions define a message type and parameters to send. Information about how to register functions is described below.

- `mav_reader` thread: This thread's job is to receive incoming messages from the MAVLink connection, either IP or serial port. Pymavlink handles deserializing the message and checking for errors, and returns a packed class object of the message type. The message parameters can then be accessed via normal `.` accessors. When a message is received, it is placed on the `receive_queue` for the subscription manager to route to subscribed messages.

- `subscription` thread: Functions can be registered as callbacks for when a message of a specific type and from a specific source are received. When registered, the function is placed in a dictionary with the desired message type. When a new message is received, the thread checks if there are any entries in it's function dictionary that respond to the message's type. If so, the function is called with the message passed as an argument. The subscribed message can then use the message however it likes.

##### Registering a subscriber or publisher

Functions are registered using the Python `decorator` syntax ([learn more here](https://www.geeksforgeeks.org/python/decorators-in-python/)). The decorator is a function that takes in a function and extends it, letting arbitrary functions conform to a normalized interface for the subscription and publication managers.

The `subscribe` function from the `SubscriptionManager` class:

```Python
def subscribe(self, message_type):
    """
    Decorator: Add a function to the mailing list for message_type or attribute. 
    Usage: Decorate with @__class__.__name__.subscribe('message_type') to register a function.
    """

    def wrap(function):
        with self.lock:
            if message_type not in self.message_subscribers:
                self.message_subscribers[message_type] = []
            self.message_subscribers[message_type].append(function)
        return function

    return wrap
```

The `subscibe` function's parameter is a `mavlink.MAVLink_message.msgname` `message_type` (not type annotated in the snippet above). The `wrap` function's parameter is an arbitrary function, `function`. The dictionary has an entry for each `message_type` which is a list of functions. If the `message_type` doesn't yet have an entry, it is created. The function is added to the list of functions that should be called when a message of `message_type` is received.

An example of the decorator being used to subscribe to a message type:

```Python
@vehicle.subscribe(mavlink.MAVLink_local_position_ned_message.msgname)
def update_local_position(message: mavlink.MAVLink_local_position_ned_message):
    with self.lock:
        self.x_north_m = message.x
        self.y_east_m = message.y
        self.z_down_m = message.z  # Negative altitude
```

`@vehicle.subscribe()` is used to decorate `update_local_position()`. The `message_type` is `mavlink.MAVLink_local_position_ned_message.msgname`, which is a string version of the name. `update_local_position()` takes a `mavlink.MAVLink_local_position_ned_message` object, which we can then access the fields of with `message.x`. The fields are defined in the Pymavlink library (ctrl-click in the code to jump to it's definition) or at the MAVLink website. For example, this is the definition for the `MAVLink_local_position_ned_message` class in Pymavlink:

```Python
class MAVLink_local_position_ned_message(MAVLink_message):
    """
    The filtered local position (e.g. fused computer vision and
    accelerometers). Coordinate frame is right-handed, Z-axis down
    (aeronautical frame, NED / north-east-down convention)
    """

    id = MAVLINK_MSG_ID_LOCAL_POSITION_NED
    msgname = "LOCAL_POSITION_NED"
    fieldnames = ["time_boot_ms", "x", "y", "z", "vx", "vy", "vz"]
    ordered_fieldnames = ["time_boot_ms", "x", "y", "z", "vx", "vy", "vz"]
    fieldtypes = ["uint32_t", "float", "float", "float", "float", "float", "float"]
    fielddisplays_by_name: Dict[str, str] = {}
    fieldenums_by_name: Dict[str, str] = {}
    fieldunits_by_name: Dict[str, str] = {"time_boot_ms": "ms", "x": "m", "y": "m", "z": "m", "vx": "m/s", "vy": "m/s", "vz": "m/s"}
    native_format = bytearray(b"<Iffffff")
    orders = [0, 1, 2, 3, 4, 5, 6]
    lengths = [1, 1, 1, 1, 1, 1, 1]
    array_lengths = [0, 0, 0, 0, 0, 0, 0]
    crc_extra = 185
    unpacker = struct.Struct("<Iffffff")
    instance_field = None
    instance_offset = -1

    def __init__(self, time_boot_ms: int, x: float, y: float, z: float, vx: float, vy: float, vz: float):
        MAVLink_message.__init__(self, MAVLink_local_position_ned_message.id, MAVLink_local_position_ned_message.msgname)
        self._fieldnames = MAVLink_local_position_ned_message.fieldnames
        self._instance_field = MAVLink_local_position_ned_message.instance_field
        self._instance_offset = MAVLink_local_position_ned_message.instance_offset
        self.time_boot_ms = time_boot_ms
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz

    def pack(self, mav: "MAVLink", force_mavlink1: bool = False) -> bytes:
        return self._pack(mav, self.crc_extra, self.unpacker.pack(self.time_boot_ms, self.x, self.y, self.z, self.vx, self.vy, self.vz), force_mavlink1=force_mavlink1)
```

Publishing a message works the same way. The publisher is used to report the heartbeat of the drone system, and will be important when setting up the communication link between InvestiGator and NaviGator. An example of a subscription is taken from MAVConnection:

```Python
@self.publish('HEARTBEAT', 1)
def publish_heartbeat():
    with self.heartbeat_lock:
        self.mav.heartbeat_send(
            type=self.mav_type,
            autopilot=mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=self._system_status
        )
```

Here, the message name is typed directly, instead of using the Pymavlink class's msgname field. The frequency is set to 1Hz, and the function calls the `heartbeat_send()` function provided by Pymavlink with the message fields set by the passed in arguments.

##### Custom Protocol within MAVLink

MAVLink provides a few message types with undefined fields to allow setting up a custom sub-protocol within the MAVLink protocol. The custom protocol can be in the form of a custom command type, `mavlink.MAV_CMD_USER_x` or through one of the message types `TUNNEL`, [see it's definition here](https://mavlink.io/en/messages/common.html#TUNNEL).

System commands and mission commands are communicated via custom `MAV_CMD_USER_x`. The definition of these commands can be found in `/src/InvestiGator/constants.py`. The command protocol is desribed further below.

The `TUNNEL` message's fields describe a payload_length and an arbitrary payload. This can be used to encode MIL's custom serial protocol within the MAVLink transport bewteen InvestiGator and NaviGator. Other message types can be explored for a better fit, but `TUNNEL` seems to be the best situated.

##### Command Protocol

MAVLink defines a [Command Protocol microservice](https://mavlink.io/en/services/command.html) that outlines how an entity on the MAVLink network can acknowlege commands received. Functions subscribe to the command message and send back a command_ack message with success or failure.

When communicating between the companion computer and the flight controller, thread-safe events, `ack_event` in the below snippet, are used to communicate between the `main thread` and the `subscription thread` to report this ack. The `send_command()` function in `src/InvestiGator/vehicle.py` creates a subscription that contains cross-thread `nonlocal` `ack_result` variable. It then sends the `command_long` message type, and waits for `ack_event` to be set. If an ack is not received, it resends the message a configurable number of times. If a timeout occurs, the mission is canceled due to loss of connection from the companion computer and the flight controller. If the event is set, the `on_ack` function is taken off the subscription list and `ack_result` is returned.

```Python
def send_command(self, command: int, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0, target_system=1, target_component=0, retries:int = 3, retry_timeout_s: float=1.0):
    """
    Send a MAVLink COMMAND_LONG message. Wait for COMMAND_ACK to be received.
    Retry up to retries times if not received within retry_timeout_s seconds. Maximum wait is retries * retry_timeout_s seconds.
    Return True if command acknowledged, False otherwise.
    """
    ack_event = Event()
    ack_result = None

    def on_ack(message: mavlink.MAVLink_command_ack_message):
        if message.command == command:
            nonlocal ack_result
            ack_result = message.result
            ack_event.set()

    self.subscribe(mavlink.MAVLink_command_ack_message.msgname)(on_ack)

    for attempt in range(retries):
        # TODO: Log retries
        ack_event.clear()
        ack_result = None

        self.mav.command_long_send(
            target_system=target_system,
            target_component=target_component,
            command=command,
            confirmation=attempt,
            param1=param1,
            param2=param2,
            param3=param3,
            param4=param4,
            param5=param5,
            param6=param6,
            param7=param7
        )
            
        if self.wait_for_condition(lambda: ack_event.is_set(), timeout_s=retry_timeout_s):
            break
        
        elif self.cancel_mission_event.is_set():
            self.unsubscribe(mavlink.MAVLink_command_ack_message.msgname, on_ack)
            return False
            
    self.unsubscribe(mavlink.MAVLink_command_ack_message.msgname, on_ack)
    return ack_result == mavlink.MAV_RESULT_ACCEPTED
```

Commands sent to the `flight controller` by the `companion computer` are routed through this `send_command` function. [The command types are defined here](https://mavlink.io/en/messages/common.html#mav_commands). Functions for controlling InvestiGator are defined in `src/InvestiGator/vehicle.py` such as:

- `def takeoff(self, alt_m, timeout_s=30, threshold_m=0.5)`
- `def land(self, timeout_s=30.0)`

Read through `vehicle.py` to see all available functions.

Commands are also used to communicate between the `ground control` and the `companion computer` to select missions to run. This uses a similar scheme as between the `companion computer` and `flight controller`. In `src/main.py`, a subscription is made for `command_long` messages. The main loop waits for a message with the `MIL_MISSION_CMD` command type, which sets an event that calls `accept_mission`. This function checks that the requested mission is in the `companion computer`'s mission list, and it decides to start or ignore the request. An ack is then sent to the `ground_control` if the mission is started. Another ack is sent when the mission ends, either in success or failure. 

Commands are again used for system commands to InvestiGator. The following system commands are defined:

1. Ping
2. Set GUIDED
3. Set uncontrolled
4. Set cancel mission

Ping is used to check the connection between the `ground control` and the `companion computer`. Set GUIDED is used to put InvestiGator into an autonomous-ready state, and is required as a "soft reset" before the `companion computer` accepts any missions. Set uncontrolled is used to cancel the mission and place InvestiGator in an emergency stop state, where it returns to launch and waits for a reset. Cancel mission is used to stop the current mission and have the `companion computer` wait for another mission command, without needing to be set to GUIDED mode again.

##### Missions

Missions are functions that take in a `VehicleManager` class and issue commands to the `flight controller` from the `companion computer`. Missions are decorated with the `@mission(name: str)` function to automatically add them to a list of missions, of the `Mission dataclass`. The `ground control` and `companion computer` use this list to synchronize their missions and make sure they agree on a mapping of mission numbers to mission functions. A mission number (index to the `MISSIONS` list) is sent via the `MAVLink_command_long_message` using a `MAV_CMD_USER_X` command type defined in `src/InvestiGator/constants.py`. Param1 of this message contains the mission number. Missions are placed in `src/missions.py`. The `Mission dataclass` and `MISSIONS` list are in `src/missions.py`:

```Python
@dataclass
class Mission():
    name: str
    function: Callable

MISSIONS: list[Mission] = []

def mission(name: str):
    """
    Register a mission as a function in MISSIONS.
    Decorator usage: @mission("name") above a mission function definition.
    """
    def wrap(function):
        MISSIONS.append(Mission(name, function))
        return function
    return wrap
```

`MISSIONS` is a list of `Mission` objects. A `Mission` object has a name and a function. The `mission(name: str)` function appends a `Mission` object to the `MISSIONS` list, instantiated with the passed in name and function arguments.

A simple mission is shown here:

```Python
@mission("Arm")
def arm(vehicle: VehicleManager):
    if not vehicle.set_mode(target_mode = "GUIDED"):
        return False
    return vehicle.arm()
```

The mission is named "Arm" and the function `arm(vehicle: VehicleManager)` is passed to the decorator to add it to the `MISSIONS` list. The `VehicleManager` argument is then used to access the different functions to control the `flight controller`. In this case, the `flight controller` is commanded to set it's mode to guided. Each mission should return a true or false, indicating the success of the mission.

# References and Libraries Used

- [MAVLink](https://mavlink.io/en/)
- [ArduCopter Wiki](https://ardupilot.org/copter/)
- [RFD900x Documentation](https://files.rfdesign.com.au/docs/)
- [RFD900x Tools](https://files.rfdesign.com.au/tools/)
- [Textual Library](https://textual.textualize.io/)
- [Mission Planner](https://ardupilot.org/planner/)
