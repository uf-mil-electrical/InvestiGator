from pymavlink import mavutil
import test_radio

# Message for GPS coordinates through the follow the path task
    # Drone can get a bunch of gps coordinates and calculate a rough path for the rover (not necessary for boat probably)
# Message for wildlife encounter
    # Need color, gps
# Message for location of light tower
    # Need gps
# Message for location of docks
    # Need gps (for each dock?)
# Message for launching
    # Need to change mode to guided
# Message for heartbeat information (stowed, deployed, faulted) (May just use the default heartbeat)
    # Heartbeat by default has system mode. If heartbeat not heard in 3 seconds, consider faulted
# Message for UAV Replinishment
    # Not picked up, picked up, delivered
    # Color of tin to pick up
# Message for UAV Search and Report
    # Need GPS of object
        # N/S, E/W indicator
    # Identity of object


# Useful messages
    # SET_GPS_GLOBAL_ORIGIN
        # Sets the GPS location for the local origin of the drone (0, 0, 0)
        # Can use this to give the drone and other robot the same origin
    # SAFETY_SET_ALLOWED_AREA
        # Sets a cube where way points are allowed. Drone will reject waypoints outside of this area
    # MESSAGE_ITEM_INT
        # Has gps coordinates as deg*10E7
        # Has 4 free params (can use to indicate which object)
        # Includes a lot of unecessary extras
        # Good for any message that needs GPS
    # SET_POSITION_TARGET_LOCAL_NED 
        # Set the target for the drone in local North East Down frame
        # Might use this for precision alignment and landing
    # SET_POSITION_TARGET_GLOBAL_INT
        # Same as above, but using GPS instead
    # BATTERY_STATUS
        # Gives voltage and current
    # GLOBAL_POSITION_INT
        # Get the current gps coordinates
    # MAV_CMD_DO_SET_HOME  
        # Set a new home position for the drone. This is where it will go when RTK is activated.
    # TRAJECTORY_REPRESENTATION_WAYPOINTS
        # Sends an array of trajectories for up to 5 way points
        # Might be useful for describing a possible path between points of interest?
        # In the NED local frame, so it is relative to the drone's origin GPS
    # TUNNEL 
        # Message for arbitrary variable-length data
        # We can use this to send a custom message based on the elctrical-software protocol
    # PLAY_TUNE_V2 
        # Plays a tune on the buzzer (could be fun)
        # Maybe a buzz on object detection or failure to detect

# Commands
    # COMMAND_INT 
        # The message struture to use when sending a command type message
        # Will be using this a lot
    # MAV_CMD_NAV_LOITER_UNLIM/MAV_CMD_NAV_LOITER_TIME 
        # Loiter at a gps location until canceled or time is up
    # MAV_CMD_NAV_RETURN_TO_LAUNCH 
        # Return to home gps location
    # MAV_CMD_NAV_LAND 
        # Begin landing sequence
    # MAV_CMD_NAV_TAKEOFF 
        # Take off
    # MAV_CMD_NAV_LAND_LOCAL 
        # Land at a location relative to the drone
        # Might use this for final landing in precision land
    # MAV_CMD_NAV_LOITER_TO_ALT 
        # Loiter wherever you are, and exit only once altitude has been reached
    # MAV_CMD_CONDITION_YAW 
        # Change yaw
    # MAV_CMD_DO_SET_MODE 
        # Change mode
    # MAV_CMD_DO_CHANGE_ALTITUDE 
    # MAV_CMD_DO_REPOSITION 
    # MAV_CMD_REQUEST_MESSAGE 
        # Use to get GPS
    # MAV_CMD_SPATIAL_USER_1 
        # User defined command, may use for our messages as needed

# Maybe:
# Message for password connection to drone. Probably use signing for this instead



# Establish a connection with the device
connection = mavutil.mavlink_connection('/dev/ttyUSB0')
connection.wait_heartbeat()

# Filter all messages for a specific message type
message = connection.recv_match()