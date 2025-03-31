import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from src.mavconnetion import MAVConnection
from vehicle_properties import Location

radio = "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTDCKG37-if00-port0"
simulation = 'udp:127.0.0.1:14550'


class VehicleManager:
    """
    Represents properties of a vehicle and handles communication with it.
    """

    def __init__(self, address, baud=115200):
        self.mav_connection = MAVConnection(address, baud)
        self.mode_map = mavutil.mode_mapping_byname(mavlink.MAV_TYPE_QUADROTOR)

        self.publish = self.mav_connection.publish
        self.publish_function = self.mav_connection.publish_function
        self.unpublish = self.mav_connection.unpublish
        self.subscribe = self.mav_connection.subscribe

        self.send_alias = self.mav_connection.mav_connection.mav

        self.location = Location(self)

        @self.publish('HEARTBEAT', 1)
        def publish_heartbeat():
            self.send_alias.heartbeat_send(
                type=mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                autopilot=mavlink.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=0
            )

    def takeoff(self, alt_m):
        """
        Wait for vehicle to arm and take off to alt_m meters.
        """
        pass

    def wait_for_armed(self):
        """
        Wait for vehicle to be armed.
        """
        self.send_alias.command_long_send(
            target_system=0,
            target_component=0,
            command=mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=1.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0, )

    def close(self):
        self.mav_connection.close()


if __name__ == "__main__":
    vehicle = VehicleManager("udp:127.0.0.1:14550")

    time.sleep(1)

    vehicle.mav_connection.mav_connection.mav.command_long_send(
        1,  # Target system ID
        0,  # Target component ID
        mavlink.MAV_CMD_DO_SET_MODE,  # Command to set mode
        0,  # No confirmation
        1,  # Mode ID for GUIDED mode
        4, 0, 0, 0, 0, 0  # Unused parameters
    )

    time.sleep(2)

    vehicle.mav_connection.mav_connection.mav.command_long_send(
        1,  # target system ID
        0,  # target component ID
        mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command ID to arm/disarm
        0,  # Confirmation flag (0 for no confirmation)
        1,  # Arm flag (1 to arm the system)
        0, 0, 0, 0, 0, 0  # Unused parameters
    )

    time.sleep(2)

    vehicle.mav_connection.mav_connection.mav.command_long_send(
        1, 0, mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 5)

    time.sleep(10)

    vehicle.mav_connection.mav_connection.mav.command_long_send(
        0,
        0,
        mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0,
    )

    time.sleep(2)

    vehicle.close()
