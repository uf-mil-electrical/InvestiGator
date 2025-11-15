from mavconnection import MAVConnection
from enum import Enum
from pymavlink.dialects.v20 import ardupilotmega as mavlink


ground_control_address = "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTDCKG37-if00-port0"
rover_address = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00HS6A-if00-port0"

System = Enum('System', [('INVESTIGATOR', 37), ('ROVER', 44), ('NAVIGATOR', 47), ('SUBJUGATOR', 59), ('GROUND_CONTROL', 255)])


if __name__ == "__main__":

    rover_control = MAVConnection(address=rover_address, baud=57600, source_system=System.ROVER.value)

    while True:
        continue