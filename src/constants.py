class Radio():
    GROUND_CONTROL_SIM = 'udpin:127.0.0.1:14551' 
    DRONE_SIM = 'udpin:127.0.0.1:14552'
    GROUND_CONTROL_LINUX = "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTDCKG37-if00-port0"
    ROVER_LINUX = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00HS6A-if00-port0"

class Robot():
    INVESTIGATOR = 37
    ROVER = 44
    NAVIGATOR = 47
    SUBJUGATOR = 59
    GROUND_CONTROL = 255