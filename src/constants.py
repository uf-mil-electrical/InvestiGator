SIMULATION_RADIO = 'udpin:0.0.0.0:14550' 
GROUND_CONTROL_RADIO_LINUX = "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTDCKG37-if00-port0"
ROVER_RADIO_LINUX = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00HS6A-if00-port0"

class Radio():
    SIMULATION = 'udpin:0.0.0.0:14550' 
    GROUND_CONTROL_LINUX = "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTDCKG37-if00-port0"
    ROVER_LINUX = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00HS6A-if00-port0"

class Robot():
    INVESTIGATOR = 37
    ROVER = 44
    NAVIGATOR = 47
    SUBJUGATOR = 59
    GROUND_CONTROL = 255