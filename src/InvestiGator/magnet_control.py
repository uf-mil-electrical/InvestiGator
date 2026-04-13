# Tutorial: https://oneuptime.com/blog/post/2026-03-02-how-to-configure-gpio-access-on-ubuntu-for-raspberry-pi/view
    # Install the following:
    # sudo apt-get install -y gpiod libgpiod-dev
    # sudo apt install liblgpio-dev
    # Then follow the tutorial above for configuring GPIO permissions
# API: https://gpiozero.readthedocs.io/en/latest/api_output.html#outputdevice
# RPi 5 Pinout: https://pinout.xyz/pinout/pin11_gpio17/
# Magnet documentation: https://fluxgrip.zubax.com/chapters/tutorials/quick_start_fg40_analog.html#voltage-level-control

try:
    from gpiozero import Device, OutputDevice
    from gpiozero.pins.lgpio import LGPIOFactory
    from gpiozero import Device
    Device.pin_factory = LGPIOFactory()


    PIN17: int = 17
    OFF: int = 0
    ON: int = 1
    magnet = OutputDevice(PIN17)

    def magnet_control(setting: int):
        """
        Turn on or off the magnet via GPIO. 3.3V = ON, 0V = OFF.
        """
        if setting == ON:
            magnet.on()
        elif setting == OFF:
            magnet.off()

except ImportError:
    pass