# Install the rpi_lgpio library: uv pip install rpi-lgpio OR uv add rpi-lgpio
# API: https://rpi-lgpio.readthedocs.io/en/latest/api.html
# RPi 5 Pinout: https://pinout.xyz/pinout/pin11_gpio17/
# Magnet documentation: https://fluxgrip.zubax.com/chapters/tutorials/quick_start_fg40_analog.html#voltage-level-control

from RPi import GPIO

PIN17: int = 17
OFF: int = 0
ON: int = 1

def init_gpio():
    """
    Set Board Pin 11/BCM Pin 17 to output for usage with GPIO magnet.
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(channel=PIN17, direction=GPIO.OUT, pull_up_down=GPIO.PUD_OFF, initial=OFF)


def cleanup_gpio():
    """
    Wrapper for Rpi.GPIO cleanup function. Resets GPIO pins to default values.
    """
    GPIO.cleanup(channel=PIN17)


def magnet_control(setting: int):
    """
    Turn on or off the magnet via GPIO. 3.3V = ON, 0V = OFF.
    """
    if setting == ON:
        GPIO.output(PIN17, GPIO.HIGH)
    elif setting == OFF:
        GPIO.output(PIN17, GPIO.LOW)