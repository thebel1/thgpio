# thgpio
Native ESXi driver for RPi 4's GPIO interface.

Allows controlling the pins of the Raspberry Pi 4's GPIO interface.

Still very much work-in-progress!

Branch main contains:
- the base GPIO hardware driver
- a Python library for interacting with the GPIO MMIO memory directly
- a Python utility for interacting with the GPIO interface
- a Python utility for interacting with the Pimoroni FanShim

### Videos

These demos use the Pimoroni FanShim as an example. Any GPIO accessory should work analogously.

Branch main: https://youtu.be/Mw8L1XAi8qA?t=170

Branch fanShimButton: https://youtu.be/YjSxgKr7gZg

Branch charDev: https://youtu.be/Ri1py3AzIsU?t=107

### Installation

To install the driver, download the VIB file in the build directory and copy it to your RPi.

Install it as described here: https://kb.vmware.com/s/article/2008939

Note: you will need to reboot the RPi after installing the VIB.

I would recommend downloading the Python library in ./pyUtil/gpioLib/ as well so you don't have to worry about all the bit twiddling.
