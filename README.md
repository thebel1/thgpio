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
Branch main: https://youtu.be/Mw8L1XAi8qA?t=170

Branch fanShimButton: https://youtu.be/YjSxgKr7gZg

Branch charDev: https://youtu.be/Ri1py3AzIsU?t=107

### Installation

To install the driver, download the VIB file in the build directory and copy it to your RPi.

Install it as described here: https://kb.vmware.com/s/article/2008939

I would recommend download the Python library in ./pyUtil/pyLib/ as well so you don't have to worry about bit twiddling and calculating addresses yourself.
