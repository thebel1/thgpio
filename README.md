# thgpio
Native ESXi driver for RPi 4's GPIO interface.

Allows controlling the pins of the Raspberry Pi 4's GPIO interface.

Still very much work-in-progress!

Currently, there are two branches: main and charDev.

Branch main contains the base GPIO pin driver and logic to control the Pimoroni FanShim using the built-in button.

Video: https://youtu.be/YjSxgKr7gZg

Branch charDev allows controlling the FanShim using commands redirected into a character device.

Video: https://youtu.be/Ri1py3AzIsU?t=107
