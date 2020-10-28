#!/usr/bin/python
################################################################################
# Script to test the Pimoroni FanShim LED.
# 
# Tom Hebel, 2020
################################################################################

import sys
import time
import argparse
from gpioLib import *

#########################################################################

GPIO_FANSHIM_PIN_CLOCK  = 14
GPIO_FANSHIM_PIN_DATA   = 15
GPIO_FANSHIM_PIN_BUTTON = 17
GPIO_FANSHIM_PIN_FAN    = 18

#########################################################################

class FanShim:
    gpio = GPIO()

    #########################################################################
    # toggleFan --
    #
    #   Turn on/off the FanShim fan.
    #########################################################################
    def toggleFan(self):
        self.gpio.funcSelPin(GPIO_FANSHIM_PIN_FAN, 'OUT')
        val = self.gpio.levPin(GPIO_FANSHIM_PIN_FAN)
        if val:
            self.gpio.clrPin(GPIO_FANSHIM_PIN_FAN)
        else:
            self.gpio.setPin(GPIO_FANSHIM_PIN_FAN)

    #########################################################################
    # setLED --
    #
    #   Turn on/off the FanShim LED.
    #########################################################################
    def setLED(self, red, green, blue):
        buf = [0, 0, 0, 0,
            255, blue, green, red,
            ~0, ~0, ~0, ~0]
        self.gpio.funcSelPin(GPIO_FANSHIM_PIN_CLOCK, 'OUT')
        self.gpio.funcSelPin(GPIO_FANSHIM_PIN_DATA, 'OUT')
        self.gpio.clrPin(GPIO_FANSHIM_PIN_CLOCK)
        self.gpio.clrPin(GPIO_FANSHIM_PIN_DATA)
        for i in range(len(buf)):
            byte = buf[i]
            for j in range(8):
                bit = (byte & 0x80) > 0
                if bit:
                    self.gpio.setPin(GPIO_FANSHIM_PIN_DATA)
                else:
                    self.gpio.clrPin(GPIO_FANSHIM_PIN_DATA)
                self.gpio.setPin(GPIO_FANSHIM_PIN_CLOCK)
                time.sleep(0)
                self.gpio.clrPin(GPIO_FANSHIM_PIN_CLOCK)
                time.sleep(0)
                byte <<= 1

    #########################################################################
    # flashLED --
    #
    #   Flash the FanShim LED.
    #########################################################################
    def flashLED(self, red, green, blue, durationSec, intervalMs):
        bufOn = [0, 0, 0, 0,
                255, blue, green, red,
                ~0, ~0, ~0, ~0]
        bufOff = [0, 0, 0, 0,
                255, 0, 0, 0,
                ~0, ~0, ~0, ~0]
        curBuf = bufOff
        intervalUs = intervalMs * 1000
        self.gpio.funcSelPin(GPIO_FANSHIM_PIN_CLOCK, 'OUT')
        self.gpio.funcSelPin(GPIO_FANSHIM_PIN_DATA, 'OUT')
        self.gpio.clrPin(GPIO_FANSHIM_PIN_CLOCK)
        self.gpio.clrPin(GPIO_FANSHIM_PIN_DATA)
        startTime = time.monotonic()
        while True:
            curTime = time.monotonic()
            if curTime - startTime > durationSec:
                return
            if curBuf == bufOff:
                curBuf = bufOn
            else:
                curBuf = bufOff
            for i in range(len(curBuf)):
                byte = curBuf[i]
                for j in range(8):
                    bit = (byte & 0x80) > 0
                    if bit:
                        self.gpio.setPin(GPIO_FANSHIM_PIN_DATA)
                    else:
                        self.gpio.clrPin(GPIO_FANSHIM_PIN_DATA)
                    self.gpio.setPin(GPIO_FANSHIM_PIN_CLOCK)
                    time.sleep(0)
                    self.gpio.clrPin(GPIO_FANSHIM_PIN_CLOCK)
                    time.sleep(0)
                    byte <<= 1
            time.sleep(intervalMs / 1000)

    #########################################################################
    # pulseLED --
    #
    #   Pulse the FanShim LED.
    #########################################################################
    def pulseLED(self, col, durationSec, periodMs):
        buf = [0, 0, 0, 0,
            255, 0, 0, 0,
            ~0, ~0, ~0, ~0]
        curCol = [0, 0, 0]
        intervalSec = (periodMs / 1000) / 256
        self.gpio.funcSelPin(GPIO_FANSHIM_PIN_CLOCK, 'OUT')
        self.gpio.funcSelPin(GPIO_FANSHIM_PIN_DATA, 'OUT')
        self.gpio.clrPin(GPIO_FANSHIM_PIN_CLOCK)
        self.gpio.clrPin(GPIO_FANSHIM_PIN_DATA)
        sign = 1
        startTime = time.monotonic()
        while True:
            curTime = time.monotonic()
            if curTime - startTime > durationSec:
                return
            for i in range(len(buf)):
                byte = buf[i]
                for j in range(8):
                    bit = (byte & 0x80) > 0
                    if bit:
                        self.gpio.setPin(GPIO_FANSHIM_PIN_DATA)
                    else:
                        self.gpio.clrPin(GPIO_FANSHIM_PIN_DATA)
                    self.gpio.setPin(GPIO_FANSHIM_PIN_CLOCK)
                    time.sleep(0)
                    self.gpio.clrPin(GPIO_FANSHIM_PIN_CLOCK)
                    time.sleep(0)
                    byte <<= 1
            if curCol[col] == 255:
                sign = -1
            elif curCol[col] == 0:
                sign = 1
            curCol[col] += sign
            buf[5] = curCol[0]
            buf[6] = curCol[1]
            buf[7] = curCol[2]
            time.sleep(0)

    #########################################################################
    # gradientLED --
    #
    #   Smoothly transition from blue to green to red to blue.
    #########################################################################
    def gradientLED(self, durationSec, periodMs):
        buf = [0, 0, 0, 0,
            255, 0, 0, 0,
            ~0, ~0, ~0, ~0]
        intervalSec = (periodMs / 1000) / 256
        col = [255, 0, 0]
        self.gpio.funcSelPin(GPIO_FANSHIM_PIN_CLOCK, 'OUT')
        self.gpio.funcSelPin(GPIO_FANSHIM_PIN_DATA, 'OUT')
        self.gpio.clrPin(GPIO_FANSHIM_PIN_CLOCK)
        self.gpio.clrPin(GPIO_FANSHIM_PIN_DATA)
        k = 1
        startTime = time.monotonic()
        while True:
            curTime = time.monotonic()
            if curTime - startTime > durationSec:
                return
            for i in range(len(buf)):
                byte = buf[i]
                for j in range(8):
                    bit = (byte & 0x80) > 0
                    if bit:
                        self.gpio.setPin(GPIO_FANSHIM_PIN_DATA)
                    else:
                        self.gpio.clrPin(GPIO_FANSHIM_PIN_DATA)
                    self.gpio.setPin(GPIO_FANSHIM_PIN_CLOCK)
                    time.sleep(0)
                    self.gpio.clrPin(GPIO_FANSHIM_PIN_CLOCK)
                    time.sleep(0)
                    byte <<= 1
            if col[k] == 255:
                k = (k + 1) % 3
            if k == 0:
                col[2] -= 1
                col[0] += 1
            elif k == 1:
                col[0] -= 1
                col[1] += 1
            elif k == 2:
                col[1] -= 1
                col[2] += 1
            buf[5] = col[0]
            buf[6] = col[1]
            buf[7] = col[2]
            time.sleep(0)

#########################################################################
# main --
#
#   Main entry point.
#########################################################################
def main(argv):
    fanShim = FanShim()
    progDesc = ('Utility for controlling the Pimoroni FanShim.'
                '\n\nCommands:'
                '\nfan'
                '\nset\t<red> <green> <blue>'
                '\nflash\t<red> <green> <blue>'
                '\npulse\t<[red|green|blue]>'
                '\ngradient'
                '\n\nAll colors 0-255.')
    parser = argparse.ArgumentParser(prog='gpio',
                                     description=progDesc,
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('command', help='interact with the FanShim accessory')
    parser.add_argument('args',
                        nargs='*',
                        help='arguments for the command')
    args = parser.parse_args()
    if args.command == 'fan':
        fanShim.toggleFan()
    elif args.command == 'set':
        red = int(args.args[0])
        green = int(args.args[1])
        blue = int(args.args[2])
        fanShim.setLED(red, green, blue)
    elif args.command == 'flash':
        red = int(args.args[0])
        green = int(args.args[1])
        blue = int(args.args[2])
        fanShim.flashLED(red, green, blue, 5, 500)
        fanShim.setLED(0, 0, 0)
    elif args.command == 'pulse':
        arg = args.args[0]
        cols = ['blue', 'green', 'red']
        if arg not in cols:
            print('Invalid color: {}'.format(col))
            print('Must be one of: red, green, blue')
            exit(1)
        col = cols.index(arg)
        fanShim.pulseLED(col, 5, 500)
        fanShim.setLED(0, 0, 0)
    elif args.command == 'gradient':
        fanShim.gradientLED(8, 2000)
        fanShim.setLED(0, 0, 0)

if __name__ == '__main__':
    main(sys.argv)