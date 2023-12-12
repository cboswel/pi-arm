#!/bin/python3
import sys
import pdb
import serial
import time
import subprocess
max_speed = 1000
"""
Default command is hard coded. Then it is editted when UDP packet received to simulate GUI. 
Constantly sent via serial. (print for debugging)

References: https://wiki.lynxmotion.com/info/wiki/lynxmotion/download/servo-erector-set-system/ses-electronics/ses-modules/ssc-32u/WebHome/lynxmotion_ssc-32u_usb_user_guide.pdf

https://github.com/Phylliade/ikpy
"""

def send(string):
    ser = serial.Serial("COM7", 9600)
    ser.write(bytearray(string, 'ascii'))

class Command:
    def __init__(self):
        self.words = [[1500, max_speed]] * 6
        self.string = ""

    def to_string(self):
        self.string = ""
        for _ in range (0, 6):
            body = self.words[_]
            self.string += f"#{_ + 1}P{body[0]}S{body[1]}"
        self.string += "\n"

def main():
    command = Command()
    while True:
        time.sleep(1)
        command.to_string()
        print(command.string)
    return

if __name__ == "__main__":
    main()
