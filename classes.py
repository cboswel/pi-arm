#!/usr/bin/env python3
import math
from ikpy.chain import Chain
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import ikpy.utils.plot as plot_utils
import time

neutral = [0, 0, 0]
ARM_EXTENSION = 0.3

class Arm():
    def __init__(self):
  #      self.timer = RepeatTimer(0.001, self.motion)
        self.chain = Chain.from_urdf_file("urdf/AL5D.urdf")
        self.forearm = Chain.from_urdf_file("urdf/forearm.urdf")
        self.current_position = neutral
        self.graphs = True
        self.joints = [0, 0, 0, 0]
        self.setJoints(neutral)

    def userInput(self):
        pass

    def validityCheck(self, polar):
        theta, rho, height = polar[:3]
        stretch = math.sqrt(rho ** 2 + height ** 2)
        if stretch > ARM_EXTENSION:
            return f"Those co-ordinates are out of reach! I went for {polar} which needs an arm of length {stretch}"
        if theta > 90 or theta < -90:
            return "Angle must be between -90 and 90 degrees"
        return 0

    def pol2cart(self, polar):
        theta, rho, height = polar[:3]
        theta += 90
        theta = math.radians(theta)
        x = rho * math.sin(theta)
        y = rho * math.cos(theta)
        z = height
        return (x, y, z)

    def wrist(self):
        wrist = self.chain.forward_kinematics(self.joints + 0)
        elbow = self.forearm.forward_kinematics(self.joints[:-1] + 0)
        wristCart = self.pol2cart([wrist[0][2], wrist[0][3], wrist[0][1]])
        elbowCart = [elbow[0][3], elbow[0][2], elbow[0][1]]
#        wristCart = [wrist[0][3], wrist[0][2], wrist[0][1]]
#        elbowCart = [elbow[0][3], elbow[0][2], elbow[0][1]]
        forearm_angle = 0
        required_wrist_angle = 0
        return required_wrist_angle

    def setJoints(self, target_position):
        joints = self.chain.inverse_kinematics(target_position)
        slowest = self.timing(joints)
        self.joints = joints
        self.wrist_tilt = self.wrist()
        #if self.graphs == True:
        #time.sleep(slowest)

    def timing(self, new_pos):
        movements = [abs(a - b) for a, b in zip(self.joints, new_pos)]
        speeds = [1, 1, 1, 1]
        times = [a * b for a, b in zip(movements, speeds)]
        slowest = max(times) 
        return slowest

    def draw(self, target_position):
        self.current_position = target_position
        fig, ax = plot_utils.init_3d_figure()
        self.chain.plot(self.chain.inverse_kinematics(target_position), ax, target=target_position)
        plt.xlim(-0.3, 0.3)
        plt.ylim(-0.3, 0.3)
        ax.set_zlim(-0.3, 0.3)
        plt.show()

    def speeds(self):
        pass

    """
    def motion(self, pi, servos, angles):
        for servo in len(servos):
            pi.set_angle(servos[servo], angles[servo])
    """

    def motion(self):
        pass

    def gripper(self, command):
        if command == "open":
            self.motion()
            print("Gripper open!")
        if command == "close":
            self.motion()
            print("Gripper closed!")

    def move(self, polar):
        valid = self.validityCheck(polar)
        if valid != 0:
            print(valid)
        else:
            cart = self.pol2cart(polar)
            self.setJoints(cart)
            print(f"moved to co-ords {cart}")
            self.draw(cart)
            self.motion()
"""
class RepeaTimer(Timer):
    def __init__(self):
        pass

    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)
"""
class Board():
    def __init__(self, stacks=3):
        self.cup_depth = 0.02
        self.cup_height = 0.05
        self.stacks = stacks
        self.params = {}
        self.readConfig()

    def readConfig(self):
        self.r = 0.1
        self.stack_positions = [-85, -30, 0, 30, 85]
        self.stack_heights = [5, 0, 0, 0, 0]
        self.debug = True

    def writeConfig(self):
        pass

    def liftHeight(self):
        current_tallest_stack = max(self.stack_heights)
        return (current_tallest_stack * self.cup_depth) + self.cup_height

class Pi():
    def __init__(self):
        import RPi.GPIO as GPIO
        import time
        self.servo_pins = [1, 2, 3, 4, 5, 6]
        self.PWMpins = []
        self.warningLED = [7]
        self.controllerStuff = range(8, 10)
       
        # Set the GPIO mode to BCM
        GPIO.setmode(GPIO.BCM)

        # Set the GPIO pin for the servo
        for pin in self.servo_pins:
            GPIO.setup(pin, GPIO.OUT)
        # Create a PWM instance with a frequency of 50Hz
            self.PWMpins.append(GPIO.PWM(pin, 50))

    def set_angle(self, servo, angle):
        pin = self.PWMpins[servo]
        duty_cycle = (angle / 18) + 2
        pin.ChangeDutyCycle(duty_cycle)
        time.sleep(0.5)  # Give the servo some time to move


if __name__ == "__main__":
    if len(sys.argv) == 4:
        args = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]
        arm = Arm()
        arm.graphs = False
        ans = arm.pol2cart(args)
        print(ans)
        arm.draw(ans)
    else:
        tests = [[0, 0.2, 0.05], [0, 0.2, 0.3], [-30, 0.1, 0.1]]
        #tests = [[0, 0.1, 0.05]]
        arm = Arm()
        for test in tests:
            valid = arm.validityCheck(test)
            if valid != 0:
                print(valid)
            else:
                cart = arm.pol2cart(test)
                arm.setJoints(cart)
            
