#!/usr/bin/env python3
import math
from ikpy.chain import Chain
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import ikpy.utils.plot as plot_utils
from classes import Arm, Board
import time
import random

neutral = [0, 0.075, 0]
ARM_EXTENSION = 0.3
GRIPPER_OFFSET = 0.08

def newCup(arm, board):
    coords = [board.stack_positions[0], board.r - GRIPPER_OFFSET, board.liftHeight() - board.cup_depth]
    print(f"should be pos0, {board.r - GRIPPER_OFFSET}, {board.liftHeight() - board.cup_depth}")
    arm.move(coords)
    arm.gripper("open")
    coords[1] += GRIPPER_OFFSET
    arm.move(coords)
    arm.gripper("close")
    coords[2] = board.liftHeight()
    arm.move(coords)
    board.stack_heights[0] -= 1
    if board.debug == True:
        print(f"got a new cup! Stacks are: {board.stack_heights}")

def to_neutral(arm, board):
    arm.move([arm.current_position[0], neutral[1], board.liftHeight()])
    arm.move(neutral)
    if board.debug == True:
        print("Now in neutral position")

def scan(arm, board):
    position = random.randint(1, 4)
    if board.debug == True:
        print(f"Cup scanned, belongs in position {position}")
    return position

def stack(arm, board, position):
    arm.move([arm.current_position[0], neutral[1], board.liftHeight()])
    coords = [board.stack_positions[position], neutral[1], board.liftHeight()]
    arm.move(coords)
    coords[1] = board.r
    coords[2] = (board.stack_heights[position] * board.cup_depth) + board.cup_height
    arm.move(coords)
    coords[2] -= board.cup_height
    arm.move(coords)
    arm.gripper("open")
    board.stack_heights[position] += 1
    if board.debug == True:
        print(f"Cup stacked at {position}! Stacks are: {board.stack_heights}")

def sort(arm, board):
    newCup(arm, board)
    to_neutral(arm, board)
    position = scan(arm, board)
    stack(arm, board, position)
    to_neutral(arm, board)    

if __name__ == "__main__":
    arm = Arm()
    board = Board()
    for cup in range(board.stack_heights[0]):
        sort(arm, board)
