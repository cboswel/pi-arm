#!/usr/bin/python3

'''
OK, You're gonna have to manually correct for the wrist by calculating the angle of the last limb.
We could use two separate chains, one to the wrist of the gripper and one to the joint before that. Then, you can just calculate the height difference between them and work out the angle. The wrist compensates, the extra distance is constant.
'''

from ikpy.chain import Chain
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

my_chain = Chain.from_urdf_file("AL5D.urdf")
coords = my_chain.inverse_kinematics([2, 2, 30]) #X, Y, Height
print(coords)

ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
my_chain.plot(coords, ax)
matplotlib.pyplot.show()
