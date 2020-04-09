import pybullet as pb
import time
import pybullet_data
import os

# pybullet works on the basis of client-server interactions
# so, we need to establish a connection to the pybullet GUI
# there is an option of pb.DIRECT (not very helpful, I haven't tried)
physicsClient = pb.connect(pb.GUI)

# sets the path to pybullet_data which contains some inbuilt stuff for pybuller
pb.setAdditionalSearchPath(pybullet_data.getDataPath())

# loads the simulation plane where objects are placed and simulation is done
# This is similar to the gazebo empty world
planeId = pb.loadURDF('plane.urdf')

# set the gravity in the simulation world to mimic real world
# possible to make objects float similar to outer spae by setting 
# gravity to (0,0,0)
pb.setGravity(0, 0, -9.8)

# sets the server to proceed in steps as per the Real Time Clock(RTC)
pb.setRealTimeSimulation(1)

# set starting position
cubeStartPos = [0, 0, 1]

# set starting orientation
cubeStartOrientation = pb.getQuaternionFromEuler([0, 0, 0])

# initialize object r2d2 from starwars at the provided starting position 
# and orientation
boxId = pb.loadURDF("r2d2.urdf", cubeStartPos, cubeStartOrientation)

# wait for 10 secs to keep the GUI up to see what's going on
time.sleep(10)

# once we are done, disconnect from the server
pb.disconnect()
