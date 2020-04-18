import pybullet as p
import time
import rospy
import numpy as np
# the following imports and template file can be found at
# https://github.com/cairo-robotics/cairo_simulator.git
from cairo_simulator.Simulator import Simulator, SimObject, ASSETS_PATH
from cairo_simulator.Manipulators import Sawyer

def main():
    rospy.init_node("CAIRO_Sawyer_Simulator")
    use_real_time = True

    sim = Simulator() # Initialize the Simulator

    # Add a table and a Sawyer robot
    table = SimObject("Table", ASSETS_PATH + 'table.sdf', (0.9, 0, 0), (0, 0, 1.5708)) # Table rotated 90deg along z-axis
    sawyer_robot = Sawyer("sawyer0", 0.35, 0, 0.8)

    # object that simulates the soldering iron
    iron_ID= p.loadURDF("../assets/URDF files/Soldering Iron_urdf/urdf/Soldering Iron.SLDPRT.urdf", [1.15,0,0.605], p.getQuaternionFromEuler([0, 0, 0]))
    
    # object that will simulate the human head
    head_ID = p.loadURDF('../assets/URDF files/7.6IN_Diam_Sphere.SLDPRT/urdf/7.6IN_Diam_Sphere.SLDPRT.urdf', [1.5, 0, 1], [0, 0, 0, 1])

    joint_config = sawyer_robot.solve_inverse_kinematics([0.9,0,1.5], [0,0,0,1])
    #sawyer_robot.move_to_joint_pos(joint_config)

    # get position and orientation of cube0
    ironPos , ironOrn = p.getBasePositionAndOrientation(iron_ID)

    # initial yaw orientation of human head
    yaw = 0.4

    # Loop until someone shuts us down
    while rospy.is_shutdown() is not True:
        sim.step()
        # apply force on the soldering iron to move it along y axis
        # or across the table
        force = 5.75*np.array([0, 1, 0])
        step_pos, step_orn = p.getBasePositionAndOrientation(iron_ID)

        # Apply force to the object/soldering iron 
        # make sure its position its position is not too far such that the human
        # head cannot turn that much
        if step_pos[0] < 1.155 and step_pos[1] < 0.20:
            p.applyExternalForce(objectUniqueId=iron_ID, linkIndex=-1,
                             forceObj=force, posObj=ironPos, flags=p.WORLD_FRAME)
        
            # change the yaw every step to simulate the rotation of head movement.
            if yaw > -0.3:
                yaw -= 2e-6
            
            # reset orientation at every step to prevent it from falling flat
            step_iron_pos, step_orn = p.getBasePositionAndOrientation(iron_ID)

            # calculate mid point between soldering iron and human head
            x1 = 1.50 # sphere x-coordinate
            y1 = 0    # sphere y-coordinate
            z1 = 1    # sphere z-coordinate

            x2 = step_iron_pos[0] # iron x-coordinate
            y2 = step_iron_pos[1] # iron y-coordinate
            z2 = step_iron_pos[2] # iron z-coordinate

            midpoint =( (x1 + x2)/2, (y1 + y2)/2 , (z1 + z2)/2 )


        # reset the position of the head object every step to prevent it from 
        # falling due to gravity
        p.resetBasePositionAndOrientation(bodyUniqueId=head_ID, posObj=(1.50, 0, 1), ornObj=(0, 0, yaw, 1) )
       
        # reset the orientation only of soldering iron every step to prevent it from 
        # falling due to gravity and to simulate it being held by a human hand
        p.resetBasePositionAndOrientation(bodyUniqueId=iron_ID, posObj=step_iron_pos, ornObj=p.getQuaternionFromEuler([1, 0, 0]) )
        
        # maintain the cube at the midpoint of head and soldering iron
        joint_config = sawyer_robot.solve_inverse_kinematics(midpoint, p.getQuaternionFromEuler([-1, 0, 0]))
        sawyer_robot.move_to_joint_pos(joint_config)

    p.disconnect()


if __name__ == "__main__":
    main()

