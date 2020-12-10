"""supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import copy
from controller import Supervisor
import numpy as np
import math


supervisor = None
robot_node = None
target_node = None

def init_supervisor():
    global supervisor, robot_node, target_node

    # create the Supervisor instance.
    supervisor = Supervisor()
    # do this once only
    root = supervisor.getRoot() 
    root_children_field = root.getField("children") 
    robot_node = None
    target_node = None
    for idx in range(root_children_field.getCount()):
        if root_children_field.getMFNode(idx).getDef() == "Goal": #name of bb8 robot
            robot_node = root_children_field.getMFNode(idx)
        if root_children_field.getMFNode(idx).getDef() == "EPUCK": #name of epuck
            target_node = root_children_field.getMFNode(idx) 

    start_translation = copy.copy(robot_node.getField("translation").getSFVec3f())
    start_rotation = copy.copy(robot_node.getField("rotation").getSFRotation())


def supervisor_reset_to_home():
    global robot_node
    pos_field = robot_node.getField("translation")
    pos_field.setSFVec3f(start_translation)
    pos_field = robot_node.getField("rotation")
    pos_field.setSFRotation(start_rotation)
    supervisor.resetPhysics()
    print("Supervisor reset robot to start position")


def supervisor_get_obstacle_positions():
    global supervisor
    coords_list = []

    root_children_field = supervisor.getRoot().getField("children") 
    for idx in range(root_children_field.getCount()):
        #print(root_children_field.getMFNode(idx).getDef())
        #y ones 
        if root_children_field.getMFNode(idx).getDef() == "WALL_3_Solid_y" or root_children_field.getMFNode(idx).getDef() == "WALL_1_Solid_y" or root_children_field.getMFNode(idx).getDef() == "WALL_1_Solid_x" or root_children_field.getMFNode(idx).getDef() == "WALL_3_Solid_x":
            box_node = root_children_field.getMFNode(idx)
            if root_children_field.getMFNode(idx).getDef() == "WALL_3_Solid_y" or root_children_field.getMFNode(idx).getDef() == "WALL_1_Solid_y":
                #index 2 is where the change is
                size = ("y", root_children_field.getMFNode(idx).getField("children").getMFNode(0).getField("geometry").getSFNode().getField("size").getSFVec3f()[2])
            else:
                size = ("x", root_children_field.getMFNode(idx).getField("children").getMFNode(0).getField("geometry").getSFNode().getField("size").getSFVec3f()[0])
            #print(root_children_field.getMFNode(idx).getField("children").getMFNode(0).getField("geometry").getSFNode().getField("size").getSFVec3f())
            
            box_coords = box_node.getField("translation").getSFVec3f()
            coords_list.append((np.array([box_coords[0], 1 - box_coords[2]]),size))

    return coords_list

def supervisor_get_target_pose():
    '''
    Returns target position relative to the robot's current position.
    Do not call during your solution! Only during problem setup and for debugging!
    '''

    target_position = np.array(target_node.getField("translation").getSFVec3f())
    target_pose = np.array([target_position[0], 1. - target_position[2], target_node.getField("rotation").getSFRotation()[3] + math.pi/2.])
    # print("Target pose relative to robot: %s" % (str(target_pose)))
    return target_pose

    
def supervisor_get_robot_pose():
    """
    Returns robot position
    """
    robot_position = np.array(robot_node.getField("translation").getSFVec3f())
    robot_pose = np.array([robot_position[0], 1. - robot_position[2], robot_node.getField("rotation").getSFRotation()[3]+math.pi/2])
    return robot_pose