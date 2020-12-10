"""robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import copy
from controller import Robot, Motor, Camera
import bb8_supervisor.py
import numpy as np

# create the Robot instance.
bb8_supervisor.init_supervisor()
#honestly I'm winging this
robot = bb8_supervisor.supervisor


#bb8 waits until it is "found" by the epuck
state = "lost"

#####Developement TODOs
#bb8_supervisor
    #needs to be able to read in the epucks location
    #so that we change flag to "found" when epuck location = bb8 location (it bumps into it)
#initialize bb8 data
    #Initialize Motors
        #BB8 motors:
            #body yaw motor
                # - to turn right
                # + to turn left
            #body pitch motor
                #goes forward/backward
            #head yaw motor
                # doesn't matter
#update odometry data for bb8 dimensions
    #need to find the right body pitch motor-burst to keep the bb8 just behind the epuck
    #idea is: locate epuck, rotate to face direction, motor burst, locate epuck, ...
   



# Map Variables
MAP_BOUNDS = [1.,1.] 
CELL_RESOLUTIONS = np.array([0.01, 0.01]) # 10cm per cell
NUM_X_CELLS = int(MAP_BOUNDS[0] / CELL_RESOLUTIONS[0])
NUM_Y_CELLS = int(MAP_BOUNDS[1] / CELL_RESOLUTIONS[1])

world_map = np.zeros([NUM_Y_CELLS,NUM_X_CELLS])

def populate_map(m):
    #i don't think it matters if we change this so I didn't
    obs_list = csci3302_lab5_supervisor.supervisor_get_obstacle_positions()
    #obs_size = 0.06 # 6cm boxes
    for obs, size_tuple in obs_list:
        #print(size_tuple)
        obs_size = size_tuple[1]
        if size_tuple[0] == "y":
            obs_coords_lower_left = [obs[0]-.045, obs[1] - size_tuple[1]/2 -.04]  
            obs_coords_upper_right = [obs[0]+.045, obs[1] + size_tuple[1]/2 + .04]
            obs_coords_lower_right = [obs[0]+.045, obs[1] - size_tuple[1]/2 - .04]  
            obs_coords_upper_left = [obs[0]-.045, obs[1] + size_tuple[1]/2 + .04]
                    
        else:
            obs_coords_lower_left = [obs[0]- size_tuple[1]/2 - .04, obs[1] - .045]  
            obs_coords_upper_right = [obs[0]+ size_tuple[1]/2 + .04, obs[1] + .045]
            obs_coords_lower_right = [obs[0] + size_tuple[1]/2 + .04, obs[1] - .045]  
            obs_coords_upper_left = [obs[0] - size_tuple[1]/2 - .04, obs[1] + .045]
        corners = [obs_coords_lower_left,obs_coords_upper_right,obs_coords_upper_left,obs_coords_lower_right]
        i = 0
        for x,y in corners:
            if x > 1:
                corners[i][0] = .99
            elif x < 0:
                corners[i][0] = 0
            if y > 1:
                corners[i][1] = .99
            elif y < 0:
                corners[i][1] = 0
            i = i+1
                
        obs_coords = np.linspace(obs_coords_lower_left, obs_coords_upper_left, 80)
        for coord in obs_coords:
            m[transform_world_coord_to_map_coord(coord)] = 1
        obs_coords = np.linspace(obs_coords_lower_left, obs_coords_lower_right, 80)
        for coord in obs_coords:
            m[transform_world_coord_to_map_coord(coord)] = 1
        obs_coords = np.linspace(obs_coords_lower_right, obs_coords_upper_right, 80)
        for coord in obs_coords:
            m[transform_world_coord_to_map_coord(coord)] = 1
        obs_coords = np.linspace(obs_coords_upper_left, obs_coords_upper_right, 80)
        for coord in obs_coords:
            m[transform_world_coord_to_map_coord(coord)] = 1


# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Odometry data TODO:
pose_x = 0
pose_y = 0
pose_theta = 0
left_wheel_direction = 0
right_wheel_direction = 0
#?

# Constants to help with the Odometry update
#?
WHEEL_FORWARD = 1
WHEEL_STOPPED = 0
WHEEL_BACKWARD = -1

# GAIN Values
theta_gain = 1.0
distance_gain = 0.3

#params
waypoints = []
puckstart = (0,0)
targetstart = (0,0)

EPUCK_MAX_WHEEL_SPEED = 0.12880519 # m/s
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_WHEEL_RADIUS = 0.0205 # ePuck's wheels are 0.041m in diameter.


# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
#leftMotor = robot.getMotor('left wheel motor')
#rightMotor = robot.getMotor('right wheel motor')
yawMotor = robot.getMotor('body yaw motor') #rotates clockwise, counterclockwise to turn left/right
pitchMotor = robot.getMotor('body pitch motor') #goes forward, backward
#only doing body motors, not incl head
#leftMotor.setPosition(float('inf'))
#rightMotor.setPosition(float('inf'))
yawMotor.setPosition(float('inf'))
pitchMotor.setPosition(float('inf'))
#just winging it
yawMotor.setVelocity(0.0)
pitchMotor.setVelocity(0.0)

MAX_VEL_REDUCTION = 0.25
#idk if we need this
#MAX_VEL_REDUCTION = 0.25


def main():
    global robot, state, sub_state, map
    global leftMotor, rightMotor, SIM_TIMESTEP, WHEEL_FORWARD, WHEEL_STOPPED, WHEEL_BACKWARDS
    global pose_x, pose_y, pose_theta, left_wheel_direction, right_wheel_direction

    last_odometry_update_time = None

    # Keep track of which direction each wheel is turning
    #need to update this to just rotate or something like that
    left_wheel_direction = WHEEL_STOPPED
    right_wheel_direction = WHEEL_STOPPED

    # Important IK Variable storing final desired pose
    target_pose = None # Populated by the supervisor, only when the target is moved.


    # Sensor burn-in period
    for i in range(10): robot.step(SIM_TIMESTEP)

    #start_pose = csci3302_lab5_supervisor.supervisor_get_robot_pose()
    start_pose = bb8_supervisor.supervisor_get_robot_pose()
    pose_x, pose_y, pose_theta = start_pose

    #dijkstra([3,5])
    # Main Control Loop:
    while robot.step(SIM_TIMESTEP) != -1:
        # Odometry update code -- do not modify
        if last_odometry_update_time is None:
            last_odometry_update_time = robot.getTime()
        time_elapsed = robot.getTime() - last_odometry_update_time
        update_odometry(left_wheel_direction, right_wheel_direction, time_elapsed)
        last_odometry_update_time = robot.getTime()

        # Get target location -- do not modify
        if target_pose is None:
            target_pose = csci3302_lab5_supervisor.supervisor_get_target_pose()
            world_map[transform_world_coord_to_map_coord(target_pose[:2])] = 3 # Goal vertex!
            print("New IK Goal Received! Target: %s" % str(target_pose))
            print("Current pose: [%5f, %5f, %5f]\t\t Target pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta, target_pose[0], target_pose[1], target_pose[2]))
            populate_map(world_map)
            display_map(world_map)

        


        if state == 'get_path':
            ###################
            # Part 2.1a
            ###################       
            # Compute a path from start to target_pose
            goalV = transform_world_coord_to_map_coord((target_pose[0], target_pose[1]))
            startV = transform_world_coord_to_map_coord((pose_x,pose_y))
            #print((1,8))
            #robot_pos = [robot_pos[0],robot_pos[1]]
            
            prev = dijkstra(startV)
            sol = reconstruct_path(prev, goalV)
            path_length = len(prev)
            display_map(world_map)
            counter = 1
            if startV in sol:
                visualize_path(sol)
                state = 'get_waypoint'
            pass
            
        elif state == 'get_waypoint':
            ###################
            # Part 2.1b
            ###################       
            # Get the next waypoint from the path
            #print(counter, path_length)
            #if counter == path_length:
            #    #TODO LAST VERTEX 
            #    #print("hmm")
            ##    continue;
            #way_point = prev[counter]
            #print(way_point)
            #goal_pose = transform_map_coord_world_coord(way_point)
            #if counter == path_length-1:
            #    bearing_error = target_pose[2]
            #else:
            #    next_wp = transform_map_coord_world_coord(prev[counter+1])
            #    bearing_error = math.atan2((next_wp[1] - goal_pose[1]), (next_wp[0] - goal_pose[0]) )
            #target_wp = [goal_pose[0],goal_pose[1],bearing_error]
            #print(bearing_error)
            #counter = counter + 1
            #state = 'move_to_waypoint'
            if(len(sol) != 0):
                x = sol.pop(0)
            if(sol == []):
                theta = target_pose[2]
                currentPoint = transform_map_coord_world_coord(x)
            else:
                nextPoint = transform_map_coord_world_coord(sol[0])
                currentPoint = transform_map_coord_world_coord(x)
                theta = math.atan2(nextPoint[1] - currentPoint[1], nextPoint[0] - currentPoint[0])
            newWaypoint = (currentPoint, theta)
            waypoints.append(newWaypoint)
            if waypoints == []:
                state = "get_path"
            else:
                state = "move_to_waypoint"
            pass
        elif state == 'move_to_waypoint':
            ###################
            # Part 2.1c
            ###################       

            # Hint: Use the IK control function to travel to the current waypoint
            # Syntax/Hint for using the IK Controller:
            lspeed, rspeed = get_wheel_speeds(target_wp)
            if lspeed == 0 and rspeed == 0:
                state = 'get_waypoint'
           
            leftMotor.setVelocity(lspeed)
            rightMotor.setVelocity(rspeed)
            
            #print(distance_error,heading_error)
            
            pass
        else:
            # Stop   
            pass
            
            
        #display_map(world_map)
        #print(world_map.shape[0])
    
if __name__ == "__main__":
    main()
