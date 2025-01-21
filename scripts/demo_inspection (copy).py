#!/usr/bin/env python3

from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math 
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()


    
 # Define the waypoints for the square
    pre_thumper = [

		# Left Grid (back-and-forth pattern)
		#[-0.12, -1.75, (math.pi/2)],
		
		#Get out of circle
		[2.5, -1.42, (math.pi/2)],
		
		[2.75, -0.59, math.pi],
		[-0.15, -0.52, -(math.pi/2)],
		
		[-0.15, -0.80, 0.0],
		[2.65, -0.75, -(math.pi/2)],
		
		[2.65, -1.15, math.pi],
		[-0.15, -1.0, -(math.pi/2)],

		[2.75, -0.6, (math.pi/2)],
		
		[2.75, 0.0, math.pi],
		[-0.20, 0.0, 0.0],
    ]

    post_thumper = [
    		#Getting to top
       	        [2.70, 0.0, -0.707, 0.707],
       	        [2.35, -1.45, 0.9239, -0.3827],
       	        
       	        #Entry point
        	[1.15, -1.60, -0.707, 0.707],
        	
        	#First Line 
        	[1.13, -2.60, -1.0, 0.0],
        	[0.96, -2.60, 0.707, 0.707],
        	
        	#Second line
        	[1.20, -1.38, -0.3827, 0.9239],
        	
        	#This will be fixed
        	#[0.90, -2.12, -0.707, 0.707],
        	#[1.05, -2.55, -1.0, 0.0],
        	#[0.98, -2.48, 0.0, 1.0], 
        	#[1.62, -2.45, -0.707, 0.707],
        	
        	#Get out point
        	#[1.38, -1.55, 0.707, 0.707],
        	
        	]
        	  	
    
    
    
    # Set the initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Send the route
    pre_thumper_points = []
    post_thumper_points = []
    inspection_pose = PoseStamped()
    inspection_pose.header.frame_id = 'map'
    inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
    for pt in pre_thumper:
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        # Convert yaw to quaternion
        inspection_pose.pose.orientation.z = math.sin(pt[2] / 2)
        inspection_pose.pose.orientation.w = math.cos(pt[2] / 2)
        pre_thumper_points.append(deepcopy(inspection_pose))

    for pt in post_thumper:
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        inspection_pose.pose.orientation.z = pt[2]
        inspection_pose.pose.orientation.w = pt[3]
        post_thumper_points.append(deepcopy(inspection_pose))
        

    navigator.followWaypoints(post_thumper_points)


    # Print feedback during navigation
    i = 0
    
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(post_thumper))
            )

    navigator.cancelTask()

    navigator.backup(backup_dist=0.55, backup_speed=0.25)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'Distance traveled: {feedback.distance_traveled}')
    
    navigator.cancelTask()
    
    navigator.followWaypoints(pre_thumper_points)

    # Print feedback during navigation
    i = 0
    
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:

            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(pre_thumper))
            )


    # Print the result of the task
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Square path completed! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Task was canceled. Returning to start...')
    elif result == TaskResult.FAILED:
        print('Task failed! Returning to start...')

    # Return to the initial pose
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()

