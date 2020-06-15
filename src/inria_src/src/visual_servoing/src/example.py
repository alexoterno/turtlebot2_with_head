#!/usr/bin/env python

"""
    ar_follower.py - Version 1.0 2013-08-25

    Follow an AR tag published on the /ar_pose_marker topic.  The /ar_pose_marker topic
    is published by the ar_track_alvar package

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""
import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
#from move_base_msgs.msg import MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatusArray
#from move_base_msgs.msg import GoalStatus
from math import copysign
import time
import tf
from math import e # importing exp for control purposes

class ARFollower():
    #global counter
	def __init__(self):
		rospy.init_node("ar_follower")

		# Set the shutdown function (stop the robot)
		rospy.on_shutdown(self.shutdown)

		# How often should we update the robot's motion?
		self.rate = rospy.get_param("~rate", 15)
		r = rospy.Rate(self.rate)

		# The maximum rotation speed in radians per second
		self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.5)

		# The minimum rotation speed in radians per second
		self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.1)

		# The maximum distance a target can be from the robot for us to track
		self.max_x = rospy.get_param("~max_x", 20.0)

		# The goal distance (in meters) to keep between the robot and the marker
		self.goal_x = rospy.get_param("~goal_x", 0.6)#0.0001

		# How far away from the goal distance (in meters) before the robot reacts
		self.x_threshold = rospy.get_param("~x_threshold", 0.01)#0.01

		# How far away from being centered (y displacement) on the AR marker
		# before the robot reacts (units are meters)
		self.y_threshold = rospy.get_param("~y_threshold", 0.05) #0.05 0.2

		# How much do we weight the goal distance (x) when making a movement
		self.x_scale = rospy.get_param("~x_scale", 0.5)

		# How much do we weight y-displacement when making a movement
		self.y_scale = rospy.get_param("~y_scale", 0.9)

		# The max linear speed in meters per second
		self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.15)

		# The minimum linear speed in meters per second
		self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.05)

		#intialize frame counter
		self.TargetFlag = rospy.get_param("~TargetFlag", False)
		self.sm_markerFlag = rospy.get_param("~sm_markerFlag", False)
		self.MappingFlag = rospy.get_param("~MappingFlag",False)
		self.marker_id = rospy.get_param("~marker_id", 0)
		self.marker_id2 = rospy.get_param("~marker_id2", 0)
		self.rightFlag = rospy.get_param("~rightFlag", False)
		self.leftFlag = rospy.get_param("~leftFlag", False)
		self.AlignedZeroFlag = rospy.get_param("~AlignedZeroFlag", False)
		self.stopSearchingTargetLostFlag = rospy.get_param("~stopSearchingTargetLostFlag", False)
		self.stopspinning = rospy.get_param("~stopspinning", False)
		# Publisher to control the robot's movement
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5)

		# Intialize the movement command
		self.move_cmd = Twist()


		# Set flag to indicate when the AR marker is visible
		self.target_visible = False  #flag to show if the marker is in vision for the kinect or not
		self.TargetFlag	 = False #flag for indicating if the target have been spotted once or not at all
		self.MappingFlag = False # initial tag was aimed for recieving the flag from other groups! to initilize the visual servoing
		self.marker_id = 0  # initializing marker ID to zero ,, since we don't use tag number zero
		self.marker_id2 = 0 # initialize second marker id to zero.
		self.sm_markerFlag = False #flag to let the robot continue motion if target is lost
		self.rightFlag = False # flag to show if the robot comeing to the marker from the right side
		self.leftFlag = False # flag to show if the robot comeing to the marker from the left side
		self.AlignedZeroFlag = False #flag to show if the robot aligned 90 degree to the marker
		self.stopSearchingTargetLostFlag=False  #flag to stop the searching process when we lose the target on purpose for alignment
		self.stopspinning=False #similar to stopSearchingTargetLostFlag but for another part of the code,, we made two flags to keep it flexable
		rospy.loginfo("Waiting for ar_pose_marker topic...(notHassan....not hassan....Hassan is not here !)")
		#rospy.wait_for_message('move_base', String)
		#rospy.Subscriber('move_base', String, self.set_cmd_vel)
		rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
		#rospy.loginfo("waiting for move_base")
		#rospy.wait_for_message('move_base/status', GoalStatusArray)
		#rospy.loginfo("move_base message recieved")
		# Subscribe to the ar_pose_marker topic to get the image width and height
		rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.set_cmd_vel)
		#rospy.Subscriber('move_base', GoalStatusArray, self.set_cmd_vel)

		rospy.loginfo("Marker messages detected. Starting follower...but hassan didn't come yet.")

       	    # Begin the cmd_vel publishing loop
		while not rospy.is_shutdown():
            # Send the Twist command to the robot
			self.cmd_vel_pub.publish(self.move_cmd)

            # Sleep for 1/self.rate seconds
			r.sleep()
	#def initial_spin(self, msg):
    # TAG OF THE PREVIOUS TEAM
        # Pick off the first marker (in case there is more than one)
        #if (( self.target_visible == False) and (MappingFlag==True)):
        #self.move_cmd.angular.z = 1.1
        #if(self.target_visible == True)
        # self.move_cmd.angular.z =0
        #break\
	def set_cmd_vel(self,msg):
		global TargetFlag
		global MappingFlag
		global marker_id
		#Various flags, enabling disabling certain control statements and behaviours, depending on the state of the robot and the goal
		try:
		#Try and catch block loops (or activates) for each change of the msg.markers[0].
		#Therefore, it's close to an endless loop while the visual signal is coming in, and alvar is working.
		#To control this behaviour, but keep the loop working due to the fact that we want to access each frame and use its data
		#for servoing, multiple flags are introduced to stop the algorithm from reaching previous commands (instructions),
		#which are not longer necessary.
		#When the new state of the robot is reached e.g., when he got closer to the target, some flag will go up and prohibit him,
		#when the try and catch will be visited again, to perform searching for target, or, for example, rapid turning,
		#to avoid strong motions or unnecessary turning. So, at each step previous instructions are blocked from being visited by
		# the looping and new ones are introduced.

			marker = msg.markers[0] #save the marker into variable
			self.marker_id = msg.markers[0].id #get the first detected marker
			#self.marker_id2 = msg.markers[1].id
			rospy.loginfo("ZERO MARKER ID ZERO:%i\n",self.marker_id)
			#rospy.loginfo("ONE MARKER ID ONE:%i\n",self.marker_id2) #printing markers
		        #rospy.loginfo("The MARKER ID is:%i\n",marker_id) #printing markers


			if ((not self.target_visible) and (self.marker_id==5)):#If target is visible and the marker id is 5

				rospy.loginfo("FOLLOWER is Tracking Target!")#Output in terminal

				self.move_cmd.angular.z = 0.0 #Stop searching (spinning)
				self.target_visible = True #Change the state of the flag based on visibility of the tag. Now visible.
		except:
			# If target is lost, stop the robot by slowing it incrementally
			self.move_cmd.linear.x /= 1.4 #1.9
			if ((self.TargetFlag is not True) and (self.marker_id !=5) and (self.stopSearchingTargetLostFlag==False)):
				self.move_cmd.angular.z = 0.6 #Spinning for the purpose of searching, possible values : 1.08, 0.9
			elif(self.sm_markerFlag == False):# Complete linear stop
				self.move_cmd.linear.y = 0.0
				self.move_cmd.linear.z = 0.0
				self.move_cmd.linear.x = 0.0
				#self.move_cmd.angular.z= 0.0

			if self.target_visible:# If target is lost
				rospy.loginfo("FOLLOWER LOST Target!")
				self.target_visible = False

			return


        #rospy.loginfo("the frame count is : %f", counter)

		#if (msg.markers[1] is not None)
			#self.marker_id2 = msg.markers[1].id
			#rospy.loginfo("ONE MARKER ID ONE:%i\n",self.marker_id2)
		quaternion = (
						marker.pose.pose.orientation.x,
						marker.pose.pose.orientation.y,
						marker.pose.pose.orientation.z,
						marker.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2] # these are the quaternions transormed to euler angles -  we only need the yaw, to make the robot perpindecular with the surface of the marker


		#rospy.loginfo("the Roll is: %f",roll)
		#rospy.loginfo("the Pitch is: %f",pitch)
		rospy.loginfo("the yaw is: %f",yaw)#Output yaw value



		# Get the displacement of the marker relative to the base
		target_offset_y = marker.pose.pose.position.y

		# Get the distance of the marker from the base
		target_offset_x = marker.pose.pose.position.x

		# Get the distance of the marker from the base
		target_offset_z = marker.pose.pose.position.z

		rospy.loginfo("the target_offset_x is: %f",target_offset_x)# Output the value of distance to the tag
		#rospy.loginfo("the target_offset_y is: %f",target_offset_y)
		#rospy.loginfo("the target_offset_z is: %f",target_offset_z)

		if (self.marker_id==5):
			self.TargetFlag = True
			# Rotate the robot only if the displacement of the target exceeds the threshold
			if ((abs(target_offset_y) > (self.y_threshold)) and (self.sm_markerFlag == False) and (self.AlignedZeroFlag==False)):  #and (self.TargetFlag is not True)
			    # Set the rotation speed proportional to the displacement of the target
				speed = (target_offset_y * self.y_scale)
				self.move_cmd.angular.z = copysign(max(self.min_angular_speed,
				                        min(self.max_angular_speed, abs(speed))), speed)
				rospy.loginfo("TURNING Y")
			elif((self.sm_markerFlag == False) and (self.AlignedZeroFlag==False)):
				self.move_cmd.angular.z = 0.0
				rospy.loginfo("STOP ZED")
			    #self.counter = 121;

			# Now get the linear speed
			if ((target_offset_x - self.goal_x) > self.x_threshold and self.sm_markerFlag == False):
				speed =  abs((self.goal_x - (target_offset_x))) * self.x_scale #target_offset_x +0.2 work on slowing the general speed
				if speed > 0:
					speed = 0.3#1.5 #giving constant speed for better control of the robot
				self.move_cmd.linear.x = copysign(min(self.max_linear_speed, max(self.min_linear_speed, abs(speed))), speed)
				rospy.loginfo("NO ELSE AT ALL")
			else:
				self.move_cmd.linear.x /= 2 #1.9
				cspeed =  self.move_cmd.linear.x # we save the current speed here
				rospy.loginfo("WEEEE GOOOOOT HEEEREEEEEEEE!!!!  (hassan not)")
				rospy.loginfo("the current speed is : %f",cspeed)
				if((target_offset_x <= 0.8 ) and (target_offset_x >= 0.4) and (cspeed <= 0.1)):#0.8 0.05 0.1, respectively
					self.move_cmd.linear.x = cspeed*e**(0.43)-cspeed*e**(cspeed)+0.08 # 0.4 worked cspeed*0.1+0.16
					#this if statement is to compensate the variant difference of distance between the robot and the marker
					#in other words this if statement is responsible for the control part
					#model is linear modeled as v*e^c-v*e^(v)+c2

					rospy.loginfo("increasing Speed \n")
					self.sm_markerFlag=True
				elif(self.sm_markerFlag==True):
					self.move_cmd.linear.x /= 1.5 #0.85 #Slowing down the robot
				# self.move_cmd.linear.y = 0.0
				# self.move_cmd.linear.z = 0.0

					rospy.loginfo("arrived to destination : %r", self.TargetFlag)
					rospy.loginfo("target achieved\n")

					#the following 3 if statement are for aligning the robot 90 degree to the marker
					#the first if statement checks if the robot is coming from the right side of the marker
					#the second if statement checks if the robot is looking 90 degree to the marker
					#the third if statement checks if the robot is coming from the left side of the marker

					if(yaw>0.8 and yaw<1.7 and self.AlignedZeroFlag==False): #from 0.8 to 1.4
						#right blind alignment
						self.move_cmd.angular.z = -0.2
						rospy.loginfo("target aligning -- Z turning right\n")
					       	self.rightFlag=True #setting the right flag to TRUE

					elif(yaw>-1.6 and yaw <=-1.4 and self.AlignedZeroFlag==False):# from -1.7 to -1.4
						#center
						self.move_cmd.angular.z = 0.0
						rospy.loginfo("target aligned\n")
						csOFZ=self.move_cmd.angular.z #current angular speed
						self.AlignedZeroFlag=True #robot is aligned

					elif(yaw<-0.8 and yaw > -1.7 and self.AlignedZeroFlag==False):# from -0.8 to -1.4
						#left alignment
						self.move_cmd.angular.z = 0.2
						rospy.loginfo("target aligning ++ Z turning left \n")
						self.leftFlag=True #robot is coming from left, flag is set to true


		if (self.rightFlag==True and self.AlignedZeroFlag==True):#start of blind parking when the robot comes from the right side
			rospy.loginfo("WE CAME FROM THE RIGHT and (Hassan didn't come from home)\n")

		elif(self.leftFlag==True and self.AlignedZeroFlag==True): #start of blind parking when the robot comes from the left side
			rospy.loginfo("WE CAME FROM THE LEFT\n")
			self.stopSearchingTargetLostFlag=True
			if ((self.stopspinning==False) and (target_offset_x<=1.9)):#Based on distance to the tag estimated by alvar.
				self.move_cmd.angular.z = -0.35
			if (self.marker_id !=5):#((self.target_visible == False) and (self.marker_id !=5)):
				self.stopspinning=True
				self.move_cmd.angular.z /= 1.05 #lowering the angular speed
				rospy.loginfo("I LOST THIS TARGET ON PURPOSE  and (hassan doesn't know why)!:3\n") #robot lost target on purpose for blind parking (see readme, centering on the center of the tag).



     					#if yaw > 1.5 and yaw < 3:
                 			#	self.move_cmd.angular.z = 0.2

                 			#	self.move_cmd.linear.x = -0.01
                 			#	rospy.loginfo("Parking\n")
		#elif((self.TargetFlag is not True) and ((self.marker_id != 5 ) or (self.marker_id2!= 5 ))  and ((self.marker_id2 != 12) or (self.marker_id != 12))):
		elif((self.TargetFlag is not True) and (self.marker_id != 5 ) and (self.stopSearchingTargetLostFlag==False) and (self.stopspinning==False)):
			self.move_cmd.angular.z = 0.5 #Enable spinning for the search if lost. Possible values: 1.08 #0.6




	def shutdown(self):
		rospy.loginfo("Stopping the robot...")
		self.cmd_vel_pub.publish(Twist())
		rospy.sleep(1)

if __name__ == '__main__':

	try:

		ARFollower()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("AR follower node terminated.")
