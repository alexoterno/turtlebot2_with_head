#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image




class Visual_Servoing():
    def __init__(self):
        # Init visual servoing node
        rospy.init_node('visual_servoing', anonymous=True)
        # Register handler to be called when rospy process begins shutdown
        rospy.on_shutdown(self.shutdown)
        # Intialize the movement command
        self.move_cmd = Twist()

        self.kinect = "Kinect camera"
        self.depth_kinect = "Depth Kinect camera"
        self.robotis = "Robotis OP3 Head camera"


        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=5)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback, self.kinect)
        # rospy.Subscriber('/camera/depth/image_raw', Image, self.callback, self.depth_kinect)
        rospy.Subscriber('/robotis_op3/camera/image_raw', Image, self.callback, self.robotis)

        rospy.loginfo("Marker messages detected. Starting visual servoing.")
        # Begin the cmd_vel publishing loop
        while not rospy.is_shutdown():
            #Send the Twist command to the robot
		    self.cmd_vel_pub.publish(self.move_cmd)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        twist = Twist()
        # twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)
        # rospy.sleep(1)

    def callback(self, data, msg):
        rospy.loginfo("I recieved an Image from " + msg)
        if msg == self.kinect:
            rospy.loginfo((data.data))


if __name__=="__main__":
    try:
		Visual_Servoing()
		rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Visual_Servoing node terminated.")
