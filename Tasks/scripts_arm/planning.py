#!/usr/bin/env python3

import rospy
import sys
# import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
# import math
import cv2 as cv
import numpy as np
from rospy.exceptions import ROSInterruptException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import tf2_ros
# import tf_conversions

cv_image = None

#Moveit Class
class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('tomato_tf', anonymous=True)

        self._planning_group = "arm_controller"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan


    def gripper_open(self): #function to open the gripper
        gripper_group= moveit_commander.MoveGroupCommander('gripper_controller')
        gripper_group.set_named_target("open")
        plan2= gripper_group.go()

    def gripper_close(self): #function to close the gripper
        gripper_group= moveit_commander.MoveGroupCommander('gripper_controller')
        gripper_group.set_named_target("close")
        plan2= gripper_group.go()

    def move_2_point(self):
        global world_x,world_y,world_z
        ur5_pose = geometry_msgs.msg.Pose()

        ur5_pose.position.x = world_z
        ur5_pose.position.y = -world_x
        ur5_pose.position.z = -world_y
        ur5_pose.orientation.x = 0
        ur5_pose.orientation.y = 0
        ur5_pose.orientation.z = 0
        ur5_pose.orientation.w = 0
        while not rospy.is_shutdown(): 
            print("check")
            ur5.go_to_pose(ur5_pose)
            rospy.sleep(2)

    def move_2_basket(self):
        ur5_pose = geometry_msgs.msg.Pose()

        ur5_pose.position.x = 0
        ur5_pose.position.y = 0
        ur5_pose.position.z = 0
        ur5_pose.orientation.x = 0
        ur5_pose.orientation.y = 0
        ur5_pose.orientation.z = 0
        ur5_pose.orientation.w = 0
        while not rospy.is_shutdown(): 
            ur5.go_to_pose(ur5_pose)
            rospy.sleep(2)

    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()

    # Destructor
    def del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
    
    

#----Moveit Class Ends here -----

#---Finding world coordinates--

def tf_pub(x,y,d,i):
    global world_x,world_y,world_z
    cx = 320.5
    cy = 240.5
    fx = 554.387
    fy = 554.387
    # print(x,y,d)
    world_x = d * ((x-cx)/fx)
    world_y = d * ((y-cy)/fy)
    world_z = d


#---Depth camera processing--

def depth(ros_image):
    bridge = CvBridge()
    try:
        depth_image = bridge.imgmsg_to_cv2(ros_image,"passthrough")        
        depth_array= np.array(depth_image, dtype = np.float32)

    except CvBridgeError as e:
        print(e)



def callback(data):
    global cv_image

    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8") #converting data file to bgr8 colour space
        hsv_frame = cv.cvtColor(frame,cv.COLOR_BGR2HSV) # converting to HSV frame to do thresholding operation
        hsv=hsv_frame.copy()

        #Setting red colour limits in HSV space
        lower1 = np.array([0, 100, 20])
        upper1 = np.array([4, 255, 255])

        lower2 = np.array([170, 100, 20])
        upper2 = np.array([179, 255, 255])

        lower_mask = cv.inRange(hsv_frame, lower1, upper1)
        upper_mask = cv.inRange(hsv_frame, lower2, upper2)
 
        full_mask = lower_mask + upper_mask
 
        hsv = cv.bitwise_and(hsv, hsv, mask=full_mask)
        h,s,gray = cv.split(hsv)


        blurred = cv.GaussianBlur(gray, (5, 5), 0)  


        cnts = cv.findContours(blurred, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        # #drawing contours and dot inside the contour 
        i = -1
        for c in cnts:
            try:
                position=[]
                M = cv.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"]) 
                
                # draw the contour and center of the shape on the image
                if cv.contourArea(c) > 50:
                    cv.drawContours(frame, [c], -1, (0, 255, 0), thickness=1)
                    cv.circle(frame, (cX, cY), 1, (255, 255, 255), -1)
                    cv.putText(frame, "object", (cX - 20, cY - 20),
                    cv.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

                    tf_pub(cX,cY,depth_array[cY][cX],i)                   
                    position.append([world_x,world_y,world_z]) 
                    ur5.gripper_open()
                    ur5.move_2_point()
                    ur5.gripper_close()
                    ur5.move_2_basket()
                    # while not rospy.is_shutdown(): 
                    #     ur5.go_to_pose(ur5_pose)
                    #     rospy.sleep(2)
                    ur5.gripper_open()
                    rospy.sleep(2)
                    
            except ZeroDivisionError:
             pass
            
        cv.imshow("camera",frame)
        cv.waitKey(1)
    except CvBridgeError as e:
        print(e)
 
 #--- Callback functions 



def main(args):
    global position, world_x,world_y,world_z,ur5
    ur5 = Ur5Moveit()
    global depth_array
    depth_array = np.zeros([480,640])

    depth_sub = rospy.Subscriber("camera/depth/image_raw2", Image, callback=depth)
    image_sub = rospy.Subscriber("camera/color/image_raw2", Image, callback)
    
    try:
        rospy.spin()
    except ROSInterruptException:
        pass
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)