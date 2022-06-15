#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import Image
from rospy.exceptions import ROSInterruptException
from cv_bridge import CvBridge , CvBridgeError
import cv2 as cv
import numpy as np
import tf2_ros
from tf2_ros import transform_broadcaster
import tf_conversions



cv_image = None


def to_point(array):

  return Point(*array)

def from_vector3(msg):
  """
  Converts a C{geometry_msgs/Vector3} ROS message into a numpy array.
  @type  msg: geometry_msgs/Vector3
  @param msg: The ROS message to be converted
  @rtype: np.array
  @return: The resulting numpy array
  """
  return np.array([msg.x, msg.y, msg.z])
#Class Moveit 
class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

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

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

#Class Moveit Ends here

#Compute World Coorindates
def world(x,y,depth):

    cx = 320.5
    cy = 240.5
    fx =554.387
    fy = 554.387
    world_x = depth * ((x-cx)/fx)
    world_y = depth * ((y-cy)/fy)
    world_z = depth

    return world_x,world_y, world_z 

#Publish the transforms in rviz
def tf_publish(x,y,z,i):
    # broadcasting TF for each aruco marker
    global t
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "camera_link2"
    t.child_frame_id = "obj_"+str(i)

    # putting world coordinates coordinates as viewed for camera frame
    t.transform.translation.x = z
    t.transform.translation.y = -x
    t.transform.translation.z = -y
    # not extracting any orientation thus orientation is (0, 0, 0)
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)
    


   
    # TF publish


    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('ebot_base', 'obj_'+str(i), rospy.Time.now(), timeout=rospy.Duration(5.0))
            array=from_vector3(trans)
            final=to_point(array)
            print(type(final))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return 0

 


# Callback function for RGB camera
def color(colour):

    global cv_image


    try:
        
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(colour, "bgr8") #converting data file to bgr8 colour space
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

        # Finding Contours
        cnts = cv.findContours(blurred, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        i = -1
        tr = geometry_msgs.msg.TransformStamped()
        # print("checking")
        coordinates =[]
        transl = None
        for c in cnts:
            
            try:
                # print(i)
                M = cv.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"]) 

                # draw the contour and center of the shape on the image
                if cv.contourArea(c) > 80:
                    i +=1
                    cv.drawContours(frame, [c], -1, (0, 255, 0), thickness=1)
                    cv.circle(frame, (cX, cY), 1, (255, 255, 255), -1)
                    cv.putText(frame, "object", (cX - 20, cY - 20),
                cv.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                    world_x,world_y,world_z = world(cX,cY, depth_array[cY][cX])
                    transl = tf_publish(world_x,world_y,world_z,i)
                    # trans = tf_listen(i)
                    # # coordinates.append(trans)
                    print(transl.transform)
            except ZeroDivisionError:
                pass 

        cv.imshow("camera" , frame)
        cv.waitKey(1)

    except CvBridgeError as e:
        print(e)


#Callback function for Depth Camera
def depth(ros_image):
    bridge = CvBridge()
    # print("cehck depth")
    try:
        depth_image = bridge.imgmsg_to_cv2(ros_image,desired_encoding="passthrough")

        global depth_array
        depth_array = np.array(depth_image, dtype= np.float32)
    except CvBridgeError as e:
        print(e)



if __name__ == '__main__':

    try:
        global depth_array
        depth_array = np.zeros([480,640])
        # ur5=Ur5Moveit()
        rospy.init_node("tomato", anonymous=True)
        image_sub = rospy.Subscriber('/camera/color/image_raw2', Image , callback=color) 
        depth_sub = rospy.Subscriber('/camera/depth/image_raw2', Image , callback=depth)
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(10.0)

        rospy.spin()
    except ROSInterruptException :
        pass