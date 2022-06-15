#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math




from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import cv2 as cv
import numpy as np
import roslib


from cv_bridge import CvBridge, CvBridgeError

import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from rospy.exceptions import ROSInterruptException
import tf2_ros
from tf2_ros import transform_broadcaster
import tf_conversions

aruco_detected = 0
velocity_msg = Twist()
global var 
var =0
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

class Ur5Moveit:

    # Constructor
    def __init__(self):

        
        
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
            self.go_to_pose(arg_pose)
        return flag_plan
    
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')
            self.set_joint_angles(arg_list_joint_angles)

        return flag_plan

    def get_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()
        return list_joint_values


    def gripper_open(self): #function to open the gripper
        gripper_group= moveit_commander.MoveGroupCommander('gripper_controller')
        gripper_group.set_named_target("open")
        plan2= gripper_group.go()

    def gripper_close(self): #function to close the gripper
        gripper_group= moveit_commander.MoveGroupCommander('gripper_controller')
        gripper_group.set_named_target("close")
        plan2= gripper_group.go()

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def aruco_cam(data):


    # Initializing variables
    global cv_image
    focal_length = 476.70308
    center_x = 400.5
    center_y = 400.5
    aruco_dimension = 0.1
    
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

        # load the dictionary that was used to generate the markers
        dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_7X7_1000)

        # initializing the detector parameters with default values
        parameters =  cv.aruco.DetectorParameters_create()

        # detect the markers in the frame
        corners, ids, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)
        
        if len(corners) > 0:
            # Flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
    
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # draw the bounding box of the ArUCo detection
                cv.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv.circle(frame, (cX, cY),20, (0, 0, 255), -1)
                # print(frame.shape[1])
                # print (cX)
                pixel_width = topLeft[1] - bottomRight[1]

                # draw the ArUco marker ID on the frame
                cv.putText(frame, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                
                '''uncomment to view aruco ID and verify the working of the code'''
                #print("[INFO] ArUco marker ID: {}".format(markerID))

                # obtain depth for each ArUco marker
                distance = (focal_length*aruco_dimension)/pixel_width

                # transforming pixel coordinates to world coordinates
                world_x = (cX - center_x)/focal_length*distance
                world_y = (cY - center_y)/focal_length*distance
                world_z = distance

                # broadcasting TF for each aruco marker
                br = tf2_ros.TransformBroadcaster()
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "sjcam_link"
                t.child_frame_id = "aruco"+str(markerID)

                # putting world coordinates coordinates as viewed for sjcam frame
                t.transform.translation.x = world_z
                t.transform.translation.y = -world_x
                t.transform.translation.z = world_y
                # not extracting any orientation thus orientation is (0, 0, 0)
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                br.sendTransform(t)
                #print (cX)
                if (cX <370 and cX>300):
                    aruco_detected = 1
                    global var
                    var = 1
                else:
                    var = 0
                    aruco_detected = 0

        '''uncoment to view the visual of detection'''
        cv.imshow("frame", frame)
        cv.waitKey(1)
    except CvBridgeError as e:
        print(e)


def PID(target,current):
    global error,PID_value,P,I,D,previous_I,previous_error
    Kp = 20
    Kd = 40
    Ki = 20
    PID_value_upper = 0.8
    PID_value_lower = -0.8
    error = target - current 
    P = error
    I = I + previous_I
    D = error - previous_error
    PID_value = (Kp * P) + (Ki * I) + (Kd * D)
    if PID_value > PID_value_upper:
        PID_value = 0.8
    elif PID_value < PID_value_lower:
        PID_value = -0.8    
    previous_I = I
    previous_error = error




def odom_callback(data):
    global pose
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y,
            euler_from_quaternion([x, y, z, w])[2]]

def laser_callback(msg):
    global regions
    regions = {
        'right': min(min(msg.ranges[0:143]), msg.range_max),
        'fright': min(min(msg.ranges[144:287]), msg.range_max),
        'front': min(min(msg.ranges[288:431]), msg.range_max),
        'fleft': min(min(msg.ranges[432:575]), msg.range_max),
        'left': min(min(msg.ranges[576:719]), msg.range_max),
    }


def control_loop():


    rate = rospy.Rate(10)

    x_vel = 0  # stores the value of linear velocity of the bot
    ang_vel = 0  # stores the value of angular velocity of the bot
    velocity_msg = Twist()
    velocity_msg.linear.x = x_vel
    velocity_msg.angular.z = ang_vel
    pub.publish(velocity_msg)

    state = 0
    flag = 0  
    while not rospy.is_shutdown() and flag!=1 :
        print("state= ",state)
        print("var = ",var)
        velocity_msg.linear.x = x_vel
        velocity_msg.angular.z = ang_vel
        pub.publish(velocity_msg)
        if (var==1):
            if (state ==3 or state ==1):
                x_vel=0
                ang_vel =0
                pub.publish(velocity_msg)
                contour_check()
            
        if(x_vel == 0):
            if(ang_vel == 0):
                if (state ==3 or state ==1):
                    rospy.sleep(1)
                    velocity_msg.linear.x = 0.3
                    velocity_msg.angular.z = 0
                    pub.publish(velocity_msg)
                    rospy.sleep(1)
                print("STOP")
        if (x_vel==1):
            rospy.sleep(1)
            state=-2

        if state ==-1:
            x_vel=1
            print("started run")

        if state == 0: # Clockwise turn and go near rightmost lane
            if not(pose[2] < 0.4 and pose[2] > 0.3):

                ang_vel = -0.3
                x_vel = 0

            else:
                ang_vel = 0
                x_vel = 0.3
                if(regions['left'] < 0.8):
                    state = 1
        
        elif state == 1:
            PID(0.65,regions['left'])
            ang_vel = -PID_value
            x_vel = 0.3
            if (pose[1]< 0):
                if(pose[2] < -0.5):
                    ang_vel = 0
                    x_vel = 0
                    state = 2
            
        
        elif state == 2:
            if not(pose[1] < -1.25):
                x_vel = 0.4
                ang_vel = 0
            else :
                if (pose[2] < 1.6):
                    ang_vel = 0.3
                    x_vel = 0
                else:
                    x_vel = 0
                    ang_vel = 0
                    state = 2.5
        
        elif state == 2.5:
            if not(regions['left']<0.7):
                x_vel = 0.3
                ang_vel = -0.05
            else:
                state = 3
        
        elif state == 3:            
            PID(0.65,regions['left'])
            ang_vel = -PID_value
            x_vel = 0.3
            if (pose[1]< -0.8):
                if(pose[2] < -0.5):
                    ang_vel = 0
                    x_vel = 0
                    state = 4
        
        elif state == 4:
            if not(pose[1] < -1.25):
                x_vel = 0.4
                ang_vel = 0
            elif not(pose[2]<0.1 and pose[2]>-0.06):
                x_vel = 0
                ang_vel = 0.3

            elif not(pose[0]<0.85 and pose[0]>0.75):
                x_vel = 0.3
                ang_vel = 0
            
            else:
                state = 4.5

        elif state == 4.5:
            if (pose[2] < 1.6):
                    ang_vel = 0.3
                    x_vel = 0
            else:
                x_vel = 0
                ang_vel = 0
                state = 5        
            
        elif state == 5:
            print("Mission Accomplished!")
            flag = 1

        rate.sleep()

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
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('ebot_base', 'obj_'+str(i), rospy.Time.now(), timeout=rospy.Duration(3.0))
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return 0

def task():
    file = open("object_info.txt", "r")
    # ur5 = Ur5Moveit()
    x= file.readlines()
    print(x)
    print("execute pick and place")


def world(x,y,depth):

    cx = 320.5
    cy = 240.5
    fx =554.387
    fy = 554.387
    world_x = depth * ((x-cx)/fx)
    world_y = depth * ((y-cy)/fy)
    world_z = depth

    return world_x,world_y, world_z

def color_cam(colour):
    global cv_image,frame_rgb
    try:
        bridge = CvBridge()
        frame_rgb = bridge.imgmsg_to_cv2(colour, "bgr8") #converting data file to bgr8 colour space
        hsv_frame = cv.cvtColor(frame_rgb,cv.COLOR_BGR2HSV) # converting to HSV frame to do thresholding operation
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
        global cnts
        cnts = cv.findContours(blurred, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]


    except CvBridgeError as e:
        print(e)

def depth_cam(ros_image):
    bridge = CvBridge()
    # print("cehck depth")
    try:
        depth_image = bridge.imgmsg_to_cv2(ros_image,desired_encoding="passthrough")

        global depth_array
        depth_array = np.array(depth_image, dtype= np.float32)
    except CvBridgeError as e:
        print(e)

def contour_check():
    i=-1
    translate =[]

    if len(cnts) == 0:
        print("No contour")
        return 0

    else:
        print("contour exists")
        file = open("object_info.txt","w+")
        for c in cnts:
                
                try:
                    # print(i)
                    M = cv.moments(c)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"]) 

                    # draw the contour and center of the shape on the image
                    if cv.contourArea(c) > 140:
                        i +=1
                        cv.drawContours(frame_rgb, [c], -1, (0, 255, 0), thickness=1)
                        cv.circle(frame_rgb, (cX, cY), 1, (255, 255, 255), -1)
                        cv.putText(frame_rgb, "obj_"+str(i), (cX - 20, cY - 20),cv.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                        world_x,world_y,world_z = world(cX,cY, depth_array[cY][cX])
                        trans = tf_publish(world_x,world_y,world_z,i)
                        translate.append(str(trans.transform.translation.x) + '\n')
                        translate.append(str(trans.transform.translation.y) + '\n')
                        translate.append(str(trans.transform.translation.z) + '\n')
                        #translate_str = [str(trans.transform.translation.x) + '\n', str(trans.transform.translation.y) + '\n', str(trans.transform.translation.z) + '\n']
                        #file.writelines(translate_str)
                        #a=file.readlines()
                        print(translate,len(translate))
                        #print(a,len(a))
                        if (len(translate)==3*(len(cnts))):
                            flag=1
                            file.writelines(translate)
                            file.close()
                            task()
                            break
                        # # coordinates.append(trans)
                        
                except ZeroDivisionError:
                    pass
        return 0


if __name__ == '__main__':
    try:
        # initialization of global variables pose and regions
        global error, P, I, D, previous_error, previous_I, PID_value
        PID_value = 0 
        error = 0
        P = 0
        I = 0
        D = 0
        previous_error = 0
        previous_I = 0
        pose = [0, 0, 0]
        regions = {'right': 0,
                   'fright': 0,
                   'front': 0,
                   'fleft': 0,
                   'left': 0}
        rospy.init_node('ebot_controller')
        ur5  = Ur5Moveit()
        
        global start  
        start = [math.radians(98),
                          math.radians(-21),
                          math.radians(-39),
                          math.radians(58),
                          math.radians(0),
                          math.radians(0)]
        ur5.set_joint_angles(start) 
        rospy.sleep(1)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
        rospy.Subscriber('/odom', Odometry, odom_callback)
        rospy.Subscriber('/odom', Odometry, odom_callback)
        rospy.Subscriber("/ebot/camera1/image_raw", Image, callback  = aruco_cam)
        global rate,tfBuffer
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        
        rate = rospy.Rate(10.0)
        image_sub = rospy.Subscriber('/camera/color/image_raw2', Image , callback=color_cam) 
        depth_sub = rospy.Subscriber('/camera/depth/image_raw2', Image , callback=depth_cam)
        control_loop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass