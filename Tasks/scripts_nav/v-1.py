#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def odom_callback(data):
    global pose
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y,
            euler_from_quaternion([x, y, z, w])[2]]

    # *odom_callback*(user-defined) simply stores the [x, y, theta] or the pose
    # (position + heading) of the robot in the
    # global variable pose. But if participants are curious,
    # notice how the euler_from_quaternion function is used to get the
    # quaternion's heading(Euler angle).


def control_loop():
    rospy.init_node('ebot_controller')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10)
    x_vel = 0.3  # stores the value of linear velocity of the bot
    ang_vel = 0  # stores the value of angular velocity of the bot
    velocity_msg = Twist()
    velocity_msg.linear.x = x_vel
    velocity_msg.angular.z = ang_vel
    pub.publish(velocity_msg)

    state = 0   # stores the value
    # of different sequences
    # of the path to be taken by the bot
    while not rospy.is_shutdown():
        velocity_msg.linear.x = x_vel
        velocity_msg.angular.z = ang_vel
        pub.publish(velocity_msg)
# makes the bot turn 90 degrees to the left and move forward
        if state == 0:
            if not(pose[2] < 3 and pose[2] > 2.95):

                ang_vel = 0.5
                x_vel = 0

            else:
                ang_vel = 0
                x_vel = 0.5
                # checks the presence of an obstacle to the right side of the
                # bot
                if(regions['right'] > 0.7 and regions['right'] < 0.8):
                    state = 1
# ensures the bot stays at the same distance from the obstacle column to the
# right
# checks if an obstacle is in close proximity to the left of the bot

        elif state == 1:
            if regions['left'] < 1:
                state = 2
                # staying to the left side obstacle is priorotized
            else:
                # checks if the bot is at a given distance range from the
                # obstacle
                if(regions['right'] < 0.8 and regions['right'] > 0.7):
                    ang_vel = 0
                    x_vel = 0.5
                # condition to check if the bot has to turn right if the
                # distance from the obstacle increases
                elif(regions['right'] > 0.8):
                    ang_vel = -1.2
                    x_vel = 0.2
                else:  # turns left if the bot gets close to the obstacle
                    ang_vel = 1.2
                    x_vel = 0.2
# ensures the bot stays at the same distance from the obstacle column to the
# left
# checks if the bot has reached the top right corner of the
# # simulation world (stopping point)
        elif state == 2:
            if(pose[0] > 2.4 and pose[1] > 7.5):
                state = 3
                x_vel = 0
                ang_vel = 0
            else:
                # condition to check when the bot has to move straight
                if(regions['left'] < 0.7 and regions['left'] > 0.6):
                    ang_vel = 0
                    x_vel = 0.5
                # checks if the bot has to turn left
                elif(regions['left'] > 0.7):
                    ang_vel = 1.2
                    x_vel = 0.2
                else:  # checks if the bot has to turn right
                    ang_vel = -1.2
                    x_vel = 0.2

        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()


def laser_callback(msg):
    global regions
    regions = {
        'right': min(min(msg.ranges[0:143]), msg.range_max),
        'fright': min(min(msg.ranges[144:287]), msg.range_max),
        'front': min(min(msg.ranges[288:431]), msg.range_max),
        'fleft': min(min(msg.ranges[432:575]), msg.range_max),
        'left': min(min(msg.ranges[576:719]), msg.range_max),
    }


if __name__ == '__main__':
    try:
        # initialization of global variables pose and regions
        pose = [0, 0, 0]
        regions = {'right': 0,
                   'fright': 0,
                   'front': 0,
                   'fleft': 0,
                   'left': 0}
        control_loop()
    except rospy.ROSInterruptException:
        pass
