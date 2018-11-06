#!/usr/bin/env python

"""
movement.py

"""
import math

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Twist
from nav_msgs.msg import Odometry
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



def _forward(pub):
    """Move turtlebot forward.

    """
    move_msg = Twist()
    move_msg.linear.x = 0.4

    dist = 0
    t0 = rospy.get_time()

    while dist < 1.:
        pub.publish(move_msg)
        t1 = rospy.get_time()
        dist = (t1-t0)*.4


def _forward_ac(ac):
    """Move turtlebot forward.

    """
    # goal.target_pose is a PoseStamped msg
    # frame_id will be 'base_link' b/c we are moving relative to robot
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_link'

    target = {'x': 1., 'y': 0., 'z': 0.}
    q = {'x': 0., 'y': 0., 'z': 0., 'w': 1.}

    goal.target_pose.header.stamp = rospy.get_rostime()
    goal.target_pose.pose = Pose(Point(**target), Quaternion(**q))
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(10))


def _rotate(pub, degrees, direction):
    """Rotates turtlebot.
    
    Parameters
    ----------
    pub : rospy.Publisher
    degrees : int
    direction : str
    """
    move_msg = Twist()
    angle = 0
    t0 = rospy.get_time()

    if direction == 'clockwise':
        move_msg.angular.z = -.4
        rad = -math.pi / (180./degrees)
        while angle > rad:
            pub.publish(move_msg)
            t1 = rospy.get_time()
            angle = (t1-t0)*(-.4)
    else:
        move_msg.angular.z = .4
        rad = math.pi / (180./degrees)
        while angle < rad:
            pub.publish(move_msg)
            t1 = rospy.get_time()
            angle = (t1-t0)*.4

    # stop turtlebot from rotating due to momentum
    move_msg.angular.z = 0
    pub.publish(move_msg)


def process_kw(data, args):
    """
    """
    kw = data.data.strip().lower() 
    pub = args[0]
    ac = args[1]

    keyword_actions = {
        'forward': lambda: _forward(pub),
        #'forward': lambda: _forward_ac(ac),
        'right turn': lambda: _rotate(pub, 90., 'clockwise'),
        'left turn': lambda: _rotate(pub, 90., 'counter clockwise'),
        'back': lambda: _rotate(pub, 90., 'counter clockwise'),
    }

    # TODO: We need to ask for confirmation
    # if it cannot be confirmed, then just return from this function
    if kw in keyword_actions:
        keyword_actions[kw]()


def main():

    rospy.init_node('movement', anonymous=False)

    # create both a publisher and action client ... not sure which one we
    # are going to use yet
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    ac.wait_for_server(rospy.Duration(5))

    rospy.Subscriber('/pocketsphinx_recognizer/output', String, process_kw,
                     callback_args=(pub, ac))
    
    while not rospy.is_shutdown():
        rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
