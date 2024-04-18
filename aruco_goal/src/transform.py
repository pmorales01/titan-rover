#!/usr/bin/env python

"""This python file serves to function as a way to gather the incoming ARUCO
    tag data and send it to the action client."""

# Currently, this code is all in respect to the camera. We are relying on
# move_base which hasnt been implemented yet once the camera rotates or if
# we need to posistion the camera differently on the physical model.
# So it is sending a goal to the move_base 1 meter away from the aruco tag,
# but ALSO in respect to where the camera is without respect to camera
# rotation or posistion on the rover

import rospy
import actionlib
from aruco_goal.srv import aruco_follow
from aruco_goal.srv import aruco_post_id
from aruco_goal.srv import stop_goal
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import Int32, Bool, String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, Point, Twist
from ui_interface.msg import aruco_post
from autonomous_interface import AutonomousInterfaceClient
from servicemonitor import ServiceMonitor

global CURRENT_ARUCO_POST
CURRENT_ARUCO_POST = None


def feedback_cb(feedback):
    """Prints out any feedback from the Autonomous Interface Client."""
    print(feedback)


# monitors the aruco_follow service
srv_monitor = ServiceMonitor()


def stop_follow_cb(data):
    """If true, stop following ARUCO code."""
    if data.data:
        print("Stopping ARUCO follow")
        srv_monitor.stop()


def set_aruco_id(req):
    """Callback function for the '/current_aruco_post' service. Sets
    CURRENT_ARUCO_POST to the int received. If a new ARUCO post is set, any
    active move base goal gets canceled."""

    global CURRENT_ARUCO_POST

    # for any subsequent calls, cancel the current move base goal
    if CURRENT_ARUCO_POST != None:
        print("canceling move base goal")
        move_base.cancel_goal()

        # exits the while loop in go to post mode
        if not srv_monitor.is_following:
            print("stopping waiting for result")
            srv_monitor.stop_waiting_for_result()

    CURRENT_ARUCO_POST = req.aruco_id

    print("Current ARUCO ID  = {}".format(CURRENT_ARUCO_POST))

    # return current id to indicate successfully received request
    return CURRENT_ARUCO_POST


def aruco_follow_cb(req):
    """Sets True for Follow mode. False for Go To Post mode. If switching
    modes, cancel any move base goal and set the new mode."""

    # for any subsequent calls, cancel the current move base goal
    if srv_monitor.is_following != None:
        move_base.cancel_goal()

        # exits the while loop in go to post mode
        if not srv_monitor.is_following:
            print("stopping waiting for result")
            srv_monitor.stop_waiting_for_result()

    # set the mode
    srv_monitor.set_mode(req.is_following)

    # return true to indicate successfully received request
    return True


def stop_goal_cb(req):
    """Cancels any current move base goal and shuts down the node."""
    print("Clearing move base goal")

    if srv_monitor.is_following != None:
        move_base.cancel_goal()

        # exits the while loop in go to post mode
        if not srv_monitor.is_following:
            print("stopping waiting for result")
            srv_monitor.stop_waiting_for_result()

    print("exiting")
    srv_monitor.stop()
    return True


def callback(data):
    """Callback function for the /fiducial_transforms topic. Whenever an
    ARUCO tag is detected, create a MoveBaseGoal to send to the
    action client."""

    global CURRENT_ARUCO_POST
    for index in range(len(data.transforms)):
        fiducial_id = data.transforms[index].fiducial_id

        # convert fiducial id to an int if it is not None
        if fiducial_id != None:
            fiducial_id = int(fiducial_id)

        # if a fiducial marker matching the current ARUCO post has been detected
        # and a mode is set, create and send a goal
        if (
            fiducial_id == CURRENT_ARUCO_POST
            and fiducial_id != None
            and srv_monitor.is_following != None
        ):
            # an ARUCO tag with the current ARUCO id has been detected, send
            # True to '/current_aruco_tag_detected'
            response = Bool()
            response.data = True
            detect_pub.publish(response)

            print(fiducial_id)

            print(str(data.transforms[index].transform.translation.z))

            action_client_prep(
                data.transforms[index].transform.translation,
                data.transforms[index].transform.rotation,
            )
        elif fiducial_id != CURRENT_ARUCO_POST and fiducial_id != None:
            # if an ARUCO tag is detected AND it is not the current ARUCO ID,
            # send False to current_aruco_tag_detected
            response = Bool()
            response.data = False
            detect_pub.publish(response)


def action_client_prep(translation, rotation):
    """Creates and sends a MoveBaseGoal to action client using the translation
    and rotation vectors of an ARUCO tag's FiducialTransform."""

    print("calculating")

    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = "map"  # send a name
    goal.target_pose.header.stamp = rospy.Time.now()  # send a timestamp

    # move_base goal = position 1.5 meters away from aruco code (x, y, z),
    # quaternion (which way the rover is facing when reaching the ARUCO tag)

    goal.target_pose.pose.position = get_goal_position(translation)

    # set the target orientation the orientation of the aruco,
    # which will turn the rover where the aruco is facing currently
    goal.target_pose.pose.orientation = get_orientation(rotation)

    print("sending goal")
    move_base.send_goal(goal)

    if not srv_monitor.is_following:
        print("logging")
        log_results()

    # subscribe to the '/cmd_vel' topic
    rospy.Subscriber("/cmd_vel", Twist, vel_cb)


def get_orientation(aruco_rotation):
    """Calculates the Quaternion (orientation) of the rover when it reaches
    the MoveBaseGoal's position."""
    rover_w = 0
    rover_x = 0
    rover_y = 0
    rover_z = 0  # getroverrotation()?

    w = aruco_rotation.w
    x = aruco_rotation.x
    y = aruco_rotation.y
    z = aruco_rotation.z

    goal_w = 1.0
    goal_x = 0
    goal_y = 0
    goal_z = 0

    # the return quaternion would be the pose that the rover would be in when
    # it reaches the position.
    # Imagine it reaches the aruco going directly to it, the orientation of
    # where it would be looking after is this, it's also in respect to the
    # camera right now, currently its passing the aruco's rotation exactly
    # so the rover will face where the aruco is facing.

    return Quaternion(goal_x, goal_y, goal_z, goal_w)


def get_goal_position(translation):
    """Calculates the goal's position 1 meter away from the detected
    ARUCO Tag."""

    # ARUCO position
    print(translation.x, translation.y, translation.z)
    aruco_x = translation.x
    aruco_y = translation.y
    aruco_z = translation.z  # z = depth of image (distance to travel to)

    distance_from_post = 1  # distance of rover from ARUCO tag (in meters) 1.5 m

    # goal_x = aruco_x
    # goal_y = aruco_y
    # goal_z = aruco_z - distance_from_post

    goal_x = aruco_z - distance_from_post
    goal_y = -aruco_x
    goal_z = -aruco_y

    if(goal_x < .5):
        goal_x = 0
        goal_y = 0
        goal_z = 0

    return Point(goal_x, goal_y, goal_z)


def log_results():
    """Waits for the action server to complete the current goal. If the goal
    has not been completed, shut down the current ROS node. Else, log that
    the goal has been received."""

    rate = rospy.Rate(10)

    response = Int32()

    # wait for result
    while not rospy.is_shutdown():
        # if not waiting for result, exit the loop
        if not srv_monitor.waiting_for_result:
            srv_monitor.waiting_for_result = True
            break

        print("waiting for log_results()")
        print(move_base.get_state())
        if move_base.get_state() == 8:
            # communicate with UI that the goal was preempted
            response.data = -8
            pub.publish(response)
            print("move base goal was preempted")
            break

        if move_base.get_state() == 3:
            # communicate with UI that we have reached goal and to reset aruco tag
            response.data = -1
            pub.publish(response)
            rospy.loginfo("Goal has been recieved with response")

            # goal succeeded, so stop the service callback
            srv_monitor.goal_succeeded()
            srv_monitor.stop()
            print("move_base got the aruco transform")
            break

        if move_base.get_state() == 1:
            # communicate with UI that we havent reached goal
            response.data = -2
            pub.publish(response)
            print(response.data)

        if move_base.get_state() == 0:
            # communicate with UI that goal is pending, waiting to be processed
            response.data = -10
            pub.publish(response)
            print(response.data)

        rate.sleep()


def vel_cb(msg):
    """Prints out the Twist received whenever a new message is published to
    the '/cmd_vel' topic."""
    #print(msg)


if __name__ == "__main__":
    # Creates an Autonomous Interface Client, also initializes current node
    auto_client = AutonomousInterfaceClient()

    # Creates a service to manage requests to follow/not follow an ARUCO tag
    rospy.Service("aruco_follow", aruco_follow, aruco_follow_cb)

    # Creates a service to set the current ARUCO post ID
    rospy.Service("current_aruco_post", aruco_post_id, set_aruco_id)

    # Creates a service to listen to requests to stop the node and any move base goals
    rospy.Service("stop_aruco_goal", stop_goal, stop_goal_cb)

    # Create action client server to send a MoveBaseAction
    move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

    # publisher to communicate a goal's status to UI
    pub = rospy.Publisher("/fiducial_goal_status", Int32, queue_size=1)

    # publisher to communicate to the UI if the current aruco tag ID has been detected
    detect_pub = rospy.Publisher("/current_aruco_tag_detected", Bool, queue_size=1)

    demo_pub = rospy.Publisher("/demo_mode_status", String, queue_size=10)

    move_base.wait_for_server(rospy.Duration(2))

    try:
        print("starting aruco_goal process")

        print(
            "To start listening to '/fiducial_transforms', send a request to 'aruco_follow' and 'current_aruco_post'"
        )

        # listen for FiducialTransformArray messages (multiple fiducial markers)
        fiducial_sub = rospy.Subscriber(
            "/fiducial_transforms", FiducialTransformArray, callback
        )

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # not following ARUCO and goal was successfully sent
            if srv_monitor.is_successful and not srv_monitor.is_running:
                print("Goal was successful! Stopping")
                break

            # canceled the service without setting a mode
            if srv_monitor.is_following == None and not srv_monitor.is_running:
                print("exiting without doing anything")
                break

            # canceled Go To Post Mode
            if not srv_monitor.is_following and not srv_monitor.is_running:
                print("Stopping Go to Post mode")
                break

            # following ARUCO and request to stop following was sent
            if srv_monitor.is_following and not srv_monitor.is_running:
                print("Stopped following ARUCO POST")
                break

            if srv_monitor.is_following and CURRENT_ARUCO_POST != None:
                demo_pub.publish("Demo mode is running")
            else:
                demo_pub.publish("Demo mode is not running")
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
