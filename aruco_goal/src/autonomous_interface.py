#!/usr/bin/env python3
"""
Defines the AutonomousInterfaceServer/Client and support classes
"""
import rospy
import actionlib

import ui_interface.msg


class InterfaceStatus:
    """
    A status returned from the AutonomousInterfaceServer for action feedbacks ("feedback status") and 
    for action results ("result status")

    Class Attributes
    STATUS_DICT -- Contains response_phrase: response_code key: value pairs

    Instance Attributes
    response_phrase -- A short phrase describing the status instance
    response_code -- A three digit code identifying the status instance
    Code Ranges
    [100, 200) -- Informational statuses, unimplemented
    [200, 300) -- Success statuses
    [300, 400) -- Reserved, unimplemented
    [400, 500) -- Client failure statuses
    [500, 600) -- Server failure statuses

    Public Methods
    set_status(response_phrase) -- setter for the response_phrase variable
    and corresponding response_code value according to the STATUS_DICT
    get_status() -- returns response_phrase, response_code tuple
    """

    STATUS_DICT = {
        "OK": 200,  # Generic success
        "BAD REQUEST": 400,  # Generic client failure
        "SERVER ERROR": 500,  # Generic server failure
        "UNIMPLEMENTED": 550,  # Goal is planned to be supported but is not yet implemented
    }

    def __init__(self, response_code = None, response_phrase = None):
        """Accept response_code, response_phrase as args and set instance attributes."""
        self.response_phrase = response_phrase
        self.response_code = response_code

    def set_status(self, response_phrase):
        """Set response_phrase and set corrresponding response_code from STATUS_DICT."""
        self.response_phrase = response_phrase
        self.response_code = InterfaceStatus.STATUS_DICT.get(response_phrase)

    def get_status(self):
        """Return instance response_phrase and response_code tuple."""
        return self.response_phrase, int(self.response_code)


class AutonomousInterfaceServer:
    """
    Starts a ROS node named NODE_NAME. Extends the SimpleActionServer class for communication to/from
    the UI and autonomous modules.

    Class Attributes
    NODE_NAME -- Name of the ROS node
    LOG_LEVEL -- ROS logging level

    Instance Attributes
    server -- An instance of the SimpleActionServer class

    Public Methods
    execute(goal) -- Callback for the SimpleActionServer. Executes when a
    goal is received by the SimpleActionServer

    Private Methods

    along with some feedback to client
    _navigate_to_GNSS() -- Calls the necessary services to navigate to a GNSS coordinate
    _navigate_to_post() -- Calls the necessary services to navigate to a single Aruco code
    _navigate_to_gate() -- Calls the necessary services to navigate to a pair of Aurco codes
    _abort_navigation_attempt() -- Calls the necessary services to return to previous navigation goal
    """

    NODE_NAME = "autonomous_interface_server"
    LOG_LEVEL = rospy.INFO

    def __init__(self):
        """Starts node and action server."""

        rospy.init_node(
            AutonomousInterfaceServer.NODE_NAME,
            log_level=AutonomousInterfaceServer.LOG_LEVEL,
        )

        self.server = actionlib.SimpleActionServer(
            "autonomous_actions",
            ui_interface.msg.autonomous_actionsAction,
            self.execute,
            False,
        )

        self.server.start()
        rospy.loginfo("[%s] Started", AutonomousInterfaceServer.NODE_NAME)

    def execute(self, goal):
        """Parses goal sent to UI and enters corresponding function."""

        action = goal.autonomous_action

        rospy.loginfo(
            "[%s] Recevied action goal: %s", AutonomousInterfaceServer.NODE_NAME, action
        )

        if action == "navigate_to_GNSS":
            status = self._navigate_to_GNSS(goal.gnss_long, goal.gnss_lat)

        elif action == "navigate_to_post":
            status = self._navigate_to_post(goal.aruco_id_01, goal.follow_mode)

        elif action == "navigate_to_gate":
            status = self._navigate_to_gate(
                goal.aruco_id_01, goal.aruco_id_02, goal.follow_mode
            )

        elif action == "abort_navigation_attempt":
            status = self._abort_navigation_attempt()

        else:
            status = InterfaceStatus()
            status.set_status("BAD REQUEST")

        response_phrase, response_code = status.get_status()
        result = ui_interface.msg.autonomous_actionsResult(
            action_result_code=response_code
        )

        if response_code >= 200 and response_code < 300:
            self.server.set_succeeded(result)
            rospy.loginfo(
                "[%s] Action Goal Succeeded: %s",
                AutonomousInterfaceServer.NODE_NAME,
                action,
            )

        elif response_code >= 400 and response_code < 500:
            self.server.set_aborted(result)
            rospy.loginfo(
                "[%s] Action Goal Refused: %s",
                AutonomousInterfaceServer.NODE_NAME,
                action,
            )

        elif response_code >= 500:
            self.server.set_aborted(result)
            rospy.loginfo(
                "[%s] Action Goal Failed: %s",
                AutonomousInterfaceServer.NODE_NAME,
                action,
            )

    def _publish_success_feedback(self, feedback=''):
        """Publishes successful feedback with feedback body containing feedback"""
        _feedback = ui_interface.msg.autonomous_actionsFeedback(100, feedback)
        self.server.publish_feedback(_feedback)

    def _publish_fail_feedback(self, feedback=''):
        """Publishes failure feedback with feedback body containing feedback"""
        _feedback = ui_interface.msg.autonomous_actionsFeedback(150, feedback)
        self.server.publish_feedback(_feedback)

    def _navigate_to_GNSS(self, gnss_long, gnss_lat):
        """Call services to begin navigation to given GNSS coordinates. Return status."""
        status = InterfaceStatus()
        status.set_status("UNIMPLEMENTED")
        return status

    def _navigate_to_post(self, aruco_code, follow_mode):
        """Call services to begin navigation to given aruco code. Return status"""
        status = InterfaceStatus()
        status.set_status("UNIMPLEMENTED")
        return status

    def _navigate_to_gate(
        self, aruco_code_01, aruco_code_02, follow_mode):
        """Call services to begin navigation to aruco gate. Return status"""
        status = InterfaceStatus()
        status.set_status("UNIMPLEMENTED")
        return status

    def _abort_navigation_attempt(self):
        """
        Call services to stop current navigation attempt and return to previous goal. Return status
        """
        status = InterfaceStatus()
        status.set_status("UNIMPLEMENTED")
        return status


class AutonomousInterfaceClient:
    """
    Starts a ROS node named NODE_NAME. Extends the SimpleActionClient class and provides methods
    to send navigation goals to an AutonomousInterfaceServer node.

    Class Attributes
    NODE_NAME -- Name of the ROS node
    LOG_LEVEL -- ROS logging level

    Instance Attributes
    client -- An instance of the SimpleActionClient

    Public Methods
    navigate_to_GNSS()
    navigate_to_post()
    navigate_to_gate()
    abort_navigate_attempt()
    """

    NODE_NAME = "autonomous_interface_client"
    LOG_LEVEL = rospy.INFO

    def __init__(self):
        rospy.init_node(
            AutonomousInterfaceClient.NODE_NAME,
            anonymous=True,
            log_level=AutonomousInterfaceClient.LOG_LEVEL,
        )

        rospy.logdebug(
            "[%s] Autonomous interface client started",
            AutonomousInterfaceClient.NODE_NAME,
        )

        self.client = actionlib.SimpleActionClient(
            "autonomous_actions", ui_interface.msg.autonomous_actionsAction
        )

    def navigate_to_GNSS(
        self, feedback_cb_func, gnss_lat, gnss_long
    ):
        """
        Send Goal: 'navigate_to_GNSS' to AutonomousInterfaceServer
        """
        self.client.wait_for_server()
        goal = ui_interface.msg.autonomous_actionsGoal(
            "navigate_to_GNSS", 0, 0, gnss_lat, gnss_long, False
        )
        self.client.send_goal(goal, feedback_cb=feedback_cb_func)
        self.client.wait_for_result()
        return self.client.get_result()

    def navigate_to_post(
        self, feedback_cb_func, aruco_code, follow_mode=False,
    ):
        """
        Send Goal: 'navigate_to_post' to AutonomousInterfaceServer
        """
        self.client.wait_for_server()
        goal = ui_interface.msg.autonomous_actionsGoal(
            "navigate_to_post", aruco_code, 0, 0, 0, follow_mode
        )
        self.client.send_goal(goal, feedback_cb=feedback_cb_func)
        self.client.wait_for_result()
        return self.client.get_result()

    def navigate_to_gate(
        self, feedback_cb_func, aruco_code_01, aruco_code_02, follow_mode = False
    ):
        """
        Send Goal: 'navigate_to_gate' to AutonomousInterfaceServer
        """
        self.client.wait_for_server()
        goal = ui_interface.msg.autonomous_actionsGoal(
            "navigate_to_gate", aruco_code_01, aruco_code_02, 0, 0, follow_mode
        )
        self.client.send_goal(goal, feedback_cb=feedback_cb_func)
        self.client.wait_for_result()
        return self.client.get_result()

    def abort_navigate_attempt(self, feedback_cb_func):
        """
        Send Goal: 'abort_navigation_attempt' to AutonomousInterfaceServer
        """
        self.client.wait_for_server()
        goal = ui_interface.msg.autonomous_actionsGoal(
            "abort_navigation_attempt", 0, 0, 0, 0, False
        )
        self.client.send_goal(goal, feedback_cb=feedback_cb_func)
        self.client.wait_for_result()
        return self.client.get_result()


if __name__ == "__main__":
    """ros unit testing stuff goes here"""
    pass
