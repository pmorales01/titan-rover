class ServiceMonitor:
    """A class that monitors when to stop the while loops in __main__ and
    in Go To Post mode when waiting for a result from move_base."""

    def __init__(self):
        self._is_following = None
        self._is_running = True
        self._is_successful = False
        self._waiting_for_result = True

    def set_mode(self, should_follow):
        """Sets flag to true if following ARUCO codes."""
        if should_follow:
            self._is_following = True
        else:
            self._is_following = False

    def goal_succeeded(self):
        """Acknowledges that move base goal has been reached."""
        self._is_successful = True

    def stop(self):
        """Stops the while loop from running."""
        self._is_running = False

    def stop_waiting_for_result(self):
        """Stops the while loop that waits for a goal's result from move_base
        in Go To Post mode."""
        self._waiting_for_result = False

    @property
    def waiting_for_result(self):
        """Returns true if waiting for a goal's result from move_base."""
        return self._waiting_for_result

    @waiting_for_result.setter
    def waiting_for_result(self, value):
        """Sets the _waiting_for_result flag to True or False"""
        self._waiting_for_result = value

    @property
    def is_running(self):
        """Returns true if the service callback is still running."""
        return self._is_running

    @property
    def is_successful(self):
        """Returns true if move base goal has been reached."""
        return self._is_successful

    @property
    def is_following(self):
        """Returns true if following an ARUCO tag."""
        return self._is_following
