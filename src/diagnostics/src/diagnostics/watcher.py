"""
watcher package - Watch a topic for activity. 
"""

import rospy 
from rospy.msg import AnyMsg

class TopicWatcher: 
    """
    TopicWatcher - Watch a ROS topic for activity. Notify caller if messages stop.
    
    The topic watcher is a state machine. It waits for the topic to be published 
    then goes into the active state. If there is no message after the warning time
    a warning is generated, if a new message is received it goes back into the 
    active state. If no message is received before the fail time the topic watcher
    fails permanently. 
    
    """
    
    INIT_STATUS     = 0 
    ACTIVE_STATUS   = 1
    WARNING_STATUS  = 2 
    FAILED_STATUS   = 3
    
    def __init__(self, topicname, init_time=60, warn_time=5, fail_time=10):
        """
        Watch the specified topic. 
        
        topicname - The topic path, relative paths will have the namespace added. 
        init_time - Wait this long to see the topic published. (default 60 seconds)
        warn_time - Cause a warning if the topic has no messages after this much time (default 5 seconds) 
        fail_time - Cause an error if the topic has no message after this much time (default 10 seconds)
        """
        self._state = TopicWatcher.INIT_STATUS
        self._last_t = rospy.get_rostime()
        self._sub = None 
        self._ns = rospy.get_namespace()
        if not topicname.startswith('/'):
            self._topic = self._ns + topicname 
        else:
            self._topic = topicname             
        self._init_t = init_time 
        self._warn_t = warn_time 
        self._fail_t = fail_time 
        self._msg = ''
        
    def _callback(self, message):
        self._last_t = rospy.get_rostime()
        
    def check(self):
        """
        Check for activity. Returns the status of the topic. 
        """
        try:
            init_cutoff = rospy.get_rostime() - rospy.Duration(self._init_t)
            warn_cutoff = rospy.get_rostime() - rospy.Duration(self._warn_t)
            fail_cutoff = rospy.get_rostime() - rospy.Duration(self._fail_t)
        except TypeError as e:
            # This happens when the system hasn't been running longer than a cutoff duration. 
            return 
        
        if self._state == TopicWatcher.INIT_STATUS:
            if self._last_t < init_cutoff:
                self._msg = "Topic " + self._topic + " was not published after " + str(self._init_t) + " seconds."
                self._state = TopicWatcher.FAILED_STATUS
            else:
                topics = rospy.get_published_topics()
                for topic in topics:
                    topic_name, topic_type = topic
                    if topic_name == self._topic:
                        # Subscribe
                        self._sub = rospy.Subscriber(topic_name, AnyMsg, self._callback)
                        self._state = TopicWatcher.ACTIVE_STATUS 
                        self._last_t = rospy.get_rostime()

        elif self._state == TopicWatcher.ACTIVE_STATUS:
            if self._last_t < warn_cutoff:
                self._msg = "WARNING: No messages on " + self._topic + " for > " + str(self._warn_t) + " seconds."
                self._state = TopicWatcher.WARNING_STATUS

        elif self._state == TopicWatcher.WARNING_STATUS:
            if self._last_t < fail_cutoff:
                self._state = TopicWatcher.FAILED_STATUS
                self._msg = "ERROR: No messages on " + self._topic + " for > " + str(self._fail_t) + " seconds."
            elif self._last_t >= warn_cutoff:
                self._state = TopicWatcher.ACTIVE_STATUS
                
        elif self._state == TopicWatcher.FAILED_STATUS:
            if self._sub is not None:
                self._sub.unregister()

        else:
            raise ValueError('Invalid state!')

        return self._state
    
    def get_message(self):
        """Return the warning or failure message."""
        return self._msg

    def get_state_message(self):
        """Return the state and warning or failure message."""
        return (self._state, self._msg)
