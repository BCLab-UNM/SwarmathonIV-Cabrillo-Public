"""
Watch a topic for activity. 
"""

class TopicWatcher: 

    INIT_STATE     = 0 
    ACTIVE_STATE   = 1
    WARNING_STATE  = 2 
    FAILED_STATE   = 3
    
    def __init__(self, topicname, init_time=60, warn_time=5, fail_time=10):
        self._state = TopicWatcher.INIT_STATE
        self._last_t = rospy.get_rostime()
        self._topic = topicname 
        self._sub = None 
        self._ns = rospy.get_namespace()
        self._init_t = init_time 
        self._warn_t = warn_time 
        self._fail_t = fail_time 
        self._msg = ''
        
    def _callback(self, message):
        self._last_t = rospy.get_rostime()
        
    def check(self):
        if self._state == TopicWatcher.INIT_STATE:
            if self._last_t < rospy.get_rostime() - rospy.Duration(self._init_t):
                print ('Topic never published. Entering failed state.')
                self._msg = "Topic " + self._topic + " was not published after " + str(self._init_t) + " seconds."
                self._state = TopicWatcher.FAILED_STATE
            else:
                topics = rospy.get_published_topics()
                for topic in topics:
                    topic_name, topic_type = topic
                    if topic_name == self._ns + self._topic:
                        # Subscribe
                        print ('Subscribing to', topic)
                        self._sub = rospy.Subscriber(topic_name, AnyMsg, self._callback)
                        self._state = TopicWatcher.ACTIVE_STATE 
                        self._last_t = rospy.get_rostime()

        elif self._state == TopicWatcher.ACTIVE_STATE:
            if self._last_t < rospy.get_rostime() - rospy.Duration(self._warn_t):
                print ('Entering the warning state.')
                self._state = TopicWatcher.WARNING_STATE

        elif self._state == TopicWatcher.WARNING_STATE:
            if self._last_t < rospy.get_rostime() - rospy.Duration(self._fail_t):
                print ('Entering the failed state.')
                self._state = TopicWatcher.FAILED_STATE
                self._msg = "Topic " + self._topic + " stopped publishing for " + str(self._fail_t) + " seconds."
            elif self._last_t >= rospy.get_rostime() - rospy.Duration(self._warn_t):
                print ('Saw a message, returning to OK state.')
                self._state = TopicWatcher.ACTIVE_STATE
                
        elif self._state == TopicWatcher.FAILED_STATE:
            print (self._msg)
            if self._sub is not None:
                self._sub.unregister()

        else:
            raise ValueError('Invalid state!')
