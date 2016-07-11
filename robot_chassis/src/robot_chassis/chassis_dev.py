#!/usr/bin/env python


"""
"""

import struct
import rospy
import thread
from std_msgs.msg import String

class ChassisDev(object):

    def __init__(self, name,
                 pub_topic = None,
                 pub_msg_class = String,
                 pub_rate = 1,
                 sub_topic = None,
                 sub_msg_class = String
                 ):

        self.name = name
        self.pub_topic = pub_topic
        self.pub_msg_class = pub_msg_class
        self.pub_rate = pub_rate
        self.sub_topic = sub_topic
        self.sub_msg_class = sub_msg_class

        self.pub_data = self.pub_msg_class()

        try:
        # init the node
            rospy.init_node(name, anonymous = True)
        except:
            pass

        if self.pub_topic:
            self.publisher = rospy.Publisher(self.pub_topic, pub_msg_class, queue_size = 10)

        self.sub_topic = sub_topic

    def pub_data_update(self, data):
        assert(type(data) == self.pub_msg_class)
        self.pub_data = data

    def start_pub(self):
        def pub_thread():
            if self.publisher and type(self.pub_data) == self.pub_msg_class:
                rate = rospy.Rate(self.pub_rate)
                while not rospy.is_shutdown():
                    self.publisher.publish(self.pub_data)
                    rate.sleep()

        thread.start_new_thread(pub_thread,())

    def data_handler(self, bin_data_pack):
        pass
    def start_sub(self, cb_func):
        #def sub_thread():
            rospy.Subscriber(self.sub_topic, self.sub_msg_class, cb_func)
            rospy.spin()

        #thread.start_new_thread(sub_thread, ())

    def dev_read(self):
        pass

    def dev_write(self):
        pass

    def dev_config(self):
        pass
