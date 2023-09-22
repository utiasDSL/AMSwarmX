#!/usr/bin/env python

import sys
import rospy
import roslib
import rosgraph
import std_msgs.msg
from rostopic import get_topic_class, ROSTopicException

"""
Republish a subfield of a topic as another topic.
Useful for graph_rviz_plugin.
Based on rostopic echo (rostopic/__init__.py)

Usage:
subtopic_repub.py /topic/subtopic/field /my_republished_field [queue_size (default 10)]

Examples:
./subtopic_repub.py /imu/angular_velocity/x /angular_velocity_x
./subtopic_repub.py /hullbot/sensors/bno055/imu/angular_velocity_covariance[0] /angular_velocity_covariance_0

Author: Sammy Pfeiffer <sam.pfeiffer at hullbot.com>
"""


def rostopic_republish(input_subtopic, republish_topic, queue_size=10):
    """
    Republish new message on subtopic.

    :param subtopic: topic name, ``str``
    """
    # Getting all available topics to find our real topic
    published_topics = rospy.get_published_topics()
    real_topic = ""
    for t_name, t_type in published_topics:
        if t_name in input_subtopic:
            real_topic = t_name
    if not real_topic:
        raise ROSTopicException("Didn't find real topic, is it published?")
    msg_class, _, msg_eval = get_topic_class(input_subtopic, blocking=True)
    rospy.loginfo("Input topic is of type: {}".format(msg_class._type))

    basic_types = {
        'bool': std_msgs.msg.Bool,

        'float32': std_msgs.msg.Float32,
        'float64': std_msgs.msg.Float64,

        'int8': std_msgs.msg.Int8,
        'int16': std_msgs.msg.Int16,
        'int32': std_msgs.msg.Int32,
        'int64': std_msgs.msg.Int64,

        'uint8': std_msgs.msg.UInt8,
        'uint16': std_msgs.msg.UInt16,
        'uint32': std_msgs.msg.UInt32,
        'uint64': std_msgs.msg.UInt64,

        'string': std_msgs.msg.String,
        'char': std_msgs.msg.Char,
        'time': std_msgs.msg.Time,
    }

    # Get the type of the subfield
    if len(input_subtopic) > len(real_topic):
        subtopic = input_subtopic[len(real_topic):]
        subtopic = subtopic.strip('/')
        user_array_index = None
        # If the user added [X] to the field, we get the array index
        if '[' in subtopic.split('/')[-1]:
            # Capture the user specified array index, e.g., /topic[5] '5'
            user_array_index = subtopic.split('/')[-1].split('[')[1]
            user_array_index = int(user_array_index.replace(']', ''))
        if subtopic:
            fields = subtopic.split('/')
            submsg_class = msg_class
            indexes = []
            for field in fields:
                if '[' in field:
                    if '[' in input_subtopic:
                        size_array = int(field.split(
                            '[')[1].replace(']', ''))
                        field = field.replace(
                            '[' + str(size_array) + ']', '')
                    else:
                        raise ROSTopicException(
                            "The topic has an array member, not implemented.")
                # Store the index in the slots to later on access the subfields
                index = submsg_class.__slots__.index(field)
                indexes.append(index)

                type_information = submsg_class._slot_types[index]
                # Deal with array types
                if '[' in type_information:
                    if '[' in input_subtopic:
                        size_array = int(type_information.split(
                            '[')[1].replace(']', ''))
                        type_information = type_information.replace(
                            '[' + str(size_array) + ']', '')
                    else:
                        raise ROSTopicException(
                            "The topic has an array member, not implemented. You may want to add [0] or another element.")
                if type_information in basic_types:
                    submsg_class = basic_types[type_information]
                else:
                    submsg_class = roslib.message.get_message_class(
                        type_information)
    else:
        raise ROSTopicException(
            "You didn't provide a subtopic, you may want to use 'rostun topictools relay' instead")

    rospy.loginfo("Got submsg of type: {}".format(submsg_class._type))
    if user_array_index is not None:
        rospy.loginfo("Got an array index: {}".format(user_array_index))
    pub = rospy.Publisher(republish_topic, submsg_class,
                          queue_size=queue_size)

    def republish_callback(msg):
        # access subfield
        submsg = msg
        for i in indexes:
            submsg = submsg.__getattribute__(submsg.__slots__[i])
        if user_array_index is not None:
            try:
                submsg = submsg[user_array_index]
            except IndexError as e:
                rospy.logwarn_throttle(1.0,
                    "Exception on access array at pos {}, reported array size: {}, Exception: {}, raw array: {}".format(
                        user_array_index, len(submsg), e, submsg))
        pub.publish(submsg)

    sub = rospy.Subscriber(real_topic, msg_class, republish_callback)
    rospy.spin()


def usage():
    print("Usage:")
    print(
        sys.argv[0] + "/topic/subtopic/field /my_republished_field [queue_size (default 10)]")
    exit(0)


if __name__ == '__main__':
    args = rospy.myargv()
    if len(args) < 3:
        usage()
    subtopic = args[1]
    repub_topic = args[2]
    queue_size = 10
    if len(args) == 4:
        queue_size = args[3]
    rospy.init_node('subtopic_repub', anonymous=True)
    rospy.loginfo("Republishing subtopic: {} into topic: {} (with queue_size: {})".format(
        subtopic, repub_topic, queue_size))
    rostopic_republish(subtopic, repub_topic, queue_size=queue_size)
