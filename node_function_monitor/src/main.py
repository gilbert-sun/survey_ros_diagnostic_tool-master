#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import diagnostic_msgs.msg
from enum import Enum
from rostopic import ROSTopicHz
import random
from node_monitor import topic_diagnostics,node_result,DataInfo
#from listener import topic_diagnostics,node_result,DataInfo
#from node_function_monitor.msg import BoundingBox


class BoundingBox():
    def __init__(self, clas, prob):
        self.Class= clas
        self.probablity =prob


def data_callback(data):
    global msg_data
    global content
    #if data from the topic exists, store display message
    if data:
        msg_data = DataInfo.msg_correct.value
    else:
        msg_data = DataInfo.msg_wrong.value
    content = data.data

def main():
    # initialize node
    nodename = "listener"

    rospy.init_node(nodename, anonymous=True)

    # state the topic you want to subscribe to for monitoring
    topicname = "/chatter"

    # the topic's message type
    # (if all your topic's message type are the same, you may declare only one topictype)
    topictype = String

    # describe a minimum publish rate per topic. If its the same for all topics, one minimum rate is enough
    min_rate = 2000

    # create a ROSTopicHz instance per topic you wish to get the publish rate from.
    rt = ROSTopicHz(-1, None)

    # subscribe to all the topics and run data_callback to check if the data messages are empty or not
    rospy.Subscriber(topicname, topictype, data_callback)

    # subscribe once again to get the rate of publishing messages by calling rt.callback_hz everytime a message is published
    rospy.Subscriber(topicname, topictype, rt.callback_hz)

    # we will be continually publishing to diagnostics topic
    # (first one is for monitoring topics that we are subscribed to)
    # (second one is for sending results)
    pub = rospy.Publisher('diagnostics', diagnostic_msgs.msg.DiagnosticArray, queue_size=10)
    pub2 = rospy.Publisher('diagnostics', diagnostic_msgs.msg.DiagnosticArray, queue_size=10)

    # rate at which the diagnostic messages will be published
    rate = rospy.Rate(1)

    # example : object recognition will recognize one of these objects and store it in data
    data = BoundingBox("dog", "90.1")
    # data = 0#BoundingBox()
    object = ['dog', 'cat', 'truck', 'person', 'tree', 'cup', 'table', 'chair', 'beer', 'chicken']
    r = random.randint(0, 9)
    data.Class = object[r]
    data.probability = random.uniform(0, 100)
    print ("-----------------",data.Class ,data.probability)

    # should have the topic_diagnostics and node_result function inside a loop to run as long as the node is active
    while not rospy.is_shutdown():
        # monitor topics that we are subscribed to
        topic_diagnostics(topicname, nodename, rt, pub, topictype, min_rate)

        # send a result to be viewed on the console; in this case, data
        node_result(nodename, pub2, data)

        rate.sleep()


if __name__ == '__main__':
    main()
