#!/usr/bin/env python
# -*- coding: utf-8 -*-

from importlib import import_module
from jsk_rviz_plugins.msg import OverlayText
from rosgraph_msgs.msg import Log
import rospy
import rostopic
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

def expr_eval(expr):
    def eval_fn(m):
        return eval(expr)
    return eval_fn

class RepublishConversation(object):

    def __init__(self):
        self.n_input = rospy.get_param('~number_of_input', 2)
        self.n_input = int(self.n_input)
        if self.n_input <= 0:
            rospy.logerr('~number_of_input should be greater than 0.')
            sys.exit(1)
        self.pub = rospy.Publisher("~output", OverlayText, queue_size=1)

        self.subs = {}
        self.colors = {}
        self.speakers = {}
        self.transform_fns = {}
        for i in range(self.n_input):
            topic_name = '~input{}'.format(i + 1)
            color = rospy.get_param('{}_color'.format(topic_name), 'white')
            speaker = rospy.get_param('{}_speaker'.format(topic_name), None)
            transform_expression = rospy.get_param('{}_transform'.format(topic_name), 'm')
            topic_name = rospy.resolve_name(topic_name)
            self.colors[topic_name] = color
            self.speakers[topic_name] = speaker
            self.transform_fns[topic_name] = expr_eval(transform_expression)
            sub = rospy.Subscriber(
                topic_name,
                rospy.AnyMsg,
                callback=lambda msg, tn=topic_name: self.callback(tn, msg),
                queue_size=1)
            self.subs[topic_name] = sub

        self.line_buffer_length = rospy.get_param("~line_buffer_length", 100)
        self.reverse_lines = rospy.get_param("~reverse_lines", True)
        self.lines = []
        
    def callback(self, topic_name, msg):
        if isinstance(msg, rospy.msg.AnyMsg):
            package, msg_type = msg._connection_header['type'].split('/')
            ros_pkg = package + '.msg'
            msg_class = getattr(import_module(ros_pkg), msg_type)
            sub = self.subs[topic_name]
            sub.unregister()
            deserialized_sub = rospy.Subscriber(
                topic_name, msg_class,
                lambda msg, tn=topic_name: self.callback(tn, msg))
            self.sub = deserialized_sub
            msg = msg_class().deserialize(msg._buff)
            
        transform_fn = self.transform_fns[topic_name]
        transformed_text = transform_fn(msg)
        print("msg:{}".format(msg))
        print("text:{}".format(transformed_text))
        if self.reverse_lines:
            self.lines = [self.colored_message(topic_name, transformed_text)] + self.lines
            if len(self.lines) > self.line_buffer_length:
                self.lines = self.lines[0:self.line_buffer_length]
        else:
            self.lines = self.lines + [self.colored_message(topic_name, transformed_text)]
            if len(self.lines) > self.line_buffer_length:
                self.lines = self.lines[-self.line_buffer_length:]
        text = OverlayText()
        text.left = 20
        text.top = 20
        text.width = 1200
        text.height = 1200
        text.fg_color.a = 1.0
        text.fg_color.r = 0.3
        text.text_size = 12
        text.text = "\n".join(self.lines)
        self.pub.publish(text)

    def colored_message(self, topic_name, text):
        cmsg = text.decode('utf-8')
        speaker = self.speakers[topic_name]
        if speaker is not None:
            cmsg = speaker + ': ' + cmsg
        color = self.colors[topic_name]
        return '<span style="color: {};">%s</span>'.format(color) % cmsg

if __name__ == "__main__":
    rospy.init_node('republish_conversation')
    RepublishConversation()
    rospy.spin()
