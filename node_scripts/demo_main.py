#!/usr/bin/env python

from navigation_pr2 import SpeakClient
from navigation_pr2.mapping import *
from navigation_pr2.navigation import *
from navigation_pr2.idling import *
from navigation_pr2.hand_impact import *

import rospy
import smach
import smach_ros
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

class NavigationSmach():
    def __init__(self):
        rospy.init_node('navigation_state_machine')
        self.speech_sub = rospy.Subscriber('/speech_to_roman/output', SpeechRecognitionCandidates, self.speech_cb)
        self.speak = SpeakClient()

    def speech_cb(self, msg):
        rospy.set_param('~speech_raw', msg.transcript[0])
        rospy.set_param('~speech_roman', msg.transcript[1])

    def smach(self):
        ###################################
        ############  MAPPING  ############
        ###################################
        con_mapping = smach.Concurrence(outcomes=['outcome', 'succeeded', 'start navigation'],
                                        output_keys=['map_available'],
                                        default_outcome='outcome', 
                                        child_termination_cb = con_mapping_child_term_cb,
                                        outcome_cb=con_mapping_out_cb)

        with con_mapping:
            
            sm_record_with_name = smach.StateMachine(outcomes=['succeeded', 'request navigation'],
                                                     output_keys=['map_available'])
            
            with sm_record_with_name:
                smach.StateMachine.add('WAIT_FOR_TEACHING', WaitForTeaching(client=self.speak),
                                       transitions={'timeout':'EXPLAIN',
                                                    'name received':'SEND_WITH_NAME',
                                                    'end':'succeeded',
                                                    'request navigation':'request navigation',
                                                    'aborted':'WAIT_FOR_TEACHING'})
                smach.StateMachine.add('SEND_WITH_NAME', SendWithName(),
                                       transitions={'send spot with name':'SET_MAP_AVAILABLE'})
                smach.StateMachine.add('SET_MAP_AVAILABLE', SetMapAvailable(),
                                       transitions={'succeeded':'WAIT_FOR_TEACHING'})

                smach.StateMachine.add('EXPLAIN', ExplainMapping(),
                                       transitions={'succeeded':'WAIT_FOR_TEACHING'})
            
            smach.Concurrence.add('RECORD_WITH_NAME', sm_record_with_name)
            smach.Concurrence.add('RECORD_WITHOUT_NAME', SwitchRecordWithoutName())

        ###################################
        ###########  NAVIGATION  ##########
        ###################################

        sm_navigation = smach.StateMachine(outcomes=['succeeded', 'aborted', 'start mapping'],
                                           input_keys=['map_available', 'goal_spot'])
        with sm_navigation:
            con_moving = smach.Concurrence(outcomes=['outcome', 'succeeded', 'ask',
                                                     'interrupt', 'aborted', 'start mapping'],
                                           default_outcome='outcome',
                                           child_termination_cb = con_moving_child_term_cb,
                                           outcome_cb=con_moving_out_cb)
            with con_moving:
        
                sm_talk_in_moving = smach.StateMachine(outcomes=['succeeded', 'interrupt', 'start mapping', 'aborted'])
                with sm_talk_in_moving:
                    smach.StateMachine.add('GET_SPEECH_IN_MOVING', GetSpeechinMoving(),
                                           transitions={'preempted':'aborted',
                                                        'end':'succeeded',
                                                        'request interrupt':'interrupt',
                                                        'start mapping':'start mapping',
                                                        'aborted':'GET_SPEECH_IN_MOVING'})
                sm_send_waypoint = smach.StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
                with sm_send_waypoint:
                    smach.StateMachine.add('CHECK_GOAL', CheckGoal(),
                                           transitions={'succeeded':'succeeded',
                                                        'unreached':'CHECK_ELEVATOR'})
                    smach.StateMachine.add('CHECK_ELEVATOR', CheckElevator(),
                                           transitions={'use':'MOVE_TO_ELEVATOR',
                                                        'not use':'SEND_MOVE_TO',
                                                        'aborted':'aborted'})
                    smach.StateMachine.add('MOVE_TO_ELEVATOR', MoveToElevator(),
                                           transitions={'succeeded':'SEND_MOVE_TO',
                                                        'aborted':'aborted'})
                    smach.StateMachine.add('SEND_MOVE_TO', SendMoveTo(),
                                           transitions={'succeeded':'CHECK_GOAL',
                                                        'aborted':'aborted'})
                sm_hand_impact = smach.StateMachine(outcomes=['aborted', 'succeeded'])
                with sm_hand_impact:
                    smach.StateMachine.add('WAIT_FOR_HAND_IMPACT', WaitforHandImpact(),
                                           transitions={'succeeded':'succeeded',
                                                        'detected':'WAIT_FOR_HAND_IMPACT',
                                                        'preempted':'aborted',
                                                        'aborted':'WAIT_FOR_HAND_IMPACT'})

                smach.Concurrence.add('SEND_WAYPOINT', sm_send_waypoint)
                smach.Concurrence.add('TALK_IN_MOVING', sm_talk_in_moving)
                smach.Concurrence.add('HAND_IMPACT', sm_hand_impact)


            smach.StateMachine.add('CHECK_IF_NAVIGATION_AVAILABLE', CheckIfNavigationAvailable(),
                                   transitions={'true':'GET_WAYPOINTS',
                                                'false':'aborted'})
            smach.StateMachine.add('GET_WAYPOINTS', GetWaypoints(),
                                   transitions={'ready to move':'MOVING',
                                                'no path found':'aborted'})
            smach.StateMachine.add('MOVING', con_moving,
                                   transitions={'outcome':'MOVING',
                                                'succeeded':'succeeded',
                                                'start mapping':'start mapping',
                                                'interrupt':'INTERRUPT',
                                                'ask':'ASK_WHAT'})
            smach.StateMachine.add('INTERRUPT', Interrupt(),
                                   transitions={'resume': 'MOVING',
                                                'aborted':'aborted'})
            smach.StateMachine.add('ASK_WHAT', AskWhat(client=self.speak),
                                   transitions={'timeout':'MOVING',
                                                'interrupt':'INTERRUPT',
                                                'preempted':'aborted'})

            
        ###################################
        ####### MAIN STATE MACHINE ########
        ###################################
        sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
        with sm:
            smach.StateMachine.add('INITIALIZE', Initialize(),
                                   transitions={'succeeded':'IDLING'})
            smach.StateMachine.add('IDLING', Idling(client=self.speak),
                                   transitions={'timeout':'IDLING',
                                                'start navigation':'NAVIGATION',
                                                'start mapping':'MAPPING',
                                                'end':'succeeded',
                                                'aborted':'IDLING'})
            smach.StateMachine.add('NAVIGATION', sm_navigation,
                                   transitions={'succeeded':'IDLING',
                                                'aborted':'IDLING',
                                                'start mapping':'MAPPING'})
            smach.StateMachine.add('MAPPING', con_mapping,
                                   transitions={'outcome':'MAPPING',
                                                'succeeded':'IDLING',
                                                'start navigation':'NAVIGATION'}
            )

        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()
        outcome = sm.execute()

if __name__ == '__main__':
    ns = NavigationSmach()
    ns.smach()
    rospy.spin()
    sis.stop()
