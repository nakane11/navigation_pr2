#!/usr/bin/env python
# -*- coding: utf-8 -*-

from navigation_pr2 import SpeakClient
from navigation_pr2.mapping import *
from navigation_pr2.navigation import *
from navigation_pr2.idling import *
from navigation_pr2.hand_impact import *
import skrobot
from skrobot.interfaces.ros import PR2ROSRobotInterface
from navigation_pr2.elevator import *
import tf
import rospy
import smach
import smach_ros
from speech_recognition_msgs.msg import SpeechRecognitionCandidatesStamped

class NavigationSmach():
    def __init__(self):
        rospy.init_node('navigation_state_machine')
        self.debug = rospy.get_param('~debug', False)
        self.last_speech_time = rospy.Time.now()
        self.speech_sub = rospy.Subscriber('/Tablet/voice_stamped', SpeechRecognitionCandidatesStamped, self.speech_cb)
        self.speak = SpeakClient()
        self.listener = tf.TransformListener()
        self.r = skrobot.models.PR2()
        self.ri = PR2ROSRobotInterface(self.r)

    def speech_cb(self, msg):
        if msg.header.stamp - self.last_speech_time > rospy.Duration(0) or self.debug:
            rospy.set_param('~speech_raw', msg.candidates.transcript[0].replace(' ', ''))
            self.last_speech_time = msg.header.stamp

    def smach(self):
        ###################################
        ############  MAPPING  ############
        ###################################
        print(0)
        con_mapping = smach.Concurrence(outcomes=['outcome', 'succeeded', 'start navigation', 'elevator'],
                                        default_outcome='outcome', 
                                        child_termination_cb = con_mapping_child_term_cb,
                                        outcome_cb=con_mapping_out_cb)

        with con_mapping:
            sm_record_with_name = smach.StateMachine(outcomes=['succeeded', 'request navigation', 'elevator'])
            with sm_record_with_name:
                smach.StateMachine.add('WAIT_FOR_TEACHING', WaitForTeaching(client=self.speak),
                                       transitions={'timeout':'EXPLAIN',
                                                    'name received':'SEND_WITH_NAME',
                                                    'description received':'SEND_DESCRIPTION',
                                                    'end':'succeeded',
                                                    'request navigation':'request navigation',
                                                    'aborted':'WAIT_FOR_TEACHING',
                                                    'cancelled':'SEND_CANCEL_NAME'})
                smach.StateMachine.add('SEND_WITH_NAME', SendWithName(client=self.speak),
                                       transitions={'send spot with name':'WAIT_FOR_TEACHING',
                                                    'register elevator':'elevator'})
                smach.StateMachine.add('SEND_DESCRIPTION', SendDescription(),
                                       transitions={'succeeded':'WAIT_FOR_TEACHING'})
                smach.StateMachine.add('SEND_CANCEL_NAME', SendCancelName(),
                                       transitions={'succeeded':'WAIT_FOR_TEACHING'})
                smach.StateMachine.add('EXPLAIN', ExplainMapping(),
                                       transitions={'succeeded':'WAIT_FOR_TEACHING'})
            smach.Concurrence.add('RECORD_WITH_NAME', sm_record_with_name)
            smach.Concurrence.add('RECORD_WITHOUT_NAME', SwitchRecordWithoutName())

        sm_teach_elevator = smach.StateMachine(outcomes=['succeeded', 'aborted'],
                                               input_keys=['riding_position', 'adjust_riding'],
                                               output_keys=['riding_position', 'adjust_riding'])
        with sm_teach_elevator:
            smach.StateMachine.add('TEACH_RIDING_POSITION', TeachRidingPosition(client=self.speak, listener=self.listener),
                                   transitions={'succeeded':'TEACH_INSIDE_POSITION',
                                                'aborted':'TEACH_RIDING_POSITION'})
            smach.StateMachine.add('TEACH_INSIDE_POSITION', TeachInsidePosition(client=self.speak, ri=self.ri),
                                   transitions={'succeeded':'WAIT_FOR_NEXT_FLOOR_TEACHING',
                                                'aborted':'TEACH_INSIDE_POSITION'})
            smach.StateMachine.add('WAIT_FOR_NEXT_FLOOR_TEACHING', WaitforNextFloor(client=self.speak),
                                   transitions={'succeeded':'MOVE_TO_EXIT',
                                                'aborted':'WAIT_FOR_NEXT_FLOOR_TEACHING'})
            smach.StateMachine.add('MOVE_TO_EXIT', MovetoExit(client=self.speak, ri=self.ri),
                                   transitions={'succeeded':'TEACH_OUTSIDE_ELEVATOR',
                                                'aborted':'aborted'})
            smach.StateMachine.add('TEACH_OUTSIDE_ELEVATOR', TeachOutsideElevator(client=self.speak),
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'TEACH_OUTSIDE_ELEVATOR'})

        ###################################
        ###########  NAVIGATION  ##########
        ###################################
        print(1)
        sm_navigation = smach.StateMachine(outcomes=['succeeded', 'aborted', 'start mapping'])
        with sm_navigation:
            con_moving = smach.Concurrence(outcomes=['outcome', 'succeeded', 'ask', 'reached',
                                                     'interrupt', 'aborted', 'start mapping', 'elevator'],
                                           input_keys=['waypoints', 'next_point', 'goal_spot'],
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
                sm_send_waypoint = smach.StateMachine(outcomes=['preempted', 'succeeded', 'aborted', 'elevator'],
                                                      input_keys=['waypoints', 'next_point', 'goal_spot'])
                with sm_send_waypoint:
                    smach.StateMachine.add('CHECK_IF_GOAL_REACHED', CheckIfGoalReached(client=self.speak),
                                           transitions={'succeeded':'succeeded',
                                                        'unreached':'EXECUTE_STATE',
                                                        'aborted':'aborted'})
                    smach.StateMachine.add('EXECUTE_STATE', ExecuteState(),
                                           transitions={'succeeded':'CHECK_IF_GOAL_REACHED',
                                                        'aborted':'aborted',
                                                        'move':'SEND_MOVE_TO'})
                    smach.StateMachine.add('SEND_MOVE_TO', SendMoveTo(),
                                           transitions={'succeeded':'CHECK_IF_GOAL_REACHED',
                                                        'aborted':'aborted',
                                                        'preempted':'aborted',
                                                        'elevator':'elevator'})
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

            sm_navigation_elevator = smach.StateMachine(outcomes=['succeeded', 'aborted'],
                                         input_keys=['riding_position', 'adjust_riding'])
            with sm_navigation_elevator:
                smach.StateMachine.add('WAIT_FOR_ELEVATOR_AVAILABLE', WaitforElevatorAvailable(client=self.speak),
                                       transitions={'succeeded':'MOVE_TO_RIDING_POSITION',
                                                    'aborted':'WAIT_FOR_ELEVATOR_AVAILABLE'})
                smach.StateMachine.add('MOVE_TO_RIDING_POSITION', MovetoRidingPosition(),
                                       transitions={'succeeded':'MOVE_TO_INSIDE_POSITION',
                                                    'aborted':'MOVE_TO_RIDING_POSITION'})
                smach.StateMachine.add('MOVE_TO_INSIDE_POSITION', MovetoInsidePosition(ri=self.ri),
                                       transitions={'succeeded':'HOLD_DOOR',
                                                    'aborted':'MOVE_TO_INSIDE_POSITION'})
                smach.StateMachine.add('HOLD_DOOR', HoldDoor(model=self.r, ri=self.ri),
                                       transitions={'succeeded':'WAIT_FOR_NEXT_FLOOR_NAVIGATION',
                                                    'aborted':'aborted'})
                smach.StateMachine.add('WAIT_FOR_NEXT_FLOOR_NAVIGATION', WaitforNextFloor(client=self.speak),
                                       transitions={'succeeded':'succeeded',
                                                    'aborted':'aborted'})


            smach.StateMachine.add('SET_GOAL', SetGoal(client=self.speak),
                                   transitions={'succeeded':'GET_WAYPOINTS',
                                                'aborted':'aborted'})
            smach.StateMachine.add('GET_WAYPOINTS', GetWaypoints(client=self.speak),
                                   transitions={'ready to move':'START_NAVIGATION',
                                                'multiple goals':'SUGGEST_GOALS',
                                                'no path found':'aborted'})
            smach.StateMachine.add('SUGGEST_GOALS', SuggestGoals(client=self.speak),
                                   transitions={'ready to move':'START_NAVIGATION',
                                                'rejected':'SUGGEST_GOALS',
                                                'rejected all':'aborted'})

            smach.StateMachine.add('START_NAVIGATION', StartNavigation(client=self.speak),
                                   transitions={'succeeded': 'MOVING',
                                                'timeout': 'aborted'})
            smach.StateMachine.add('MOVING', con_moving,
                                   transitions={'outcome':'MOVING',
                                                'aborted':'FINISH_NAVIGATION',
                                                'succeeded':'FINISH_NAVIGATION',
                                                'start mapping':'start mapping',
                                                'interrupt':'INTERRUPT',
                                                'ask':'ASK_WHAT',
                                                'reached': 'FINISH_NAVIGATION',
                                                'elevator':'NAVIGATION_ELEVATOR'})
            smach.StateMachine.add('INTERRUPT', Interrupt(),
                                   transitions={'resume': 'MOVING',
                                                'aborted':'FINISH_NAVIGATION'})
            smach.StateMachine.add('ASK_WHAT', AskWhat(client=self.speak),
                                   transitions={'timeout':'MOVING',
                                                'interrupt':'INTERRUPT',
                                                'preempted':'FINISH_NAVIGATION'})
            smach.StateMachine.add('FINISH_NAVIGATION', FinishNavigation(),
                                   transitions={'succeeded': 'succeeded'})

            smach.StateMachine.add('NAVIGATION_ELEVATOR', sm_navigation_elevator,
                                   transitions={'succeeded': 'MOVING',
                                                'aborted': 'aborted'})
            
        ###################################
        ####### MAIN STATE MACHINE ########
        ###################################
        print(2)
        sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
        with sm:
            smach.StateMachine.add('INITIALIZE', Initialize(),
                                   transitions={'succeeded':'IDLING'})
            smach.StateMachine.add('EXPLAIN', Explain(client=self.speak),
                                   transitions={'succeeded':'IDLING'})
            smach.StateMachine.add('LIST_SPOTS', ListSpots(client=self.speak),
                                   transitions={'succeeded':'IDLING'})
            smach.StateMachine.add('INTRODUCTION', Introduction(client=self.speak),
                                   transitions={'succeeded':'EXPLAIN',
                                                'timeout':'IDLING',
                                                'aborted':'INTRODUCTION'})
            smach.StateMachine.add('IDLING', Idling(client=self.speak),
                                   transitions={'timeout':'EXPLAIN',
                                                'start navigation':'NAVIGATION',
                                                'start mapping':'START_MAPPING',
                                                'end':'SHUTDOWN',
                                                'aborted':'IDLING',
                                                'intro':'INTRODUCTION',
                                                'list spots':'LIST_SPOTS'})
            smach.StateMachine.add('NAVIGATION', sm_navigation,
                                   transitions={'succeeded':'IDLING',
                                                'aborted':'IDLING',
                                                'start mapping':'START_MAPPING'})
            smach.StateMachine.add('START_MAPPING', StartMapping(),
                                   transitions={'succeeded':'MAPPING'})
            smach.StateMachine.add('MAPPING', con_mapping,
                                   transitions={'outcome':'MAPPING',
                                                'succeeded':'FINISH_MAPPING',
                                                'start navigation':'NAVIGATION',
                                                'elevator':'TEACH_ELEVATOR'})
            smach.StateMachine.add('TEACH_ELEVATOR', sm_teach_elevator,
                                   transitions={'succeeded':'MAPPING',
                                                'aborted':'FINISH_MAPPING'})
            smach.StateMachine.add('FINISH_MAPPING', FinishMapping(),
                                   transitions={'succeeded':'IDLING'})
            smach.StateMachine.add('SHUTDOWN', Shutdown(),
                                   transitions={'succeeded':'succeeded'})

        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()
        outcome = sm.execute()

if __name__ == '__main__':
    ns = NavigationSmach()
    ns.smach()
    rospy.spin()
    sis.stop()
