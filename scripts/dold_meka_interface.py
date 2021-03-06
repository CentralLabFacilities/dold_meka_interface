#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# author Guillaume WALCK (2016)

import rospy

import actionlib

from dold_msgs.msg import DoldStates, DoldState
from m3meka_msgs.msg import M3ControlState, M3ControlStates, \
    M3StateChangeGoal, M3StateChangeAction
from m3meka_msgs.srv import M3ControlStateChange, M3ControlStateChangeResponse


class DoldMekaInterface(object):
    """
    Interface between Dold buttons and Mekabot
    """
    def __init__(self):

        # valid groups
        self._enabled_groups = {"left_hand": False,
                                "left_arm": False,
                                "head": False,
                                "right_arm": False,
                                "right_hand": False,
                                "zlift": False,
                                "torso": False,
                                "base": False}
        # mapping from button name to state change command
        self._button_cmd_map = {"B1": M3ControlState.STOP,
                                "B2": M3ControlState.FREEZE,
                                "B3": M3ControlState.START}

        self._last_message_time = 0
        self._service_ready = False

        

        self._actionclient = actionlib.SimpleActionClient("/meka_state_manager", M3StateChangeAction)
        rospy.loginfo("Looking for state manager...")
        if self._actionclient.wait_for_server(timeout=rospy.Duration(4)) is False:
            rospy.logfatal("Failed to connect to state_manager action server in 4 sec")
            self._service_ready = False
        else:
            rospy.loginfo("Found the state manager")
            self._service_ready = True
            
        self._state_sub = rospy.Subscriber("/meka_roscontrol_state_manager/state", M3ControlStates,
                                       self.state_callback,
                                       queue_size=2)

        self._button_sub = rospy.Subscriber("/dold_driver/state", DoldStates,
                                        self.button_callback,
                                        queue_size=2)

    def state_callback(self, msg):
        """
        callback to process state messages
        :param msg:
        :type msg: M3ControlStates
        """

        for group_name, state in zip(msg.group_name, msg.state):
            if group_name in self._enabled_groups:
                
                if state > M3ControlState.DISABLE:
                    # print "group: ", group_name, " is enabled"
                    self._enabled_groups[group_name] = True
                else:
                    # print "group: ", group_name, " is disabled"
                    self._enabled_groups[group_name] = False

        if not self._service_ready:
            # test reconnection only each 5 seconds
            if (rospy.Time.now() - self._last_message_time) < rospy.Duration(5.0):
                return
            self._last_message_time = rospy.Time.now()
            if self._actionclient.wait_for_server(timeout=rospy.Duration(1)) is False:
                rospy.logfatal("Failed to connect to state_manager action server, trying again in 5 seconds")
            else:
                self._service_ready = True

    def button_callback(self, msg):
        """
        callback to process button messages
        :param msg:
        :type msg: DoldStates
        """
        cmd = None
        tmp_cmd = 0
        for event in msg.inputs:
            # print "dold event: ", event
            if event.type == DoldState.BUTTON:
                # print "detected a button"
                if event.name in self._button_cmd_map:
                    # print "button ", event.name, " is mapped"
                    if event.state == DoldState.PRESSED:
                        # print "button ", event.name, " was pressed"
                        tmp_cmd = self._button_cmd_map[event.name]
                        # print "the mapped command is : ", tmp_cmd
                        # always consider the lowest state change cmd (lower is safer)
                        if cmd is None:
                            cmd = tmp_cmd
                        if tmp_cmd < cmd:
                            cmd = tmp_cmd
        if cmd is not None and cmd > 0:
            # print "sending a command: ", cmd
            self.change_state(cmd)

    def change_state(self, cmd):
        """
        try change the state according to the command
        :param cmd : State to change to
        """
        goal = M3StateChangeGoal()
        goal.retries = 2
        goal.strategy = M3StateChangeGoal.RETRY_N_TIMES

        # find enabled groups
        for group_name in self._enabled_groups:
            if self._enabled_groups[group_name]:
                goal.command.group_name.append(group_name)
                goal.command.state.append(cmd)
        try:
            if len(goal.command.group_name) > 0:
                # print "sending goal: ", goal
                self._actionclient.send_goal(goal)
        except rospy.ROSException:
            rospy.logerr("Failed to call change state")


def main():
    rospy.init_node('dold_meka_interface')
    dold_meka_if = DoldMekaInterface()
    rospy.spin()

if __name__ == "__main__":
    main()
