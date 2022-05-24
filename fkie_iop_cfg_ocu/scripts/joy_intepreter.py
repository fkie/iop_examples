#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2019 PAL Robotics SL.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of PAL Robotics SL. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Added modification by Alexander Tiderko
# Date: 2022.04.11
# * backport to ROS from ROS2.
# * introduced deadman message, which is sent when the deadman button
#   or the axis is released.
# * deadman_buttons and buttons can can now be defined together for a
#   command. Same for axes.
# * introduced 'scale' command for dynamic setting of the scale value
#   with the joystick. Added a 'default' parameter for scale command.
# * multiple axes or button for a mapping value
# * introduced low-freq timer for a command output independent of the
#   joy message rate.
# * added 'deadzone' parameter for axis. Only when the value of the
#   axis exceeds the deadzone value, the result is scaled and set.
# Date: 2022.04.29
# * enabled axes usage like a button. Configuration:
#    axes: [-4.75]  # negative axis with index 4 and deadzone 0.75
# * added 'toggle' key
#   cycle through the values in the list each time you activate the item
# Date: 2022.05.04
# * allow axis_mappings and message_value in the same command
# * added support for topics with arrays in the messages.
#   Use 'array' key to define value in the axis_mappings.
# * added 'deadman_zero' to avoid publish command if axis_mappings
#   without deadman is defined.

import importlib
import typing

import rospy
import genpy.message
from rospy import ROSException
import sensor_msgs.msg
import actionlib
from threading import Thread
from rosservice import ROSServiceException

class JoyTeleopException(Exception):
    pass


def get_interface_type(type_name: str, interface_type: str) -> typing.Any:
    split = type_name.split('/')
    if len(split) != 3:
        raise JoyTeleopException("Invalid type_name '{}'".format(type_name))
    package = split[0]
    interface = split[1]
    message = split[2]
    if interface != interface_type:
        raise JoyTeleopException("Cannot use interface of type '{}' for an '{}'"
                                 .format(interface, interface_type))

    importname = package + '.' + interface_type
    if interface_type == 'action':
        importname = package + '.msg'
    mod = importlib.import_module(importname)
    return getattr(mod, message)


def set_member(msg: typing.Any, member: str, value: typing.Any, index: int = None) -> None:
    ml = member.split('-')
    if len(ml) < 1:
        return
    target = msg
    for i in ml[:-1]:
        target = getattr(target, i)
    if index is None:
        setattr(target, ml[-1], value)
    else:
        # set list item
        list_attr = getattr(target, ml[-1])
        if index < len(list_attr):
            list_attr[index] = value
        else:
            list_attr.append(value)


class JoyTeleopCommand:

    def __init__(self, name: str, config: typing.Dict[str, typing.Any], node) -> None:
        button_name = 'buttons'
        axes_name = 'axes'
        deadman_button_name = 'deadman_buttons'
        deadman_axis_name = 'deadman_axes'
        self.node = node
        self.name = name
        self.buttons: typing.List[str] = []
        if button_name in config:
            self.buttons = config[button_name]
        self.axes: typing.List[str] = []
        if axes_name in config:
            self.axes = config[axes_name]
        self.deadman_buttons: typing.List[str] = []
        if deadman_button_name in config:
            self.deadman_buttons = config[deadman_button_name]
        self.deadman_axes: typing.List[str] = []
        if deadman_axis_name in config:
            self.deadman_axes = config[deadman_axis_name]

        no_buttons = len(self.buttons) == 0 and len(self.deadman_buttons) == 0
        no_axis = len(self.axes) == 0 and len(self.deadman_axes) == 0
        if no_buttons and no_axis and 'axis_mappings' not in config:
            raise JoyTeleopException("No buttons, axes, deadman_buttons, deadman_axis or axis_mappings configured for command '{}'".format(name))

        is_deadman = len(self.deadman_buttons) != 0 or len(self.deadman_axes) != 0
        if is_deadman:
            deadman_text = 'with'
            if self.deadman_buttons:
                deadman_text = ' buttons: %s' % self.deadman_buttons
            if self.deadman_axes:
                deadman_text += ' axes: %s' % self.deadman_axes
            rospy.loginfo("  activate deadman for '{}' {}".format(name, deadman_text))
        # Used to short-circuit the run command if there aren't enough buttons in the message.
        self.min_button = int(min(self.buttons + self.deadman_buttons + [0]))
        self.min_axis = int(min(self.axes + self.deadman_axes + [0]))

        # This can be used to "debounce" the message; if there are multiple presses of the buttons
        # or axes, the command may only activate on the first one until it toggles again.  But this
        # is a command-specific behavior, the base class only provides the mechanism.
        self.active = False
        self.has_deadman = len(self.deadman_buttons) > 0 or len(self.deadman_axes) > 0

    def update_active_from_buttons_and_axes(self, joy_state: sensor_msgs.msg.Joy) -> None:
        self.active = False

        if (self.min_button is not None and len(joy_state.buttons) <= self.min_button) and \
           (self.min_axis is not None and len(joy_state.axes) <= self.min_axis):
            rospy.loginfo('Not enough buttons or axes, so it cannot possibly be a message for this command.')
            # Not enough buttons or axes, so it can't possibly be a message for this command.
            return

        deadman_active = len(self.deadman_buttons) == 0 and len(self.deadman_axes) == 0
        for button in self.deadman_buttons:
            deadman_active |= self.is_button_pressed(joy_state.buttons, button)
        for axis in self.deadman_axes:
            deadman_active |= self.is_axis_activated(joy_state.axes, axis)

        if deadman_active:
            self.active = len(self.axes) == 0 and len(self.buttons) == 0
            for button in self.buttons:
                self.active |= self.is_button_pressed(joy_state.buttons, button)
            for axis in self.axes:
                self.active |= self.is_axis_activated(joy_state.axes, axis)

    def is_button_pressed(self, buttons, button):
        try:
            return buttons[button] == 1
        except IndexError:
            # An index error can occur if this command is configured for multiple buttons
            # like (0, 10), but the length of the joystick buttons is only 1.  Ignore these.
            pass
        return False

    def is_axis_activated(self, axes, axis):
        deadzone = 0.0
        axis_idx = axis
        if isinstance(axis, float):
            # we have a coded axis value, e.g. -1.75 <= only negative axis with 0.75 deadzone
            deadzone = axis - int(axis)
            axis_idx = abs(int(axis))
        try:
            if axis > 0:
                return axes[axis_idx] >= deadzone
            else:
                return axes[axis_idx] <= deadzone
        except IndexError:
            # An index error can occur if this command is configured for multiple axes
            # like (0, 10), but the length of the joystick axes is only 1.  Ignore these.
            pass
        return False


class JoyTeleopTopicCommand(JoyTeleopCommand):

    def __init__(self, name: str, config: typing.Dict[str, typing.Any], node) -> None:
        super().__init__(name, config, node)
        self.topic_type = get_interface_type(config['interface_type'], 'msg')

        # A 'message_value' is a fixed message that is sent in response to an activation.  It is
        # mutually exclusive with an 'axis_mapping'.
        self.msg_value = None
        self.toggled_idx = {}
        if 'message_value' in config:
            msg_config = config['message_value']

            # Construct the fixed message and try to fill it in.  This message will be reused
            # during runtime, and has the side benefit of giving the user early feedback if the
            # config can't work.
            self.msg_value = self.topic_type()
            for target, param in msg_config.items():
                if isinstance(param, dict):
                    # parameter is a dictionary: it should countain a 'value' key
                    set_member(self.msg_value, target, param['value'])
                else:
                    set_member(self.msg_value, target, param)
        self.deadman_message = None
        if 'deadman_message' in config:
            msg_config = config['deadman_message']
            # Construct the fixed message and try to fill it in.  This message will be used
            # to send after the deadman button was released.
            self.deadman_message = self.topic_type()
            for target, param in msg_config.items():
                if isinstance(param, dict):
                    # parameter is a dictionary: it should countain a 'value' key
                    set_member(self.deadman_message, target, param['value'])
                else:
                    set_member(self.deadman_message, target, param)
        self.deadman_zero = False
        if 'deadman_zero' in config:
            self.deadman_zero = config['deadman_zero']
        self._deadman_zero_values = False

        # An 'axis_mapping' takes data from one part of the message and scales and offsets it to
        # publish if an activation happens.  This is typically used to take joystick analog data
        # and republish it as a cmd_vel.  It is mutually exclusive with a 'message_value'.
        self.axis_mappings = {}
        if 'axis_mappings' in config:
            self.axis_mappings = config['axis_mappings']
            # Now check that the mappings have all of the required configuration.
            for mapping, values in self.axis_mappings.items():
                if 'axis' not in values and 'button' not in values and 'value' not in values and 'toggle' not in values and 'array' not in values:
                    raise JoyTeleopException("Axis mapping for '{}' must have an axis, button, value or toggle".format(name))

                # if 'axis' in values:
                #     if 'offset' not in values:
                #         raise JoyTeleopException("Axis mapping for '{}' must have an offset"
                #                                  .format(name))

                #     if 'scale' not in values:
                #         raise JoyTeleopException("Axis mapping for '{}' must have a scale"
                #                                  .format(name))

        if self.msg_value is None and not self.axis_mappings:
            raise JoyTeleopException("No 'message_value' or 'axis_mappings' "
                                     "configured for command '{}'".format(name))
        # if self.msg_value is not None and self.axis_mappings:
        #     raise JoyTeleopException("Only one of 'message_value' or 'axis_mappings' "
        #                              "can be configured for command '{}'".format(name))

        if 'topic_name' not in config:
            raise JoyTeleopException("No 'topic_name' configured for command '{}'".format(name))

        self.pub = rospy.Publisher(config['topic_name'], self.topic_type, queue_size=1)

        # low-freq command publishing
        self.command_message = self.topic_type()
        self.rate_joy_last_ts = rospy.Time.from_sec(0.0)
        self.rate_timer = None
        if 'hz' in config and config['hz'] != 0.:
            if 'timeout' not in config:
                raise JoyTeleopException("'hz' without 'timeout' defined in command '{}'".format(name))
            self.rate = 1. / config['hz']
            self.rate_timeout = rospy.Duration(config['timeout'])
            self.rate_timer = rospy.Timer(rospy.Duration(self.rate), self.rate_callback)
            rospy.loginfo("  activate low-freq timer for '{}' with {}hz".format(name, config['hz']))
            self.rate_timeouted = False

    def rate_callback(self, event):
        if rospy.Time.now() - self.rate_joy_last_ts < self.rate_timeout:
            # publish command message as long as joy messages are received
            if hasattr(self.command_message, 'header'):
                self.command_message.header.stamp = rospy.Time.now()
            self.pub.publish(self.command_message)
        elif self.deadman_message is not None:
            # publish deadman_message once on timeout, if defined
            if not self.rate_timeouted:
                # If there is a stamp field, fill it with now().
                if hasattr(self.deadman_message, 'header'):
                    self.deadman_message.header.stamp = rospy.Time.now()
                self.pub.publish(self.deadman_message)
                self.rate_timeouted = True

    def get_scale_value(self, values: typing.Dict[str, typing.Any], default: float):
        value = values.get('scale')
        if value is None:
            return default
        if isinstance(value, float):
            return value
        # try to get scale value from defined by a command
        if value in self.node.scales:
            return self.node.scales[value].get_value()
        raise JoyTeleopException("Invalid scale value '{}'".format(value))

    def get_axis_value(self, values: typing.Dict[str, typing.Any], joy_value: float) -> float:
        ''' Apply deadzone and scale to axis value. '''
        result = joy_value
        # recalculate axis value if deadzone is defined
        deadzone = values.get('deadzone', 0.0)
        if deadzone != 0.0:
            if abs(joy_value) <= deadzone:
                result = 0.0
            else:
                factor = 1. / (1.0 - deadzone)
                result = (abs(joy_value) - deadzone) * factor
                if joy_value < 0:
                    result *= -1.
        # apply scale value
        result = result * self.get_scale_value(values, 1.0) + values.get('offset', 0.0)
        return result

    def _handle_axis_mapping(self, msg, joy_state: sensor_msgs.msg.Joy, mapping: str, values: dict) -> None:
        if 'axis' in values:
            if isinstance(values['axis'], dict):
                for index, axis_cfg in values['axis'].items():
                    axis_index = int(index)
                    if len(joy_state.axes) > axis_index:
                        val = self.get_axis_value(axis_cfg, joy_state.axes[axis_index])
                        if val != 0.0:
                            break
                    else:
                        rospy.logerr('Joystick has only {} axes (indexed from 0),'
                                    'but #{} was referenced in config.'.format(
                                                    len(joy_state.axes), axis_index))
            else:
                if len(joy_state.axes) > values['axis']:
                    val = self.get_axis_value(values, joy_state.axes[values['axis']])
                else:
                    rospy.logerr('Joystick has only {} axes (indexed from 0),'
                                'but #{} was referenced in config.'.format(
                                                len(joy_state.axes), values['axis']))
        elif 'button' in values:
            if isinstance(values['button'], dict):
                pass
            else:
                if len(joy_state.buttons) > values['button']:
                    val = joy_state.buttons[values['button']] * self.get_scale_value(values, 1.0) + \
                        values.get('offset', 0.0)
                else:
                    rospy.logerr('Joystick has only {} buttons (indexed from 0),'
                                'but #{} was referenced in config.'.format(
                                                len(joy_state.buttons), values['button']))
        elif 'value' in values:
            # Pass on the value as its Python-implicit type
            val = values.get('value')
        elif 'toggle' in values:
            # cycle through the values in the list each time you activate it
            idx = 0
            if mapping in self.toggled_idx:
                idx = self.toggled_idx[mapping] + 1
                if idx >= len(values['toggle']):
                    idx = 0
            val = values['toggle'][idx]
            self.toggled_idx[mapping] = idx
        else:
            rospy.logerr('No Supported axis_mappings type found in: {}'.format(mapping))
        value_index = None
        if val != 0:
            self._deadman_zero_values = False
        if 'index' in values:
            value_index = values['index']
        set_member(msg, mapping, val, value_index)

    def run(self, joy_state: sensor_msgs.msg.Joy) -> None:
        # The logic for responding to this joystick press is:
        # 1.  Save off the current state of active.
        # 2.  Update the current state of active based on buttons and axes.
        # 3.  If this command is currently not active, but was active, send deadman_message if defined and return.
        #     Otherwise return without publishing.
        # 4.  If this is no axis_mappings, and the value of the previous active is the same as now,
        #     debounce and return without publishing.
        # 5.  If this is axis_mappings and deadman_zero is True and all values are zero publish only once.
        # 5.  If hz defined and timer is active update the commad_message to deadman_message and exit without publishing.
        # 6.  In all other cases, publish.  This means that this is a msg_value and the button
        #     transitioned from 0 -> 1, or it means that this is an axis mapping and data should
        #     continue to be published without debouncing.
        last_active = self.active
        self.update_active_from_buttons_and_axes(joy_state)
        if not self.active:
            if last_active and self.deadman_message is not None:
                if self.rate_timer is not None:
                    # rate timer is active -> set the command message to deadman_message
                    self.command_message = self.deadman_message
                # If there is a stamp field, fill it with now().
                if hasattr(self.deadman_message, 'header'):
                    self.deadman_message.header.stamp = rospy.Time.now()
                self.pub.publish(self.deadman_message)
            return
        if len(self.axis_mappings) == 0 and last_active == self.active:
            return

        msg = None
        if self.msg_value is not None:
            # This is the case for a static message.
            msg = self.msg_value
        if msg is None:
            # This is the case to forward along mappings.
            msg = self.topic_type()
        if len(self.axis_mappings) > 0:
            last_deadman_zero_values = self._deadman_zero_values
            self._deadman_zero_values = True
            for mapping, values in self.axis_mappings.items():
                val = 0.0
                if 'array' in values:
                    for values_idx in values['array']:
                        self._handle_axis_mapping(msg, joy_state, mapping, values_idx)
                else:
                    self._handle_axis_mapping(msg, joy_state, mapping, values)
            # handle 'deadman_zero' if not deadman is defined
            if not self.has_deadman and self.deadman_zero:
                if self._deadman_zero_values:
                    if not last_deadman_zero_values:
                        if hasattr(msg, 'header'):
                            msg.header.stamp = rospy.Time.now()
                        self.pub.publish(msg)
                        # If there is a stamp field, fill it with now().
                        if hasattr(self.deadman_message, 'header'):
                            self.deadman_message.header.stamp = rospy.Time.now()
                        self.pub.publish(self.deadman_message)
                    return
        if self.rate_timer is not None:
            self.rate_joy_last_ts = rospy.Time.now()
            self.command_message = msg
            self.rate_timeouted = False
        else:
            # If there is a stamp field, fill it with now().
            if hasattr(msg, 'header'):
                msg.header.stamp = rospy.Time.now()
            self.pub.publish(msg)


class JoyTeleopServiceCommand(JoyTeleopCommand):

    class AsyncServiceProxy(object):
        def __init__(self, name, service_class, persistent):
            self._thread = None
            rospy.wait_for_service(name, timeout=2.0)
            self._service_proxy = rospy.ServiceProxy(name, service_class, persistent)

        def __del__(self):
            # try to join our thread - no way I know of to interrupt a service
            # request
            if self._thread is not None and self._thread.is_alive():
                self._thread.join(1.0)

        def __call__(self, request):
            if self._thread is not None and  self._thread.is_alive():
                self._thread.join(0.01)
                if self._thread.is_alive():
                    return False
            self._thread = Thread(target=self._service_proxy, args=[request])
            self._thread.start()
            return True
        
        def service_is_ready(self) -> bool:
            return self._thread is None or not self._thread.is_alive()


    def __init__(self, name: str, config: typing.Dict[str, typing.Any], node) -> None:
        super().__init__(name, config, node)

        self.service_type = get_interface_type(config['interface_type'], 'srv')
        self.service_name = config['service_name']
        self.request = get_interface_type(config['interface_type'] + 'Request', 'srv')()

        if 'service_request' in config:
            # Set the message fields in the request in the constructor.  This request will be used
            # during runtime, and has the side benefit of giving the user early feedback if the
            # config can't work.
            genpy.message.fill_message_args(self.request, [config['service_request']])

        self.service_client = None
        self.client_ready = False

    def run(self, joy_state: sensor_msgs.msg.Joy) -> None:
        # The logic for responding to this joystick press is:
        # 1.  Save off the current state of active.
        # 2.  Update the current state of active.
        # 3.  If this command is currently not active, return without calling the service.
        # 4.  Save off the current state of whether the service was ready.
        # 5.  Update whether the service is ready.
        # 6.  If the service is not currently ready, return without calling the service.
        # 7.  If the service was already ready, and the state of the button is the same as before,
        #     debounce and return without calling the service.
        # 8.  In all other cases, call the service.  This means that either this is a button
        #     transition 0 -> 1, or that the service became ready since the last call.
        last_active = self.active
        self.update_active_from_buttons_and_axes(joy_state)
        if not self.active:
            return
        last_ready = self.client_ready
        try:
            if self.service_client is None:
               self.service_client = self.AsyncServiceProxy(self.service_name, self.service_type, False)
            else:
                self.client_ready = self.service_client.service_is_ready()
                if not self.client_ready:
                    rospy.logwarn('Not sending new service request for command {} because previous request has not finished'
                                .format(self.name))
                    return
                if last_ready == self.client_ready and last_active == self.active:
                    return
                rospy.logdebug('Call service {} for command {}'.format(self.service_name, self.name))
                self.service_client(self.request)
        except ROSException as error:
            rospy.logwarn('Command {} failed: {}'.format(self.name, error))


class JoyTeleopActionCommand(JoyTeleopCommand):

    def __init__(self, name: str, config: typing.Dict[str, typing.Any], node) -> None:
        super().__init__(name, config, node)
        self.client_ready = False
        self.action_client = None
        self.action_name = config['action_name']
        self.action_type = get_interface_type(config['interface_type'], 'action')
        self.action_client = actionlib.SimpleActionClient(self.action_name, self.action_type)
        self.client_ready = True

        self.goal = get_interface_type(config['interface_type'][:-6] + 'Goal', 'action')()
        if 'action_goal' in config:
            # Set the message fields for the goal in the constructor.  This goal will be used
            # during runtime, and has hte side benefit of giving the user early feedback if the
            # config can't work.
            genpy.message.fill_message_args(self.goal, config['action_goal'])

    def run(self, joy_state: sensor_msgs.msg.Joy) -> None:
        # The logic for responding to this joystick press is:
        # 1.  Save off the current state of active.
        # 2.  Update the current state of active.
        # 3.  If this command is currently not active, return without calling the action.
        # 4.  Save off the current state of whether the action was ready.
        # 5.  Update whether the action is ready.
        # 6.  If the action is not currently ready, return without calling the action.
        # 7.  If the action was already ready, and the state of the button is the same as before,
        #     debounce and return without calling the action.
        # 8.  In all other cases, call the action.  This means that either this is a button
        #     transition 0 -> 1, or that the action became ready since the last call.
        last_active = self.active
        self.update_active_from_buttons_and_axes(joy_state)
        if not self.active:
            return
        last_ready = self.client_ready
        # for ros2
        # self.client_ready = self.action_client.server_is_ready()
        if not self.client_ready:
            rospy.logwarn('Action {} is not ready, ignore call request!'.format(self.name))
            return
        if last_ready == self.client_ready and last_active == self.active:
            return
        self.action_client.send_goal(self.goal)


class JoyTeleopScaleCommand(JoyTeleopCommand):

    def __init__(self, name: str, config: typing.Dict[str, typing.Any], node) -> None:
        super().__init__(name, config, node)
        # default scale value
        self.value = config.get('default', 1.0)
        # An 'axis_mapping' defines scale values for buttons or values.
        self.axis_mappings = {}
        if 'axis_mappings' in config:
            self.axis_mappings = config['axis_mappings']
            # Now check that the mappings have all of the required configuration.
            for mapping, values in self.axis_mappings.items():
                if 'axis' not in values and 'button' not in values and 'value' not in values:
                    raise JoyTeleopException("Axis mapping for '{}' must have an axis, button, "
                                             'or value'.format(name))
            if 'axis' in values:
                if 'offset' not in values:
                    raise JoyTeleopException("Axis mapping for '{}' must have an offset"
                                                .format(name))

                if 'scale' not in values:
                    raise JoyTeleopException("Axis mapping for '{}' must have a scale"
                                                .format(name))

    def get_value(self):
        return self.value

    def run(self, joy_state: sensor_msgs.msg.Joy) -> None:
        # The logic for responding to this joystick press is:
        # 1.  Save off the current state of active.
        # 2.  Update the current state of active.
        # 3.  If this command is currently not active, return without update the scale state.
        # 4.  Get value for pressed button.
        # 5.  Update scale value.

        last_active = self.active
        self.update_active_from_buttons_and_axes(joy_state)
        if not self.active:
            return
        val = None
        for mapping, values in self.axis_mappings.items():
            if 'button' in values:
                if len(joy_state.buttons) > values['button']:
                    button_value = joy_state.buttons[values['button']]
                    if button_value != 0:
                        val = values.get('scale', 1.0) + values.get('offset', 0.0)
                        if 'value' in values:
                            # Pass on the value as its Python-implicit type
                            val = values.get('value')
                else:
                    rospy.logerr('Joystick has only {} buttons (indexed from 0),'
                                    'but #{} was referenced in config.'.format(
                                                len(joy_state.buttons), values['button']))
            elif 'axis' in values:
                if len(joy_state.axes) > values['axis']:
                    val = joy_state.axes[values['axis']] * values.get('scale', 1.0) + \
                        values.get('offset', 0.0)
                else:
                    rospy.logerr('Joystick has only {} axes (indexed from 0),'
                                    'but #{} was referenced in config.'.format(
                                                len(joy_state.axes), values['axis']))
            else:
                rospy.logerr('No Supported axis_mappings type found in: {}'.format(mapping))

            if val is not None:
                if self.value != val:
                    rospy.logdebug('New {}: {}'.format(self.name, val))
                self.value = val


class JoyTeleop():
    """
    Generic joystick teleoperation node.

    Will not start without configuration, has to be stored in 'teleop' parameter.
    See config/joy_teleop.yaml for an example.
    """

    def __init__(self):
        if not rospy.has_param("teleop") and not rospy.has_param("~teleop"):
            rospy.logfatal("no configuration was found, taking node down")
            raise JoyTeleopException("no config")

        self.commands = []
        names = []
        self.scales = {}

        teleop_cfg = {}
        if rospy.has_param("~teleop"):
            teleop_cfg = rospy.get_param("~teleop")
        else:
            teleop_cfg = rospy.get_param("teleop")

        for name, config in teleop_cfg.items():
            rospy.loginfo('Create command {}'.format(name))
            if name in names:
                raise JoyTeleopException("command '{}' was duplicated".format(name))
            try:
                interface_group = config['type']

                if interface_group == 'topic':
                    self.commands.append(JoyTeleopTopicCommand(name, config, self))
                elif interface_group == 'service':
                    self.commands.append(JoyTeleopServiceCommand(name, config, self))
                elif interface_group == 'action':
                    self.commands.append(JoyTeleopActionCommand(name, config, self))
                elif interface_group == 'scale':
                    self.scales[name] = JoyTeleopScaleCommand(name, config, self)
                else:
                    raise JoyTeleopException("unknown type '{interface_group}' "
                                             "for command '{name}'".format_map(locals()))
            except TypeError:
                import traceback
                print(traceback.format_exc())
                # This can happen on parameters we don't control, like 'use_sim_time'.
                rospy.logwarn('parameter {} is not a dict'.format(name))

            names.append(name)

        # Don't subscribe until everything has been initialized.
        rospy.Subscriber('joy', sensor_msgs.msg.Joy, self.joy_callback)

    def joy_callback(self, msg: sensor_msgs.msg.Joy) -> None:
        for name, command in self.scales.items():
            command.run(msg)
        for command in self.commands:
            command.run(msg)


if __name__ == "__main__":
    try:
        rospy.init_node('joy_teleop')
        jt = JoyTeleop()
        rospy.spin()
    except JoyTeleopException as e:
        rospy.logerr(e)
        import traceback
        print(traceback.format_exc())
        pass
    except rospy.ROSInterruptException:
        import traceback
        print(traceback.format_exc())
        pass
    except KeyboardInterrupt:
        import traceback
        print(traceback.format_exc())
        pass