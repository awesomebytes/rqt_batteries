#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# rqt_batteries: batteries_dashboard.py
#
# Copyright (c) 2015 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Authors:
#   * Sammy Pfeiffer

import rospy

from rqt_robot_dashboard.dashboard import Dashboard

from python_qt_binding.QtCore import QSize

from power_msgs.msg import BatteryState
from .wrap_battery import WrappedBattery
from .settings_dialog import SettingsDialog
import rospkg


class BatteriesDashboard(Dashboard):
    """
    Dashboard for Batteries

    :param context: the plugin context
    :type context: qt_gui.plugin.Plugin
    """
    def setup(self, context):
        self.name = 'Batteries Dashboard'
        self._rospack = rospkg.RosPack()

        self._last_dashboard_message_time = rospy.Time.now()
        self._widget_initialized = False
        self._subscriber = None
        self._currenttopic = '/battery_state'

        self._subscribe(self._currenttopic)

        self._widget = WrappedBattery(self.context, callback_on_double_click=self.trigger_configuration)
        self._widget.set_power_state_perc(0.0, False, name="Battery", tooltip_text="No info received yet on '/battery_state'.")

        self._widget_initialized = True


    def get_widgets(self):
        return [[self._widget]]

    def dashboard_callback(self, msg):
        """
        callback to process messages
        :type msg: BatteryState
        """
        if not self._widget_initialized:
            return

        # Throttling to 1Hz the update of the widget whatever the rate of the topics is, maybe make it configurable?
        if (rospy.Time.now() - self._last_dashboard_message_time) < rospy.Duration(1.0):
            return
        self._last_dashboard_message_time = rospy.Time.now()

        # Update widget
        tooltip_text = msg.name + ": " + str(msg.charge_level) + "% battery "
        if msg.remaining_time != rospy.Duration(0):
            hours = msg.remaining_time.secs / 60 / 60
            minutes = (msg.remaining_time.secs / 60) % 60
            if hours > 0:
                tooltip_text += str(hours) + "h "
            tooltip_text += str(minutes) + "m "
            if msg.is_charging:
                tooltip_text += "left to fully charge"
            else:
                tooltip_text += "left"
        self._widget.set_power_state_perc(msg.charge_level, msg.is_charging, name=msg.name, tooltip_text=tooltip_text)
        rospy.loginfo(tooltip_text)


    def _subscribe(self, topic):
        rospy.loginfo("Subscribing to '" + topic + "'")
        if self._subscriber:
            self._subscriber.unregister()
        self._subscriber = rospy.Subscriber(topic, BatteryState, self.dashboard_callback, queue_size=1)
        self._currenttopic = topic


    def shutdown_dashboard(self):
        if self._subscriber:
            self._subscriber.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("batt_topic", self._currenttopic)

    def restore_settings(self, plugin_settings, instance_settings):
        batt_topic_setting = instance_settings.value("batt_topic")
        #print "batt_topic_setting: " + str(batt_topic_setting) + "\ntype: " + str(type(batt_topic_setting))
        # seems the strings are saved as unicode by Qt, but just in case I'll leave also str as acceptable
        if type(batt_topic_setting) == str or type(batt_topic_setting) == unicode:
            self._currenttopic = batt_topic_setting
            self._subscribe(self._currenttopic)
        else:
            print "No batt_topic_setting"

    def trigger_configuration(self):
        """
        Trigger the window to choose the topic to show the battery status,
        called when the battery icon is double clicked
        """
        topics = [t for t in rospy.get_published_topics() if t[1] == 'power_msgs/BatteryState']
        topics.sort(key=lambda tup: tup[0])
        dialog = SettingsDialog(topics, self._rospack)
        topic = dialog.query(self._currenttopic)
        if topic != self._currenttopic:
            self._subscribe(topic)
