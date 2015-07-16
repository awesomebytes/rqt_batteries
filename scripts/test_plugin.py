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
"""
Created on 6/8/15

@author: sampfeiffer

test_plugin.py contains...
"""
__author__ = 'sampfeiffer'

import rospy
import random
from power_msgs.msg import BatteryState

if __name__ == "__main__":
    rospy.init_node('test_rqt_battery')
    pub1 = rospy.Publisher('/battery_status', BatteryState, queue_size=1)
    pub2 = rospy.Publisher('/other_battery', BatteryState, queue_size=1)

    while not rospy.is_shutdown():
        for i in range(0, 100, 10):
            bs = BatteryState()
            bs.name = "Battery 1"
            bs.charge_level = float(i)
            bs.is_charging = random.choice([True, False])
            bs.remaining_time = rospy.Duration(i)
            rospy.loginfo("Publishing: " + str(bs))
            pub1.publish(bs)
            bs2 = BatteryState()
            bs2.name = "ElectronHolder"
            bs2.charge_level = float(i)
            bs2.is_charging = random.choice([True, False])
            bs2.remaining_time = rospy.Duration(i + 1000) # So we show some hours
            rospy.loginfo("Publishing: " + str(bs2))
            pub2.publish(bs2)
            rospy.sleep(1.0)