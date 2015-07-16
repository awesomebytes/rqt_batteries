#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# rqt_batteries: wrap_battery.py
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

from rqt_robot_dashboard.battery_dash_widget import BatteryDashWidget

class WrappedBattery(BatteryDashWidget):
    """
    Dashboard widget to display batteries state.
    """
    def __init__(self, context, name="Battery", callback_on_double_click=None):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        """
        icon_names = ['ic-battery-0.svg', 'ic-battery-20.svg', 'ic-battery-40.svg',
                 'ic-battery-60-green.svg', 'ic-battery-80-green.svg', 'ic-battery-100-green.svg']
        icons = []
        for icon in icon_names:
            icons.append([icon])

        charge_icon_names = ['ic-battery-charge-0.svg', 'ic-battery-charge-20.svg', 'ic-battery-charge-40.svg',
                        'ic-battery-charge-60-green.svg', 'ic-battery-charge-80-green.svg', 'ic-battery-charge-100-green.svg']
        charge_icons = []
        for charge_icon in charge_icon_names:
            charge_icons.append([charge_icon])

        icon_paths = []
        icon_paths.append(['rqt_batteries', 'images'])
        self._wrapped_battery_name = name
        self._callback_on_double_click = callback_on_double_click
        super(WrappedBattery, self).__init__(name=name,
                                                  icons=icons, charge_icons=charge_icons,
                                                  icon_paths=icon_paths,
                                                  suppress_overlays=True)


        self.update_perc(0)

    def set_power_state_perc(self, percentage, charging, name=None, tooltip_text=None):
        """
        """
        if name is not None:
            self._name = name
        self.update_perc(percentage)
        if tooltip_text:
            self.setToolTip(tooltip_text)
        self.set_charging(charging)


    def mouseDoubleClickEvent(self, event):
        """
        Callback when there is a double click on the icon
        """
        if self._callback_on_double_click:
            self._callback_on_double_click()
