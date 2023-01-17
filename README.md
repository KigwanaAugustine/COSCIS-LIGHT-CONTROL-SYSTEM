# COSCIS-LIGHT-CONTROL-SYSTEM
Designing an embedded light control system for COSCIS , Makerere University

Light Control and Monitoring system in Makerere University
Makerere University is struggling with the high utility bills and especially those accumulated from the
wastage use of power through lighting. It has been observed that most rooms are always lit even when
there is nobody in those rooms. In order to limit the wastage, you have been requested to come up with a
system, which will help in the control and monitoring of power usage in 4 rooms and 1 security light in the
College of Computing and Information Sciences (CoCIS). The security lights should only be switched on
during the periods of darkness ONLY. At exactly 9:30 PM, all lights from the rooms should be switched off
unless motion is detected in such rooms. If motion is detected after 9:30, an alarm is raised for a period of
10 seconds, after which the lights are switched off. The lights will be switched on either using the switches
attached to the lighting system or by using a Bluetooth control system, accessed via a mobile app. If during
the time the lights are supposed to be off, motion is detected, the system will automatically switch on the
lights for that room and an alarm sounded, and door closed until when opened using the mobile app. If the
light intensity is low, lights during the day will be switched on if motion is detected. To conserve power, a
given room may be configured to only be switched on using the mobile phone app. This configuration
overtakes the automatic configuration. Each room is represented by an LED and each college will start
with controlling 4 rooms. For every 2 minutes, the bulbs consume 1KWh. The mobile app should also have
a monitoring interface for
o Viewing how much time lights have taken on a given date
o The room that consumes the highest amount of power
o Which room is occupied at a given time
o etc
The system will use an RTC clock module, which keeps track of the date and time to enable easy generation of
reports based on dates and time. At a given time, someone can request to see reports or perform any command of
choice using a serial interface. For one to access the serial interface control interface, they should be registered in
the system and hence should be authenticated. This password should be saved in EEPROM.
