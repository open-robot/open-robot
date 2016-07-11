#!/usr/bin/env python


"""

Protocal, 20 bytes

+---------+---+----+----------------------------------+--------+---+---+----+
|  Header |Res|Type|         Payload 12 bytes         |Checksum|Res|Res|End |
+---------+----+---+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+-_-+----+
|0x55|0xAA|ZZ |0x01|         3 float                  | 0xZZ   |ZZ |ZZ |0x0A|
+----+----+---+----+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+---+----+
Type:
    0x81: 3 Wheels Spped

Payload:
    Speed * 3: speed in float, 4 bytes

Checksum:
   checksum of type and payload

"""
from chassis_dev import ChassisDev
from protocol import Protocol
from smart_battery_msgs.msg import SmartBatteryStatus
from geometry_msgs.msg import Twist

class MoveCtrl(ChassisDev):

    TYPE = 0x81
    # 12 bytes data package format in struct
    pack_pmt = "3f"

    # Chassis
    VEL_TRANS = 329.4531
    L = 0.105

    def __init__(self, serial_dev):
        self.protocol = Protocol(self.TYPE, self.pack_pmt)

        self.serial_dev = serial_dev

        ChassisDev.__init__(self, "move_ctrl",
                            sub_topic = "cmd_vel",
                            sub_msg_class = Twist)

    def start_subscribe(self):

        def cb_func(twist_data):
            self.dev_write(self.vel_to_encoder(twist_data.linear.x,
                                          twist_data.linear.y,
                                          twist_data.angular.z))

        ChassisDev.start_sub(self, cb_func)

    def vel_to_encoder(self, linear_x, linear_y, angular_z):
        """ calculate speed for three wheels
        """
        wheel_left = ( linear_x * 0.8660254 - linear_y * 0.5 - \
                           angular_z * self.L ) * self.VEL_TRANS
	wheel_back =( linear_y - angular_z * self.L ) * self.VEL_TRANS
	wheel_right=(-linear_x * 0.8660254 -linear_y * 0.5 - \
                          angular_z*self.L)*self.VEL_TRANS;
        return [wheel_left, wheel_right, wheel_back]

    def dev_write(self, data):
        """ write data to serial
        """
        self.serial_dev.write(self.protocol.encode(data))
        print "Write to serial", data
