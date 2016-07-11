#!/usr/bin/env python


"""

Protocal, 20 bytes

+---------+---+----+----------------------------------+--------+---+---+----+
|  Header |Res|Type|         Payload 12 bytes         |Checksum|Res|Res|End |
+---------+----+---+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+-_-+----+
|0x55|0xAA|ZZ |0x01|         1 char | xxx             | 0xZZ   |ZZ |ZZ |0x0A|
+----+----+---+----+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+---+----+
Type:
    0x82: Servo control

Payload:
    Servo value: value of degree, 1 byte

Checksum:
   checksum of type and payload

"""
from chassis_dev import ChassisDev
from protocol import Protocol
from std_msgs.msg import Byte
class ServoCtrl(ChassisDev):

    TYPE = 0x82
    # 12 bytes data package format in struct, only first byte valid
    pack_pmt = "4B2f"


    def __init__(self, serial_dev):
        self.protocol = Protocol(self.TYPE, self.pack_pmt)

        self.serial_dev = serial_dev

        ChassisDev.__init__(self, "Servo",
                            sub_topic = "/robot_chassis/Servo",
                            sub_msg_class = Byte)

    def start_subscribe(self):

        def cb_func(pwm_data):
            self.dev_write(self.pwm_to_encoder(pwm_data.data))

        ChassisDev.start_sub(self, cb_func)

    def pwm_to_encoder(self, pwm):
        """ construct the payload
        """
        return [pwm, 0,0,0,0,0]

    def dev_write(self, data):
        """ write data to serial
        """
        self.serial_dev.write(self.protocol.encode(data))
        print "Servo: Write to serial", data
