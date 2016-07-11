#!/usr/bin/env python


"""
Protocol, 20 bytes

+---------+---+----+----------------------------------+--------+---+---+----+
|  Header |Res|Type|         Payload 12 bytes         |Checksum|Res|Res|End |
+---------+----+---+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+-_-+----+
|0x55|0xAA|ZZ |0x01|         3 Int                    | 0xZZ   |ZZ |ZZ |0x0A|
+----+----+---+----+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+---+----+
Type:
    0x01: Odom counter

Payload:
   Counter[3]: Odom counter, 4 bytes *3

Checksum:
   checksum of type and payload

"""
from chassis_dev import ChassisDev
from protocol import Protocol
from smart_battery_msgs.msg import SmartBatteryStatus
from std_msgs.msg import Float32MultiArray

class Odom(ChassisDev):

    TYPE = 0x01
    # 12 bytes data package format in struct
    pack_pmt = "3i"

    def __init__(self):
        self.protocol = Protocol(self.TYPE, self.pack_pmt)

        ChassisDev.__init__(self, "Chassis",
                            pub_topic = "/encoder_cnts",
                            pub_msg_class = Float32MultiArray,
                            pub_rate = 20)
        self.counters = Float32MultiArray()

        ChassisDev.start_pub(self)

    def data_handler(self, bin_data_pack):


        counter0, counter1, counter2  = \
                self.protocol.decode(bin_data_pack)

        self.counters.data = [counter0, counter1, counter2]
        ChassisDev.pub_data_update(self, self.counters)
