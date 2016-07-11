#!/usr/bin/env python


"""
Protocol, 20 bytes

+---------+---+----+----------------------------------+--------+---+---+----+
|  Header |Res|Type|         Payload 12 bytes         |Checksum|Res|Res|End |
+---------+----+---+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+-_-+----+
|0x55|0xAA|ZZ |0x02|         3 float                  | 0xZZ   |ZZ |ZZ |0x0A|
+----+----+---+----+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+---+----+
Type:
    0x02: the speed of wheel

Payload:
   Speed[3]: the speed of wheel, 4 bytes *3

Checksum:
   checksum of type and payload

"""
from chassis_dev import ChassisDev
from protocol import Protocol
from std_msgs.msg import Float32MultiArray

class Speed(ChassisDev):

    TYPE = 0x02
    # 12 bytes data package format in struct
    pack_pmt = "3f"

    def __init__(self):
        self.protocol = Protocol(self.TYPE, self.pack_pmt)

        ChassisDev.__init__(self, "Speed",
                            pub_topic = "/speed_wheel",
                            pub_msg_class = Float32MultiArray,
                            pub_rate = 20)
        self.speeds = Float32MultiArray()
        self.speeds.data = [0,0,0]
        ChassisDev.pub_data_update(self, self.speeds)
        ChassisDev.start_pub(self)

    def data_handler(self, bin_data_pack):


        wheel0, wheel1, wheel2  = \
                self.protocol.decode(bin_data_pack)

        self.speeds.data = [wheel0, wheel1, wheel2]
        ChassisDev.pub_data_update(self, self.speeds)
