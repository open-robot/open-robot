#!/usr/bin/env python


"""
Protocal, 20 bytes

+---------+---+----+----------------------------------+--------+---+---+----+
|  Header |Res|Type|         Payload 12 bytes         |Checksum|Res|Res|End |
+---------+----+---+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+-_-+----+
|0x55|0xAA|00 |0x04|         3 Float                  | 0xZZ   |ZZ |ZZ |0x0A|
+----+----+---+----+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+---+----+
Type:
    0x04: IMU
Res:
    0x00: ACC
    0x01: GYRO

Payload:
   x:  float 32 bit
   y:  float 32 bit
   z:  float 32 bit

Checksum:
   checksum of type and payload

"""
from chassis_dev import ChassisDev
from protocol import Protocol
from sensor_msgs.msg import Imu
import struct
import rospy

class MPU(ChassisDev):

    TYPE = 0x04
    # 12 bytes data package format in struct
    pack_pmt = "3f"
    TYPE_ACC = 0x00
    TYPE_GYRO = 0x01

    def __init__(self):
        self.protocol = Protocol(self.TYPE, self.pack_pmt)

        ChassisDev.__init__(self, "chassis",
                            pub_topic = "imu/data_raw",
                            pub_rate = 10,
                            pub_msg_class = Imu)
        self.imu = Imu()
        ChassisDev.start_pub(self)

    def data_handler(self, bin_data_pack):
        if struct.unpack("20B", bin_data_pack)[2] == self.TYPE_GYRO:
            try:
                x, y, z = \
                    self.protocol.decode(bin_data_pack)
                self.imu.angular_velocity.x = x
                self.imu.angular_velocity.y = y
                self.imu.angular_velocity.z = z
                self.imu.header.frame_id = 'base_footprint'
                self.imu.header.stamp = rospy.Time.now()
                ChassisDev.pub_data_update(self, self.imu)
            except Exception as e:
                print e
                return
        else:
            try:
                x, y, z = \
                    self.protocol.decode(bin_data_pack)
                self.imu.linear_acceleration.x = x
                self.imu.linear_acceleration.y = y
                self.imu.linear_acceleration.z = z
            except e:
                print e
                return
