#!/usr/bin/env python


"""
Protocal, 20 bytes

+---------+---+----+----------------------------------+--------+---+---+----+
|  Header |Res|Type|         Payload 12 bytes         |Checksum|Res|Res|End |
+---------+----+---+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+-_-+----+
|0x55|0xAA|ZZ |0x03|         2 Int 2 short            | 0xZZ   |ZZ |ZZ |0x0A|
+----+----+---+----+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+---+----+
Type:
    0x03: Battery

Payload:
   Voltage: Battery voltage(V), float 32 bit
   Rate : Discharge current(A), float 32 bit
   Charge: Charge state, short 16 bit
   precentage: Battery cap, short 16 bit

Checksum:
   checksum of type and payload

"""
from chassis_dev import ChassisDev
from protocol import Protocol
from smart_battery_msgs.msg import SmartBatteryStatus


class Battery(ChassisDev):

    TYPE = 0x03
    # 12 bytes data package format in struct
    pack_pmt = "2f2H"

    def __init__(self):
        self.protocol = Protocol(self.TYPE, self.pack_pmt)

        ChassisDev.__init__(self, "smart_battery",
                            pub_topic = "smart_battery",
                            pub_msg_class = SmartBatteryStatus)
        self.battery_status = SmartBatteryStatus()

        ChassisDev.start_pub(self)

    def data_handler(self, bin_data_pack):
        try:
            voltage, rate, _,_ = \
                self.protocol.decode(bin_data_pack)
        except e:
            print e
            return

        self.battery_status.voltage = voltage
        self.battery_status.rate = rate
        self.battery_status.charge_state = 0
        if voltage > 2000:
            self.battery_status.percentage = 100 - (2520 - voltage)/4
        else:
            self.battery_status.percentage = 100 - (1260 - voltage)/1.6
        ChassisDev.pub_data_update(self, self.battery_status)
