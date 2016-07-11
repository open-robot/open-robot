#!/usr/bin/env python


"""
Protocal, 20 bytes

+---------+---+----+----------------------------------+--------+---+---+----+
|  Header |Res|Type|         Payload 12 bytes         |Checksum|Res|Res|End | 
+---------+----+---+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+-_-+----+
|0x55|0xAA|ZZ |0x01|                                  | 0xZZ   |ZZ |ZZ |0x0A|
+----+----+---+----+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+---+----+
Type:
    0x01: odom
    0x02: speed
    0x03: battery
    0x04: IMU

Checksum:
   checksum of type and payload

"""

import struct

class Protocol():
    """ Protocol definition for communication between MCU and host
    """
    HEADER0 = 0x55
    HEADER1 = 0xAA
    END = 0x0A

    def __init__(self, cmd_type, cmd_fmt):
        """
        Protocol payload 12 bytes, cmd_fmt must have the same size
        """
        self.cmd_type = cmd_type
        self.cmd_fmt = cmd_fmt


    def decode(self, bin_pack):

        data = struct.unpack("20B",bin_pack)

        if data[3] != self.cmd_type:
            return

        # data check
        if data[0] != self.HEADER0 or data[1] != self.HEADER1 \
                or (sum(data[3:16]) % 0x100) != data[16] \
                or data[19] != self.END:
            raise struct.error("Checksum error")
            return

        # unpack data
        unpack_data = struct.unpack(
            "4B"+self.cmd_fmt+"4B", bin_pack)
        return unpack_data[4:-4]

    def encode(self, data_list):

        checksum = sum(struct.unpack("1B12B",struct.pack("1B", self.cmd_type) +struct.pack(self.cmd_fmt, *data_list)))
        #checksum = sum(struct.unpack("12B",struct.pack(self.cmd_fmt, *data_list)))
        checksum = (checksum )% 0x100

        return struct.pack("4B", self.HEADER0, self.HEADER1, 0x00, self.cmd_type) \
            + struct.pack(self.cmd_fmt, *data_list) \
            + struct.pack("4B", checksum, 0x00, 0x00, self.END)
