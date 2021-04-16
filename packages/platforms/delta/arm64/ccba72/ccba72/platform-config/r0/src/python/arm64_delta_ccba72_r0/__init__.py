#!/usr/bin/python

from onl.platform.base import *
from onl.platform.delta import *

class OnlPlatform_arm64_delta_ccba72_r0(OnlPlatformDelta):
    PLATFORM='arm64-delta-ccba72-r0'
    MODEL="CCBA72"
    SYS_OBJECT_ID=".72"

    def baseconfig(self):
        # Insert platform drivers
        self.insmod("arm64-delta-ccba72-cpld.ko")

        ########### initialize I2C bus 1 ###########
        self.new_i2c_devices (
            [
                # INA3221
                ('ina3221', 0x40, 0),

                # CPLD
                ('ccba72_cpld', 0x42, 0),

                # Temperature devices
                ('tmp1075', 0x4a, 0),
                ('tmp1075', 0x4b, 0),
            ]
        )

        #sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        #ioctl(sock.fileno(), 0x8949, struct.pack('16s3H', 'ma1', 0x01, 0x1b, 0x848b))
        #ioctl(sock.fileno(), 0x8949, struct.pack('16s3H', 'ma1', 0x01, 0x0,  0x9140))

        return True

