#!/usr/bin/env python
# Copyright (C) 2011-2014 Swift Navigation Inc.
# Contact: Fergus Noble <fergus@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.observation import *

import argparse
import math
import os
import numpy as np
import datetime

obs_count = 0
gps_tow  = 0.0
gps_week = 0
name = Str('Rover')
rinex_file = None

_obs_table_list = List()
obs = Dict()  

recording = Bool(True)

def main():
    parser = argparse.ArgumentParser(description="Swift Navigation SBP Example.")
    parser.add_argument("-p", "--port",
                        default=['/dev/ttyUSB0'], nargs=1,
                        help="specify the serial port to use.")
    args = parser.parse_args()

    # Open a connection to Piksi using the default baud rate (1Mbaud)
    with PySerialDriver(args.port[0], baud=1000000) as driver:
        with Handler(Framer(driver.read, None, verbose=True)) as source:
            try:
                for msg, metadata in source.filter(SBP_MSG_OBS):
                    obs_packed_callback(msg)
            except KeyboardInterrupt:
                pass

def rinex_save():
    if recording:
        if rinex_file is None:
            # If the file is being opened for the first time, write the RINEX header
            rinex_file = open(name+t.strftime("-%Y%m%d-%H%M%S.obs"),  'w')
            header = """     2.11           OBSERVATION DATA    G (GPS)             RINEX VERSION / TYPE
pyNEX                                   %s UTC PGM / RUN BY / DATE
                                                            MARKER NAME
                                                            OBSERVER / AGENCY
                                                            REC # / TYPE / VERS
                                                            ANT # / TYPE
   808673.9171 -4086658.5368  4115497.9775                  APPROX POSITION XYZ
        0.0000        0.0000        0.0000                  ANTENNA: DELTA H/E/N
     1     0                                                WAVELENGTH FACT L1/2
     3    C1    L1    S1                                    # / TYPES OF OBSERV
%s%13.7f     GPS         TIME OF FIRST OBS
                                                            END OF HEADER
""" % (
            datetime.datetime.utcnow().strftime("%Y%m%d %H%M%S"),
            t.strftime("  %Y    %m    %d    %H    %M"), t.second + t.microsecond * 1e-6,
        )
            rinex_file.write(header)
        prns = list(obs.iterkeys())
        rinex_file.write("%s %10.7f  0 %2d" % (t.strftime(" %y %m %d %H %M"),
                                               t.second + t.microsecond*1e-6,
                                               len(prns)))
        while len(prns) > 0:
            prns_ = prns[:12]
            prns = prns[12:]
            for prn in prns_:
                rinex_file.write('G%2d' % (prn))
            rinex_file.write('   ' * (12 - len(prns_)))
            rinex_file.write('\n')
            
        for prn in list(obs.iterkeys()):
            # G    3 C1C L1C D1C
            rinex_file.write("%14.3f  " % obs[prn][0])
            rinex_file.write("%14.3f  " % obs[prn][1])
            rinex_file.write("%14.3f  \n" % obs[prn][2])
            
        rinex_file.flush()

def update_obs():
    _obs_table_list = [(prn,) + obs for prn, obs in sorted(obs.items(), key=lambda x: x[0])]

def obs_packed_callback(sbp_msg):
    tow = sbp_msg.header.t.tow
    wn = sbp_msg.header.t.wn
    seq = sbp_msg.header.n_obs

    tow = float(tow) / 1000.0

    total = seq >> 4
    count = seq & ((1 << 4) - 1)

    # Confirm this packet is good.
    # Assumes no out-of-order packets
    if (count == 0):
        gps_tow = tow;
        gps_week = wn;
        prev_obs_total = total
        prev_obs_count = 0;
        obs = {}
    elif (gps_tow            != tow    or
          gps_week           != wn     or
          prev_obs_count + 1 != count  or
          prev_obs_total     != total):
        print "We dropped a packet. Skipping this observation sequence"
        prev_obs_count = -1;
        return;
    else:
        prev_obs_count = count
    # Save this packet
    # See sbp_piksi.h for format
    for o in sbp_msg.obs:
        prn = o.sid.sat
        if ((o.sid.code == 0)):
            prn += 1
        obs[prn] = (float(o.P) / 1e2,
                    float(o.L.i) + float(o.L.f) / (1<<8),
                    float(o.cn0) / 4)
    if (count == total - 1):
        t = datetime.datetime(1980, 1, 6) + \
        datetime.timedelta(weeks=gps_week) + \
        datetime.timedelta(seconds=gps_tow)
        update_obs()
        rinex_save()
    return

if __name__ == "__main__":
    main()
