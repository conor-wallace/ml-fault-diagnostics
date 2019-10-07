#!/usr/bin/env python
import rospy

import sys
import matplotlib.pyplot as plt
import numpy as np
from network_faults.msg import Network, Velocity

txrx_pl = 0
txrx_td = 0
path_data = []
count = 1.0
pl_percent = 0.0
stop = 0

def gotdata(txrx):
    global txrx_pl,txrx_td, path_data, count, pl_percent, stop

    if not stop:
        txrx_pl = txrx.packet_loss
        txrx_td = txrx.time_delay
        pl_percent += txrx_pl
        path_data.append([count, (pl_percent/count)])
        count += 1

        print("Packet Loss: %s, Time Delay: %s" % (txrx_pl, txrx_td))
        print("Packet Loss Percentage: %s" % (pl_percent/count))
        print(count)

        if len(path_data) == 200:
            stop = 1

rospy.init_node('GetData', anonymous=True)
network_sub = rospy.Subscriber("network_stats", Network, gotdata)

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    if stop:
        print("converting data to csv")
        path_data = np.asarray(path_data)

        plt.plot(path_data[:, 0], path_data[:, 1])
        plt.xlabel('t')
        plt.ylabel('Packet Loss %')
        plt.title('Packet Loss CDF')
        plt.legend()
        plt.show()

        sys.exit(1)

    rate.sleep()
