#!/usr/bin/env python
import rospy

import sys
import matplotlib.pyplot as plt
import numpy as np
from network_faults.msg import Network, Velocity

txrx_pl = 0
txrx_td = 0
offset = 0
path_data = []
count = 0
pl_percent = 0.0
stop = 0

def gotdata(txrx):
    global offset, txrx_pl,txrx_td, path_data, count, pl_percent, stop

    if not stop:
        txrx_pl = txrx.packet_loss
        if offset == 0:
            offset = txrx_pl
        seq_val = txrx_pl - offset
        print("seq, count: %s %s" % (seq_val, count))
        if count != seq_val:
            dropped_packets = [1] * (seq_val-count)
            path_data.extend(dropped_packets)
            count = seq_val
        else:
            path_data.append(0)
            count += 1

        print("Packet Loss Percentage: %s" % (float(pl_percent)/float(count)))
        print(len(path_data))

        if len(path_data) == 200:
            sum = 0
            for i in range(len(path_data)):
                sum += int(path_data[i])
            print("Mean Percentage of Packet Loss: %s" % (float(sum/len(path_data))))
            stop = 1

rospy.init_node('GetData', anonymous=True)
network_sub = rospy.Subscriber("network_stats", Network, gotdata)

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    if stop:
        print("converting data to csv")
        path_data = np.asarray(path_data)
        path_t = np.arange(len(path_data))

        plt.plot(path_t[:], path_data[:])
        plt.xlabel('t')
        plt.ylabel('Packet Loss %')
        plt.title('Packet Loss CDF')
        plt.legend()
        plt.show()

        sys.exit(1)

    rate.sleep()
