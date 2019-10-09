import rospy
import std_msgs
from network_faults.msg import Network

rospy.init_node('network', anonymous=True)
network_stats = rospy.Publisher("/network_stats", Network, queue_size=1, tcp_nodelay=True)

rate = rospy.Rate(10)
rate.sleep()

network_stats_pltd = Network()
count = 1

while not rospy.is_shutdown():
    network_stats_pltd.header.stamp = rospy.Time.now()
    network_stats_pltd.packet_loss = count
    count += 1
    network_stats.publish(network_stats_pltd)
    rate.sleep()
