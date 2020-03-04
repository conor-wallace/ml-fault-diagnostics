import psutil
import rospy
import std_msgs
from network_faults.msg import Network
import subprocess, re

hostname = "192.168.1.180"

rospy.init_node('agent1_stats', anonymous=True)
network_stats = rospy.Publisher("/network_stats", Network, queue_size=1, tcp_nodelay=True)

rate = rospy.Rate(10)
rate.sleep()

network_stats_pltd = Network()
prev = 0

while not rospy.is_shutdown():
    output = subprocess.Popen(["sudo", "ping",hostname, "-c", "1", "-i", "0.1"],stdout = subprocess.PIPE).communicate()[0]
    delay = re.findall(r"[0-9]+\.[0-9]+/([0-9]+\.[0-9]+)/[0-9]+\.[0-9]+/[0-9]+\.[0-9]+", output.decode('utf-8'))

    probability = re.findall(r"\d[0-9]+", str(packetloss))
    print(str(packetloss))
    if "100" in str(packetloss):
        print(100.0)
        network_stats_pltd.packet_loss = 100.0
    elif ("100" not in str(packetloss)) and ("0" in str(packetloss)):
        print(0.0)
        network_stats_pltd.packet_loss = 0.0
    else:
        print("not working")
    if len(delay) > 0:
        print(delay[0])
        network_stats_pltd.time_delay = float(delay[0])
    network_stats_pltd.header.stamp = rospy.Time.now()

    network_stats.publish(network_stats_pltd)
    rate.sleep()
