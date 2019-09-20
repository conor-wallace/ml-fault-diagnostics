import psutil
import rospy
import std_msgs
from network_faults.msg import Network
import subprocess, re

hostname = "192.168.1.152"

rospy.init_node('agent1_stats', anonymous=True)
network_stats = rospy.Publisher("/network_stats", Network, queue_size=1, tcp_nodelay=True)

rate = rospy.Rate(10)
rate.sleep()

network_stats_pltd = Network()
prev = 0

while not rospy.is_shutdown():
    output = subprocess.Popen(["sudo", "ping", "-c", "1","-i","0.01",hostname],stdout = subprocess.PIPE).communicate()[0]
    packetloss = re.findall(r"[0-9]*%", output.decode('utf-8'))
    delay = re.findall(r"[0-9]+\.[0-9]+/([0-9]+\.[0-9]+)/[0-9]+\.[0-9]+/[0-9]+\.[0-9]+", output.decode('utf-8'))

    probability = re.findall(r"\d[0-9]+", str(packetloss))
    if len(probability) > 0:
        print(re.sub(r'%', '', packetloss[0]))
        network_stats_pltd.packet_loss = float(re.sub(r'%', '', packetloss[0]))
    if len(delay) > 0:
        print(delay[0])
        network_stats_pltd.time_delay = float(delay[0])
    network_stats_pltd.header.stamp = rospy.Time.now()

    network_stats.publish(network_stats_pltd)
    rate.sleep()
