#!/usr/bin/env python
from bppose.utils import Timer,Parse_Args
from bppose.publishing import Publisher_ROS
from bppose.positioning import Position
import time
import os
import scipy
ros_namespace = os.environ['ROS_NAMESPACE']
debug=True
def process_and_publish():
    port,ekf,debug = Parse_Args()
    position = Position(port=port,n_sensors=4,ekf=ekf,debug=debug)
    publish = Publisher_ROS()
    timer = Timer()
    position.lock_lighthouse(100)

    while publish.is_alive()==True:
        position.process_pose()

        # position.find_lighthouse()
        lighthouse = position.get_lighthouse()
        #publish.publish_transform(lighthouse, ros_namespace+"/"+"Lighthouse Frame", ros_namespace+"/"+"World Frame")
        publish.publish_transform(lighthouse, "Lighthouse Frame", ros_namespace+"/"+"World Frame")
        position.find_object()
        object = position.get_object()
        publish.publish_transform(object,ros_namespace+"/"+"Base Frame","Lighthouse Frame")
        #publish.publish_transform(object,"Lighthouse Frame", ros_namespace+"/"+"Base Frame")
        # publish.publish_pose([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],"Base Frame")
        absolute_object_pose = position.get_object_world_frame()
        publish.publish_pose(absolute_object_pose,ros_namespace+"/"+"World Frame")
        time.sleep(0.02)
        if debug==True:
            timer.update(show=True)

if __name__=="__main__":
    process_and_publish()
