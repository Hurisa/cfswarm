#!/usr/bin/env python

from cfswarm.srv import getAverageMapValue,getAverageMapValueResponse
import rospy
import numpy as np


def handle_getAverageMapValue(req):
    values=[]

    for x in range(-5,5):
    	for y in range(-5,5):
            if req.posX+x>0 and req.posX+x<map.shape[0] and req.posY+y>0 and req.posY+y<map.shape[1]:
    	       values.append(map[req.posX+x][req.posY+y])
    
    S=sum(values)/len(values)
 
    return getAverageMapValueResponse(S)
    # To Do:
    # check the limits of the sensed area 

def getAverageMapValue_server():
    rospy.init_node('getAverageMapValue_server')
    global map
    map=np.asarray(rospy.get_param("/matrix"))
    s = rospy.Service('getAverageMapValue',getAverageMapValue, handle_getAverageMapValue)
    print "Ready to retrieve map data."
    rospy.spin()

if __name__ == "__main__":

    getAverageMapValue_server()
