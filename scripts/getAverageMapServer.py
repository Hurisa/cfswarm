#!/usr/bin/env python
from cfswarm.srv import getAverageMapValue,getAverageMapValueResponse
import rospy
import numpy as np
import math

class mapServer_class:

    def __init__(self):
   
        self.map=np.asarray(rospy.get_param("/matrix"))      
        self.s = rospy.Service('getAverageMapValue',getAverageMapValue, self.handle_getAverageMapValue)
        print "Ready to retrieve map data."

    def handle_getAverageMapValue(self,req):
        values=[]

        for x in range(-5,5):
            for y in range(-5,5):
                if req.posX+x>0 and req.posX+x<self.map.shape[0] and req.posY+y>0 and req.posY+y<self.map.shape[1]:
                   values.append(self.map[req.posX+x][req.posY+y])
        
        S=sum(values)/len(values)     
        return getAverageMapValueResponse(S)

    #def getAverageMapValue_server(self):
       
        
       

def main():
  mapServer = mapServer_class()
  #mapServer.getAverageMapValue_server()
  rospy.init_node('getAverageMapValue_server')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
