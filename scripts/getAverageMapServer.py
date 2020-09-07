#!/usr/bin/env python
from cfswarm.srv import getAverageMapValue,getAverageMapValueResponse
from cfswarm.msg import arraypoints
from std_msgs.msg import Bool
import rospy
import numpy as np
import math

class mapServer_class:

    def __init__(self):
   
        self.points=np.asarray(rospy.get_param("/points")) 
        self.pointsFound=np.array([[]])
        self.discoveryTimes=np.array([])
        self.sr=rospy.get_param("/sensingRadius");
        self.s = rospy.Service('getAverageMapValue',getAverageMapValue, self.handle_getAverageMapValue)
        self.pubPointsFound=rospy.Publisher('/pointsFound', arraypoints, queue_size=10)
        self.pubpointsLeft=rospy.Publisher('/pointsLeft', arraypoints, queue_size=10)
        self.subResetMapt=rospy.Subscriber('/resetMap', Bool, self.resetPoints)
        print "Ready to retrieve map data."
                            

    def handle_getAverageMapValue(self,req):

        withinX=np.asarray(np.where((self.points[:,0]>=req.posX-self.sr/2) & (self.points[:,0]<=req.posX+self.sr/2)))
        withinY=np.asarray(np.where((self.points[:,1]>=req.posY-self.sr/2) & (self.points[:,1]<=req.posY+self.sr/2)))
     
        if withinX.shape[1]>0 and withinY.shape[1]>0:
            discoveries=np.intersect1d(withinY, withinX)   
            #print self.points[discoveries,:]         
            if discoveries.shape[0]>0:                
                if self.pointsFound.shape[1]==0:
                    self.pointsFound=self.points[discoveries,:]
                    self.discoveryTimes=np.ones((discoveries.shape[0], 1))*rospy.get_time()
                else:
                    #for x in range(0,discoveries.shape[0]):
                    self.pointsFound=np.append(self.pointsFound, self.points[discoveries,:],axis=0)
                    self.discoveryTimes=np.append(self.discoveryTimes, np.ones((discoveries.shape[0], 1))*rospy.get_time())

                self.points=np.delete(self.points, discoveries, 0)     
                S=discoveries.shape[0]
                
            else:
                S=0
        else:
            S=0
 
        if self.pointsFound.shape[1]>0:
            #print "sending points update"
            msg=arraypoints()
            msg.x=self.pointsFound[:,0].tolist()
            msg.y=self.pointsFound[:,1].tolist()
            msg.c=self.pointsFound[:,2].tolist()
            msg.t=self.discoveryTimes
            #print msg
            self.pubPointsFound.publish(msg)

            msgl=arraypoints()
            msgl.x=self.points[:,0].tolist()
            msgl.y=self.points[:,1].tolist()
            self.pubpointsLeft.publish(msgl)
        else:
            msg=arraypoints()
            msg.x=[]
            msg.y=[]
            msg.c=[]
            msg.t=[rospy.get_time()]
            #print msg
            self.pubPointsFound.publish(msg)

            msgl=arraypoints()
            msgl.x=self.points[:,0].tolist()
            msgl.y=self.points[:,1].tolist()
            self.pubpointsLeft.publish(msgl)


        return getAverageMapValueResponse(S)     

    def resetPoints(self, msg):
        if msg.data:
            self.points=np.asarray(rospy.get_param("/points")) 
            self.pointsFound=np.array([[]])


def main():
  
  rospy.init_node('getAverageMapValue_server')
  mapServer = mapServer_class()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
