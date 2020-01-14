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


# #!/usr/bin/env python

# from cfswarm.srv import getAverageMapValue,getAverageMapValueResponse
# import rospy
# import numpy as np
# import math


# def handle_getAverageMapValue(req):
#     values=[]

#     for x in range(-5,5):
#         for y in range(-5,5):
#             if req.posX+x>0 and req.posX+x<map.shape[0] and req.posY+y>0 and req.posY+y<map.shape[1]:
#                values.append(map[req.posX+x][req.posY+y])
    
#     S=sum(values)/len(values)
 
#     return getAverageMapValueResponse(S)
    

# def getAverageMapValue_server():
#     rospy.init_node('getAverageMapValue_server')
#     # global map
#     # map=np.asarray(rospy.get_param("/matrix"))
#     global m1, m2, m3, m4, s1, s2, s3, s4
#     m1=np.asarray(rospy.get_param("/m1"))
#     m2=np.asarray(rospy.get_param("/m2"))
#     m3=np.asarray(rospy.get_param("/m3"))
#     m4=np.asarray(rospy.get_param("/m4"))

#     s1=np.asarray(rospy.get_param("/s1"))
#     s2=np.asarray(rospy.get_param("/s2"))
#     s3=np.asarray(rospy.get_param("/s3"))
#     s4=np.asarray(rospy.get_param("/s4"))


#     s = rospy.Service('getAverageMapValue',getAverageMapValue, handle_getAverageMapValue)
#     print "Ready to retrieve map data."
#     rospy.spin()

# def ComputeVal():


# def Single2DGaussian(u, sigma, x, y);
    
#     f=1/(math.sqrt(2*math.pi)*sigma[0])*math.exp(-1/(2*sigma[0]^2)*(x-u[0]))* \
#       1/(math.sqrt(2*math.pi)*sigma[1])*math.exp(-1/(2*sigma[1]^2)*(x-u[1]))

# if __name__ == "__main__":

#     getAverageMapValue_server()
