#!/usr/bin/env python3

import rospy 

class Listen():
  def __init__(self):

    self.node_name = "miro_listen_stt"

    rospy.init_node(self.node_name, anonymous=True)
    self.rate = rospy.Rate(100)



  if __name__ == '__main__':
    ## test