from logging import shutdown
import rospy
from std_msgs.msg import Float32MultiArray

class DataHub:


    def __init__ (self):
        self.data = []
        self.code = 0
        # vx, vy, vz, px, py, pz, m, psi, theta, phi, alpha, beta, gamma, wx, wy, wz


    def run(self):
        rospy.Subscriber('data',Float32MultiArray, self.callback)
    
    def callback(self,msg):
        self.data = msg.data
        

if __name__ == '__main__':
    datahub = DataHub()
    datahub.run()