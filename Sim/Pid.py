import numpy as np
from transform import Transformer
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String


class Actuator:
    def __init__(self):
        #vx, vy, vz, px, py, pz, m, psi, theta, phi, alpha, beta, gamma, wx, wy, wz
        self.angle = np.array([0,0,0])       
        self.set_angle = np.array([0,0,20])
        self.w = np.array([0,0,0])
        self.Kp = 0.78                   ## 0.078    
        self.Ki = 0.39                   ## 0.049     
        self.Kd = 0.94                   ## 0.25~0.255     
        self.gap_angle = np.array([0,0,0])
        self.Error_sum = 0
        self.last_Error = np.array([0,0,-10])
        rospy.init_node('Actuator')
        self.pub = rospy.Publisher('Actuator',Float32MultiArray,queue_size=1)

    def tvc(self):
        for i in [0,1,2]:
            while self.angle[i] > 180:
                self.angle[i] -= 360
            while self.angle[i] < -180:
                self.angle[i] += 360
        # print(self.angle)
        Error = self.angle - self.set_angle
        print(Error)
        P = self.Kp*Error
        self.Error_sum += Error
        I = self.Ki*self.Error_sum*0.2
        print(self.Error_sum)
        # print(self.Error_sum)
        # print('I : ',I)
        D = self.Kd*(Error-self.last_Error)/0.2
        # D = self.Kd*self.w*180/np.pi
        # print('P : ',P)
        # print('D : ',D)
        self.last_Error = Error
        # print(I)
        # print('sumI : ',self.sumI)
        self.gap_angle = Error
        motor_angle_dot = P+I+D
        tvcdata = Float32MultiArray()
        tvcdata.data = np.array([motor_angle_dot[0],motor_angle_dot[1],motor_angle_dot[2]])
        self.pub.publish(tvcdata)
        rospy.sleep(0.05)

    def run(self):
        rospy.Subscriber('MissionCode',Float32MultiArray, self.missionstarter)
        rospy.spin()

    def missionstarter(self,msg):
        # self.datahub.data : psi, theta, phi,wx, wy, wz
        self.angle = np.array([msg.data[0],msg.data[1],msg.data[2]]) 
        self.w = np.array([msg.data[5],msg.data[3],msg.data[4]])
        self.tvc()






if __name__ == '__main__' :
    actuator = Actuator()
    actuator.run()


## 갑자기 늘어나는 구간 arcsin, arctan 값 확인하기