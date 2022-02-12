import numpy as np
from transform import Transformer
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String


class Actuator:
    def __init__(self):
        #vx, vy, vz, px, py, pz, m, psi, theta, phi, alpha, beta, gamma, wx, wy, wz
        self.angle = np.array([0,0,0])       
        self.set_angle = np.array([0,0,0])
        self.w = np.array([0,0,0])
        self.w_dot = np.array([0,0,0])
        self.d = 2.8                     ## 2.8    4      내려갔을 경우 회복(그래프 오른쪽으로 이동)
        self.i = 7.7                    ## 7.7     5     내려가는 속도의 반대 - 속도 조절
        self.p = 5.2                    ## 5.2     2     내려가는 속도- 각도에 대한 민감도
        self.gap_angle = np.array([0,0,0])

        rospy.init_node('Actuator')
        self.pub = rospy.Publisher('Actuator',Float32MultiArray,queue_size=1)

    def tvc(self):
        for i in [0,1,2]:
            while self.angle[i] > 180:
                self.angle[i] -= 360
            while self.angle[i] < -180:
                self.angle[i] += 360
        print(self.angle)
        self.gap_angle = self.angle - self.set_angle
        motor_angle_dot = self.d*self.w_dot+self.i*self.w+self.p*self.gap_angle
        tvcdata = Float32MultiArray()
        tvcdata.data = np.array([motor_angle_dot[0],motor_angle_dot[1],motor_angle_dot[2]])
        self.pub.publish(tvcdata)

    def run(self):
        rospy.Subscriber('MissionCode',Float32MultiArray, self.missionstarter)
        rospy.spin()

    def missionstarter(self,msg):
        # self.datahub.data : vx, vy, vz, px, py, pz, m, psi, theta, phi, alpha, beta, gamma, wx, wy, wz, wx_dot, wy_dot, wz_dot
        self.angle = np.array([msg.data[7],msg.data[8],msg.data[9]]) 
        self.w = np.array([msg.data[15],msg.data[13],msg.data[14]])
        self.w_dot = np.array([msg.data[18],msg.data[16],msg.data[17]])
        self.gap_angle = np.array([0,0,0])
        self.tvc()






if __name__ == '__main__' :
    actuator = Actuator()
    actuator.run()


## 갑자기 늘어나는 구간 arcsin, arctan 값 확인하기