import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray


class Actuator:
    def __init__(self):
        self.angle = np.array([0,0,0])                      ## rocket angle
        self.set_angle = np.array([0,0,20])                 ## set angle

        self.Kp = 0.78                   ## 0.78    
        self.Ki = 0.39                   ## 0.39     
        self.Kd = 0.94                   ## 0.94     

        self.Error_sum = 0
        self.last_Error = np.array([0,0,-10])       # 초기 에러 rocket angle - set angle 로 설정 해 주어야 함.

        rospy.init_node('Actuator')
        self.pub = rospy.Publisher('Actuator',Float32MultiArray,queue_size=1)

    def tvc(self):

        for i in [0,1,2]:                                   ## angle set -180 ~ 180
            while self.angle[i] > 180:
                self.angle[i] -= 360
            while self.angle[i] < -180:
                self.angle[i] += 360

        Error = self.angle - self.set_angle
        print('Error : ',Error)

        P = self.Kp*Error
        self.Error_sum += Error
        I = self.Ki*self.Error_sum*0.2
        D = self.Kd*(Error-self.last_Error)/0.2
        self.last_Error = Error
        motor_angle = P+I+D
        print('motor angle : ',motor_angle)
        tvcdata = Float32MultiArray()
        tvcdata.data = np.array([motor_angle[0],motor_angle[1],motor_angle[2]])
        self.pub.publish(tvcdata)
        rospy.sleep(0.05)

    def run(self):
        rospy.Subscriber('MissionCode',Float32MultiArray, self.missionstarter)
        rospy.spin()

    def missionstarter(self,msg):
        # msg.data : psi, theta, phi, wx, wy, wz
        self.angle = np.array([msg.data[0],msg.data[1],msg.data[2]])        ## read rocket angle
        self.tvc()






if __name__ == '__main__' :
    actuator = Actuator()
    actuator.run()