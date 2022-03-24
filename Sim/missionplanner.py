import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

'''
이 코드는 센서로부터 데이터를 받아
로켓이 수행할 미션을 선택하고
Actuator 와 groundstation으로 status를 보내는 역할을 합니다.
'''

class MissionPlanner:
    def __init__(self):

        self.rf_data = []       # self.data : Vx, Vy, Vz, x, y, z Ax, Ay, Az, psi, theta, phi, wx, wy, wz
        self.payload_seperation = 0     # if payload seperated 0 --> 1

        rospy.init_node('MissionPlanner')
        self.pubtoActuator = rospy.Publisher('MissionCode',Float32MultiArray,queue_size=1)
        self.data2Actuator = Float32MultiArray()    # send data to Actuator (will change to rf)

        self.pubtoground = rospy.Publisher('SensorData',Float32MultiArray,queue_size=1)
        self.data2ground = Float32MultiArray()      # send data to groundstation  (will change to rf)

        rospy.Subscriber('data',Float32MultiArray, self.update_status)  # read sensor data (will change to serial)
        

    def run(self):
        print('---Mission Planner On---')
        while True:         ## Main loop
            ''' ground station으로 보내는 데이터 중 앞의 [a,b]는 각각
                연소 중인지 (1 --> burn / 0 --> burnout)
                페어링 분리가 되었는지( 1 --> seperated / 0 --> non-seperated)
                나타내는 Code 입니다.'''
            if len(self.rf_data) > 0:
                if self.rf_data[8] > 2:   # tvc on
                    self.data2Actuator.data = self.rf_data          
                    self.pubtoActuator.publish(self.data2Actuator)  # Tvc mission to Actuator
                    self.data2ground.data = [1,0]+list(self.rf_data) # send mission and rf data to ground station
                    print('Tvc on')


                elif self.rf_data[2] < -1 and self.rf_data[5] >100 and self.payload_seperation == 0:  # payload seperation
                    # self.pubtoActuator.publish(data2Actuator)              # send seperation mission to Actuator
                    print('Payload Seperation at {}'.format(self.rf_data[5]))
                    self.data2ground.data = [0,1]+list(self.rf_data)
                    self.payload_seperation = 1

                else :
                    if self.payload_seperation == 1:
                        self.data2ground.data = [0,1]+list(self.rf_data)

                    else:
                        self.data2ground.data = [0,0]+list(self.rf_data)

                self.pubtoground.publish(self.data2ground)

            else:
                pass

            rospy.sleep(0.01)

    def update_status(self,msg):
        self.rf_data = msg.data
        # self.rf_data : Vx, Vy, Vz, x, y, z Ax, Ay, Az, psi, theta, phi, wx, wy, wz

if __name__ == '__main__':
    m = MissionPlanner()
    m.run()