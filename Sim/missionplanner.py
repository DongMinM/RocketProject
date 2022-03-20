import rospy
from std_msgs.msg import Float32MultiArray


class MissionPlanner:
    def __init__(self):
        rospy.init_node('MissionPlanner')
        self.pub = rospy.Publisher('MissionCode',Float32MultiArray,queue_size=1)
        self.data = []
        # self.data : Vx, Vy, Vz, x, y, z Ax, Ay, Az, psi, theta, phi, wx, wy, wz
        self.motor_seperation = 0
        self.payload_seperation = 0


    def run(self):
        rospy.Subscriber('data',Float32MultiArray, self.update_status)
        print('---Mission Planner On---')
        while True:
            if len(self.data) > 0:
                if self.data[8] > 2:   # tvc on
                    data = Float32MultiArray()
                    data.data = self.data
                    self.pub.publish(data)
                    print('Tvc on')

                elif self.data[8] < -9 and self.motor_seperation == 0: # Motor seperation
                    print('Motor seperation at {}'.format(self.data[5]))
                    self.motor_seperation = 1

                elif self.data[2] < -1 and self.data[5] >100 and self.payload_seperation == 0:  # payload seperation
                    print('Payload Seperation at {}'.format(self.data[5]))
                    self.payload_seperation = 1
                else :
                    pass

            else:
                pass
            rospy.sleep(0.01)

    def update_status(self,msg):
        self.data = msg.data


if __name__ == '__main__':
    m = MissionPlanner()
    m.run()