from datahub import DataHub
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String


class MissionPlanner:
    def __init__(self):
        rospy.init_node('MissionPlanner')
        self.pub = rospy.Publisher('MissionCode',Float32MultiArray,queue_size=1)
        self.data = []
        # self.datahub.data : vx, vy, vz, px, py, pz, m, psi, theta, phi, alpha, beta, gamma, wx, wy, wz


    def run(self):
        rospy.Subscriber('data',Float32MultiArray, self.callback)
        while True:
            if len(self.data) > 0:
                # print(self.data[5])
                data = Float32MultiArray()
                data.data = self.data
                self.pub.publish(data)
            else:
                print('No data')
            rospy.sleep(0.3)

    def callback(self,msg):
        self.data = msg.data


if __name__ == '__main__':
    m = MissionPlanner()
    m.run()