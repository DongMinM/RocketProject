import rospy
from std_msgs.msg import Float32MultiArray


class MissionPlanner:
    def __init__(self):
        rospy.init_node('MissionPlanner')
        self.pub = rospy.Publisher('MissionCode',Float32MultiArray,queue_size=1)
        self.data = []
        # self.data : psi, theta, phi, wx, wy, wz


    def run(self):
        rospy.Subscriber('data',Float32MultiArray, self.callback)
        print('---Mission Planner On---')
        while True:
            if len(self.data) > 0:                      ## tvc triger
                data = Float32MultiArray()
                data.data = self.data
                self.pub.publish(data)
                print('Read data')
            else:
                pass
            rospy.sleep(0.3)

    def callback(self,msg):
        self.data = msg.data


if __name__ == '__main__':
    m = MissionPlanner()
    m.run()