from datahub import DataHub
import rospy

class MissionPlanner:
    def __init__(self):
        
        self.datahub = DataHub()
        # vx, vy, vz, px, py, pz, m, psi, theta, phi, alpha, beta, gamma, wx, wy, wz


    def run(self):
        self.datahub.run()
        while True:
            if len(self.datahub.data) > 0:
                print(self.datahub.data[5])
            else:
                print('No data')

            rospy.sleep(0.1)


if __name__ == '__main__':
    m = MissionPlanner()
    m.run()