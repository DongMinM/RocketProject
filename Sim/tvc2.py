import numpy as np
from transform import Transformer

class Tvc:
    def __init__(self,now_param,w_dot,mass_center,trust,set_angle):
        #vx, vy, vz, px, py, pz, m, psi, theta, phi, alpha, beta, gamma, wx, wy, wz
        self.angle = np.array([now_param[7],now_param[8],now_param[9]])       
        self.set_angle = np.array([set_angle[0],set_angle[1],set_angle[2]])
        self.w = np.array([now_param[13],now_param[14],now_param[15]])
        self.w_dot = np.array([w_dot[0],w_dot[1],w_dot[2]])
        self.d = 8                      ## 10    8      내려갔을 경우 회복(그래프 오른쪽으로 이동)
        self.i = 27                    ## 20     27     내려가는 속도의 반대 - 속도 조절
        self.p = 35                    ## 35     35     내려가는 속도- 각도에 대한 민감도
        self.gap_angle = np.array([0,0,0])

    def run(self):
        for i in [0,1]:
            while self.angle[i] > 180:
                self.angle[i] -= 360
            while self.angle[i] < -180:
                self.angle[i] += 360

        self.gap_angle = self.angle - self.set_angle
        motor_angle_dot = self.d*self.w_dot+self.i*self.w+self.p*self.gap_angle

        return motor_angle_dot


if __name__ == '__main__' :
    tvc = Tvc([0,0,0,0,0,0,2,0,10,0,0,0,0,0,0,0],[0,0,0],0.5,100.0,set_angle=[0,0,0])
    tvc.run()


## 갑자기 늘어나는 구간 arcsin, arctan 값 확인하기