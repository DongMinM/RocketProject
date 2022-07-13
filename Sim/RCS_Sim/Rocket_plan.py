from transform import Transformer
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray,String
from differential import Differentail_equation


class Rocket_system:
    ''' initial variables / if you wanna more information, go to main loop.'''
        
    def __init__(
        self,
        mass_struct,            ## 구조체 질량
        mass_pro,               ## 추진제 질량
        t_b,                    ## 연소 시간
        drag_coeff,             ## 항력 계수
        position,               ## 초기 위치
        velocity,               ## 초기 속도
        angular_velocity,       ## 초기 각속도
        rocket_angle,           ## 초기 각도         
        motor_angle ,           ## 초기 모터 각도
        rocket_length ,         ## 로켓 길이
        mass_center,            ## 무게 중심
        aerocenter,             ## 공력 중심
        diameter,               ## 직경
        thrust,                 ## 추력
    ):
        self.mass_struct = mass_struct              
        self.mass_pro = mass_pro                    
        self.t_b = t_b                             
        self.Cd = drag_coeff                        
        self.position = position                    
        self.velocity = velocity                    
        self.angular_velocity = angular_velocity    
        self.rocket_angle = rocket_angle            
        self.motor_angle = motor_angle              
        self.rocket_length = rocket_length            
        self.mass_center = mass_center
        self.diameter = diameter
        self.T = thrust
        self.total_aero_center = aerocenter
        self.realTime = 0
        self.Cd_para = 0

        self.arrive_ground = 0
        self.first_g = 0

        self.left_T = [0,0,0]
        self.right_T = [0,0,0]

        self.t1 = 0
        self.total_drag = [0]



        ''' Set initial variables'''
        self.mass = mass_pro+mass_struct                                ## 총 무게
        self.total_mass_center = 0                                      ## 최종 무게 중심
        self.zeroparam = np.hstack((self.velocity, self.position, self.mass, self.rocket_angle,self.angular_velocity))
                                                                        ## 로켓 상태값
        self.accel = np.empty((0,3))                                    ## 가속도 저장 공간
        self.params = []                                               ## 상태값 저장 공간
        

    def calculate_next_pos_of_rocket(self):

        ## in burnning
        if round(self.realTime,5) < self.t_b:
            self.params = Differentail_equation(self).in_burnning(self.motor_angle)     ## 추진 중 미분 방정식


        ## free fall
        else :
            self.mass_pro = 0
            self.T = [0,0,0]
            if self.params[-1][5] > 0:
                if self.params[-1][2]<0 and self.params[-1][5] < 220 and self.t1 == 0:
                    self.t1 = self.realTime
                if self.t1 != 0 and self.realTime >= self.t1:
                    
                    # self.left_T = np.array([1,0,0])
                    self.right_T = np.array([2,0,0])
                    if self.realTime >= self.t1+0.5:
                        self.right_T = np.array([0,0,0])
                        self.left_T = np.array([1,0,0])
                    if self.realTime >= self.t1+1.08:
                        self.right_T = np.array([0,0,0])
                        self.left_T = np.array([0,0,0])
                        self.T = [0,0,50]
                    if self.realTime >= self.t1+5:
                        self.T = [0,0,0]
                    if self.realTime >= self.t1+6:
                        self.Cd_para = 1.2

                    self.params = Differentail_equation(self).in_burnning(self.motor_angle)

                else:

                    self.params = Differentail_equation(self).in_burnning(self.motor_angle)                 ## 자유 낙하 미분 방정식
            else :
                self.T = [0,0,0]
                if self.first_g ==0 :
                    self.arrive_ground = self.realTime
                    self.first_g = 1