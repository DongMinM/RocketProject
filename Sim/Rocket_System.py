from difflib import Differ
from transform import Transformer
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray,String
from differential_equation import Differentail_equation


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
        Cd_para_on,
        para_on_h
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
        self.Cd_para_on = Cd_para_on
        self.para_on_h = para_on_h

        self.k = 0

        ''' Set initial variables'''
        self.mass = mass_pro+mass_struct                                ## 총 무게
        self.total_mass_center = 0                                      ## 최종 무게 중심
        self.zeroparam = np.hstack((self.velocity, self.position, self.mass, self.rocket_angle,self.angular_velocity))
                                                                        ## 로켓 상태값
        self.accel = np.empty((0,3))                                    ## 가속도 저장 공간
        self.tvc_motor_angle = np.array([0,0,0])                        ## Tvc 각도
        self.params = []                                               ## 상태값 저장 공간
        
        rospy.Subscriber('TVC',Float32MultiArray,self.readTvc)
        # rospy.Subscriber('Motor_seperation',String,self.motor_seperation)
        # rospy.Subscriber('Payload_seperation',String,self.payload_seperation)
        self.subNum = 0

    def calculate_next_pos_of_rocket(self):

        ## in burnning
        if round(self.realTime,5) < self.t_b:
            self.motor_angle = self.motor_angle+(self.tvc_motor_angle-self.motor_angle)/4         ## Read tvc angle. defualt is [0,0,0]
            self.subNum = 0
            self.motor_angle[0] = 0
            # print('Motor angle : ',self.motor_angle)
            self.params = Differentail_equation(self).in_burnning(self.motor_angle)     ## 추진 중 미분 방정식


        ## free fall
        else :
            if round(self.realTime,5) == self.t_b:
                print('---Finish Burnning---')

            if self.params[-1][5] <= 0:             ## 착륙 완료
                print('---Finisth Flight---')

            else :
                if self.params[-1][2]<0 and self.params[-1][5] < self.para_on_h :
                    self.Cd_para = self.Cd_para_on
                    # self.T = [0,0,0]
                    # self.T = [0,0,9.23*(9.81-0.7*self.params[-1][2]-0.28*self.params[-1][5])-0.5*1.225*self.params[-1][2]*self.params[-1][2]*np.pi*self.diameter*self.diameter/4*0.6 ]
                    # self.T = [(-0.14*self.params[-1][11]+0.35*self.params[-1][9]-0.17*self.params[-1][0]+0.06*(self.params[-1][3])-0.5*1.225*self.params[-1][0]*self.params[-1][0]*np.pi*self.diameter*self.diameter/4*0.6),0,9.23*(9.81-0.85*self.params[-1][2]+0.15*(50-self.params[-1][5]))-0.5*1.225*self.params[-1][2]*self.params[-1][2]*np.pi*self.diameter*self.diameter/4*0.6 ]
                    Ki = 0.35
                    Kp = 0.09
                    z = self.params[-1][5]
                    vz = self.params[-1][2]
                    x = self.params[-1][3]
                    vx = self.params[-1][0]
                    theta2 = np.arctan((Ki*vx+Kp*x)/(15*9.81-Ki*vz-Kp*z))
                    T = (15*9.81-Ki*vz-Kp*z)/np.cos(theta2)
                    # self.T = [-0.2*self.params[-1][11]+0.35*self.params[-1][9]+T*np.sin(theta2),0,3*T*np.cos(theta2)]
                    self.T = [+T*np.sin(theta2)-0.35*self.params[-1][11]+0.09*self.params[-1][9],0,T*np.cos(theta2)]
                    if self.T[2] <= 0:
                        self.T[2] = 0
                    if self.T[1] <= 0:
                        self.T[1] = 0
                    # if self.params[-1][5] < 65 :
                    #     self.Cd_para = 1.2
                    #     self.T = [0,0,0]
                    print(self.T)
                    self.params = Differentail_equation(self).in_burnning(self.motor_angle)                                  ## 착륙 x
                else :
                    self.T = [0,0,0]
                    self.params = Differentail_equation(self).in_burnning(self.motor_angle)                 ## 자유 낙하 미분 방정식


    def readTvc(self,msg):
        if self.realTime <= self.t_b and self.subNum == 0:                                  ## subscribe tvc angle ( msg => 0, ahphe, beta)
            self.tvc_motor_angle = np.array(msg.data)
            for i in [1,2]:                               ## Tvc각 제한 [roll,pich,yow (')]
                if self.tvc_motor_angle[i] > 3:
                    self.tvc_motor_angle[i] = 3
                elif self.tvc_motor_angle[i] < -3:
                    self.tvc_motor_angle[i] = -3
            
            rospy.sleep(0.01)  
            print('Get Tvc angle')
            self.subNum += 1 
        else :
            pass    
    
    def motor_seperation(self):
        # self.zeroparam = 
        pass

    def payload_seperation(self):
        # self.Cd = 
        pass