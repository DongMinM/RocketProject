from transform import Transformer
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import axes3d
import rospy
from std_msgs.msg import Float32MultiArray
import math
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
        thrust                  ## 추력
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

        ''' Set initial variables'''
        self.mass = mass_pro+mass_struct                                ## 총 무게
        self.total_mass_center = 0                                      ## 최종 무게 중심
        self.zeroparam = np.hstack((self.velocity, self.position, self.mass, self.rocket_angle,self.angular_velocity))
                                                                        ## 로켓 상태값
        self.accel = np.empty((0,3))                                    ## 가속도 저장 공간
        self.tvc_motor_angle = np.array([0,0,0])                        ## Tvc 각도
        self.params = []                                                ## 상태값 저장 공간

    def calculate_next_params_of_rocket(self):

        ## in burnning
        if realTime < self.t_b:

            self.motor_angle = self.tvc_motor_angle         ## Read tvc angle. defualt is [0,0,0]

            for i in [0,1,2]:                               ## Tvc각 제한 [roll,pich,yow (')]
                if self.motor_angle[i] > 3:
                    self.motor_angle[i] = 3
                elif self.motor_angle[i] < -3:
                    self.motor_angle[i] = -3

            self.params = Differentail_equation(self).in_burnning(self.motor_angle)     ## 추진 중 미분 방정식


        ## free fall
        else :
            self.params = Differentail_equation(self).end_of_burnning()                 ## 자유 낙하 미분 방정식


    def readTvc(self,msg):                                  ## subscribe tvc angle ( msg => 0, ahphe, beta)
        self.tvc_motor_angle = np.array(msg.data)

        rospy.sleep(0.05)        





if __name__ == '__main__':
    realTime = 0                                # Set real Time 0s

    '''Set initial status'''
    position = np.array([0,0,0])                # rocket initial posision (x,y,z (m))
    velocity = np.array([0,0,0])                # rocket initial velocity (x,y,z (m/s))
    rocket_angle = np.array([0,0,10])           # rocket initial angle (r,p,y ('))
    angular_velocity = np.array([0,0,0])        # rocket initial angular velocity (r,p,y (rad/s))

    '''Set rocket body'''
    rocket_length = 1.2                         # rocket length (m)
    diameter = 0.09                             # rocket outside diameter (m)
    mass_struct = 3                             # structure mass (kg)
    mass_center = [0.6,0.1]                     # rocket initial mass center of structure, propellant (meters from bottom)
    aerocenter = 0.45                           # rocket aero center (meters from bottom)
    drag_coeff = 0.4                            # drag coefficient


    '''Set motor'''
    mass_pro = 0.5                              # propellent mass (kg)
    thrust = np.array([0,0,100])                # thrust (N)
    burnTime = 3                                # burnning time (s)
    motor_angle = np.array([0,0,0])             # rocket initial motor angle (r,p,y ('))

    # Declare rocket class
    rocket = Rocket_system(mass_struct,mass_pro,burnTime,drag_coeff,position,velocity,angular_velocity,\
                           rocket_angle,motor_angle,rocket_length,mass_center,aerocenter,diameter,thrust)

    # just Set rospy
    rospy.init_node('simulation')                                                       # Set rospy node
    pub2missionplanner = rospy.Publisher('data',Float32MultiArray,queue_size=10)        # publish to missionplanner
    rospy.Subscriber('Actuator',Float32MultiArray,rocket.readTvc)                       # Tvc on (communicate with Tvc node)

    # just set plotting
    fig = plt.figure(facecolor='w')                             ## plotting figure
    # ax = fig.add_subplot(111,projection='3d')                   ## 3d plot
    ax = fig.add_subplot(1,1,1)                                 ## 2d plot
    # ax2 = ax.twinx()                                            ## set second y-axis
    
    ''' Just set initial variables '''
    rocket_shape = []                                           ## initialize rocket shape
    rocket_pos_list = np.empty((0,3))                           ## save all position for plotting
    rocket_velocity_list = np.empty((0,3))                      ## save all velocity
    rocket_angle_list = np.empty((0,3))                         ## save all rocket angle
    rocket_accel_list = np.empty((0,3))                         ## save all rocket accel
    rocket_total_velocity_list = np.empty((0,1))                ## total velocity
    rocket_total_mass = np.empty((0,1))                         ## total mass
    
    Apogee = 'None'
    arrive_time = 0                                             
    eject_num = 0                                               
    Max_acceleration = 0
    Max_velocity = 0
    Max_x = 0

    ''' Main loop of simulator'''
    while realTime <= 25:                                       ## flight time (s)

        data = Float32MultiArray()
        angle_data = rocket.zeroparam[7:10]                     ## slicing angle
        w_data = rocket.zeroparam[10:13]                        ## slicing anguler velocity
        data.data = np.append(angle_data,w_data,axis=0)         ## publish rocket angle, w

        pub2missionplanner.publish(data)                                       ## publish angle and anguler velocity

        # rospy.sleep(0.01)                                        ## set publish sleep time


        rocket.calculate_next_params_of_rocket()                ## calculate next params of rocket ( main simulator )
        rocket.zeroparam = rocket.params[-1]                    ## set zeroparams of rocket



        # transform matrix rocket => ground
        M2 = Transformer().body_to_earth(np.array([rocket.params[-1,7], rocket.params[-1,8], rocket.params[-1,9]]))
        rocket_pos_vector = 20*M2@[0,0,1]                               ## rocket body vector in ground (half size of rocket)

        # Apogee
        if rocket.zeroparam[2] < 0 and rocket.zeroparam[5] > 100 and eject_num == 0:    ## calculate Apogee / ejection
            Apogee = rocket.zeroparam[5]
            eject_num +=1
        # Max pos X
        if rocket.zeroparam[3] > Max_x:                                 ## calculate Max x
            Max_x = rocket.zeroparam[3]
        # Max velocity (z)
        if rocket.zeroparam[2] > Max_velocity:                          ## calculate Max velocity (z)
            Max_velocity = rocket.zeroparam[2]
        # Max accel (z)
        if rocket.accel[2] > Max_acceleration:                          ## calculate Max accleration (z)
            Max_acceleration = rocket.accel[2]
        # angular arrive time (40')
        if math.ceil(rocket.zeroparam[9]) == 40:                        ## Time when rocket arrive 40'
            arrive_time = realTime


        '''Save parameters  index i means i*0.05 second
            rocket_total_mass[i,0] means mass at i*0.05 second
            rocket_velocity_list[i,2] means Vz at i*0.05 second'''
        # rocket mass
        rocket_total_mass = np.append(rocket_total_mass,np.array([[rocket.zeroparam[6]]]),axis=0)
        # rocket angle (roll, pich, yaw ('))
        rocket_angle_list = np.append(rocket_angle_list,np.array([[rocket.zeroparam[7],\
                                                                   rocket.zeroparam[8],\
                                                                   rocket.zeroparam[9]]]),axis=0)
        # rocket accel (Ax, Ay, Az)
        rocket_accel_list = np.append(rocket_accel_list,np.array([[rocket.accel[0],\
                                                                   rocket.accel[1],\
                                                                   rocket.accel[2]]]),axis=0)
        # rocket velocity (Vx, Vy, Vz)
        rocket_velocity_list = np.append(rocket_velocity_list,np.array([[rocket.params[-1,0],\
                                                                         rocket.params[-1,1],\
                                                                         rocket.params[-1,2]]]),axis=0)
        # rocket pos (X, Y, Z)
        rocket_pos_list = np.append(rocket_pos_list,np.array([[rocket.params[-1,3],\
                                                               rocket.params[-1,4],\
                                                               rocket.params[-1,5]]]),axis=0)
        # mass center
        rocket_pos_center = np.array([rocket.params[-1,3],\
                                      rocket.params[-1,4],\
                                      rocket.params[-1,5]])
        # total velocity
        rocket_total_velocity_list = np.append(rocket_total_velocity_list,\
                                     np.array([[(rocket.params[-1,0]**2+rocket.params[-1,1]**2+rocket.params[-1,2]**2)**(1/2)]]),axis=0)
        # rocket shape for plotting
        rocket_shape.append([[rocket_pos_center[0]-rocket_pos_vector[0],rocket_pos_center[0]+rocket_pos_vector[0]],
                             [rocket_pos_center[1]-rocket_pos_vector[1],rocket_pos_center[1]+rocket_pos_vector[1]],
                             [rocket_pos_center[2]-rocket_pos_vector[2],rocket_pos_center[2]+rocket_pos_vector[2]]])
        # rocket shape : center of mass - body vector    ~~    center of mass + body vector

        # time flow
        realTime += 0.05

    '''==============================
        This is just animation function '''
    ## make animation
    # def animate(i):
        # ax.clear()
        # ax2.clear()                                                    # initialization
        # ax.set_xlim(0, 45000)
        # ax.set_ylim(0, 45000)
        # ax.set_zlim(0, 1000)

        # ax.view_init(elev=90., azim=60)                                # set view angle
        # ax.set_xlabel('time')                                          # set x label
        # ax.set_ylabel('altitude')                                      # set y label

        # ax2.set_ylim(-30,150)
        # ax2.set_ylabel('Vertical Velocity, acceleration')              # set second y label                                                            
        
        # x = rocket_shape[i][0]
        # y = rocket_shape[i][1]
        # z = rocket_shape[i][2]

        # ax.plot(rocket_pos_list[:i,0],rocket_pos_list[:i,1],rocket_pos_list[:i,2],'b-',label = '1st')   
        # time = i*0.05
        # ax.text(1000,1000,1000,'Time = %.1fs'%time)                                                           # plot time
        # ax.text(1000,1000,3000,'Altitude = %.1fm'%rocket_pos_list[i,2])

        # if rocket_velocity_list[i,2] < 0 and rocket_pos_list[i,2] > 100:
        #     ax.text(10000,10000,10000,'Eject Altitude = %.1fm'%Apogee)

        # x = np.linspace(0,i*0.05,i+1)
        # y = 90-rocket_angle_list[0:i+1,2]
        # y = rocket_pos_list[0:i+1,2]
        # y2 = rocket_velocity_list[0:i+1,2]
        # y3 = rocket_accel_list[0:i+1,2]
        
        
        # plt.plot.axhline(y=50)
        # if rocket_angle_list[i,2] > 40:
            # plt.axvline(x=arrive_time)
            # ax.text(2,45,'arrive time : %.1fs'%arrive_time,)

        # return ax.plot(x,y,z,color = 'k', lw = 2)  
        
        # ax2.plot(x,x*0,linestyle = '--')
        # ax2.plot(x,y2,'b',lw=2)
        # ax2.plot(x,y3,'g',lw=2)
        # return ax.plot(x,y,'r',lw=2)                                                         # plot rocket
    '''================================================='''
    
    # animate = animation.FuncAnimation(fig,animate, frames = 400, interval=1)                                # make animation
    ax.set_xlim(0,800)                                                      ## set x line
    ax.set_xlabel('Time')                                                   ## set x label
    ax.set_ylim(0,800)                                                      ## set y line
    ax.set_ylabel('Total mass')                                             ## set y label
    # ax2.set_ylim(-100,2000)                                               ## set second y line
    # ax2.set_ylabel('Vertical Velocity, acceleration')                     ## set second y label
    
    '''Set x coordinate '''
    # x = np.linspace(0,125,2501)                                             ## set x (time)
    # x = np.arange(0,10,0.05)                                              ## set x (time)
    # ax.plot(x,x*0,linestyle='--')                                         ## Draw line y = 0
    '''Set y coordinate'''
    # y = 90-rocket_angle_list[:,2]                                         ## rocket angle from ground
    x = rocket_pos_list[:,0]                                                ## rocket pos x
    y = rocket_pos_list[:,2]                                                ## rocket pos z
    # y = rocket_velocity_list[:,2]                                           ## rocket velocity Vz
    # y = rocket_total_velocity_list[:,0]                                     ## rocket total velocity
    # y = rocket_total_mass[:,0]                                              ## rocket mass
    # y = rocket_accel_list[:,2]                                              ## rocket acceleration Az

    ax.plot(x,y)                                                            ## plot x,y function
    # plt.axhline(y=70,ls='--')                                             ## Draw line y = 70

    # ax.text(1000,3000,'Apogee : %.1fm'%Apogee)                            ## Write text (Apogee )
    # ax.text(1000,4000,'Max x : %.1fm'%Max_x)                              ## Write text ( Max pos x )
    ax.text(30,1000,'Max velocity : %.1fm/s'%np.max(rocket_total_velocity_list))    ## Write text ( total velocity )
    # ax.text(23,500,'Max acceleration : %.1fm/s'%Max_acceleration)                 ## Write text ( Max acceleration )

    print('---plotting---please wait------')
    plt.show()                                                              ## show picture
    plt.rcParams['font.size'] = 10                                          ## font size
    # animate.save("file name').mp4",fps=20)                                ## save, 1 frame => 0.05s ( 30 fps = 1.5 real fps)
    print('---finish---check your folder which this file is in---')