from transform import Transformer
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import axes3d
import rospy
from std_msgs.msg import Float32MultiArray
import math


class Rocket_system:
    def __init__(
        self,
        mass_struct,
        mass_pro,
        t_b,
        drag_coeff,
        position,
        velocity,
        angular_velocity,
        rocket_angle,                    
        motor_angle ,
        rocket_length ,
        mass_center,
        aerocenter,
        diameter,
        thrust
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
        self.totla_aero_center = aerocenter

        self.mass = mass_pro+mass_struct                                ## total mass
        self.total_mass_center = 0                                      ## set space of total mass center
        self.angle_dot = [0,0,0]                                        ## initiate angle_dot
        self.zeroparam = np.hstack((self.velocity, self.position, self.mass, self.rocket_angle,self.angular_velocity))
        ## set zeroparam space
        self.force_effect = []                                          ## set space of force effect
        
        self.w_dot = np.array([0,0,0])                                  ## set space of w_dot
        self.accel = np.empty((0,3))                                    ## set space of accel
        self.tvc_motor_angle = np.array([0,0,0])


    ## in burnning
    def force_burn(self,zeroparam,t,alpha,beta,gamma):

        T = self.T
        vx, vy, vz, px, py, pz, m, psi, theta, phi,wx, wy, wz = zeroparam

        self.total_mass_center = (self.mass_center[0]*self.mass_struct+self.mass_center[1]*(m-self.mass_struct))/m

        M1 = Transformer().body_to_earth([psi, theta, phi])                             ## transform metrix rocket -> ground
        con_vec = Transformer().body_to_earth(np.array([alpha, beta, gamma]))           ## transform metrix motor -> rocket

        T = con_vec@T                                                                   ## transform trust motor -> rocket
        T = M1@T                                                                        ## transform trust rocket -> ground
                                                   
        r_to_bottom = -self.total_mass_center*M1@[0,0,1]                                ## vector from mass center to bottom                

        torque = np.cross(r_to_bottom,T)                                                ## calcul torque by trust




        v = np.array([vx,  vy,  vz])                                                    ## velocity
        p = np.array([px,  py,  pz])                                                    ## position
        w = np.array([wx,  wy,  wz])                                                    ## angular velocity
        g = np.array([ 0,   0, 9.8])                                                    ## gravity
        v_aerodynamic = M1@[0,0,self.totla_aero_center-self.total_mass_center]          ## vector from mass center to aero center

        S = np.pi*self.diameter*self.diameter/4                             ## cross-sectional area
        rho = 1.225*(1-2.256e-5*pz)**5.256                                  ## calculate atmospheric density
        D = -0.5*rho*S*self.Cd*np.linalg.norm(v)*v                          ## calcul drag force
        torque_of_drag= np.cross(v_aerodynamic,D)                           ## calcul torque by drag


        wx_dot, wy_dot, wz_dot = (torque+torque_of_drag)/m                  ## differential equation of angular acceleration
        theta_dot, phi_dot, psi_dot = w*180/np.pi                           ## differential equation of angul
        m_dot = -self.mass_pro/self.t_b                                     ## differential equation of propellent mass
        vx_dot, vy_dot, vz_dot = (T+D)/m -g                                 ## differential equation of velocity of rocket

        if vz_dot<=0:                                                       ## rocket can't go under the ground
            vz_dot =0

        px_dot, py_dot, pz_dot = v                                          ## differential equation of position of rocket
        self.w_dot = np.array([wx_dot,wy_dot,wz_dot])                       ## save rocket w_dot
        self.accel = np.array([vx_dot,vy_dot,vz_dot])                       ## save rocket acceleration
        

        return np.array([vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot,psi_dot, theta_dot, phi_dot, wx_dot, wy_dot, wz_dot])

        
    ## free fall
    def force_free(self,zeroparam,t):
        vx, vy, vz, px, py, pz, m, psi, theta, phi,wx, wy, wz= zeroparam
        v = np.array([vx,  vy,  vz])
        p = np.array([px,  py,  pz])
        g = np.array([ 0,   0, 9.8])
        w = np.array([wx,  wy,  wz])
        self.total_mass_center = (self.mass_center[0]*self.mass_struct+self.mass_center[1]*(m-self.mass_struct))/m


        M1 = Transformer().body_to_earth(np.array([psi, theta, phi]))        
        v_aerodynamic = M1@[0,0,self.totla_aero_center-self.total_mass_center]


        S = np.pi*self.diameter*self.diameter/4
        rho = 1.225*(1-2.256e-5*pz)**5.256
        D = -0.5*rho*S*self.Cd*np.linalg.norm(v)*v 
        torque_of_drag= np.cross(v_aerodynamic,D)

        
        wx_dot, wy_dot, wz_dot =  torque_of_drag/m                         
        theta_dot, phi_dot, psi_dot = w*180/np.pi
        m_dot = 0
        vx_dot, vy_dot, vz_dot = D/m -g
        px_dot, py_dot, pz_dot = v
        self.w_dot = np.array([wx_dot,wy_dot,wz_dot])
        self.accel = np.array([vx_dot,vy_dot,vz_dot])
        # print(psi,theta,phi)
        # if pz < 0:
        #     vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot, psi_dot, theta_dot, phi_dot, alpha_dot, beta_dot, wx_dot, wy_dot, wz_dot = \
        #     [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        return np.array([vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot, psi_dot, theta_dot, phi_dot,wx_dot, wy_dot, wz_dot])


    def calcul_force_effect(self):

        ## in burnning
        if realTime < self.t_b:
            t1 = np.linspace(0,0.05,11)

            self.motor_angle = self.tvc_motor_angle

            for i in [0,1,2]:
                if self.motor_angle[i] > 3:
                    self.motor_angle[i] = 3
                elif self.motor_angle[i] < -3:
                    self.motor_angle[i] = -3
            print('motor angle: ',self.motor_angle)

            self.force_effect = odeint(self.force_burn,self.zeroparam,t1,tuple(self.motor_angle))


        ## free fall
        else :
            t2 = np.linspace(0,0.05,11)
            self.force_effect = odeint(self.force_free,self.zeroparam,t2)


    def readTvc(self,msg):
        self.tvc_motor_angle = np.array(msg.data)

        rospy.sleep(0.05)        





if __name__ == '__main__':
    realTime = 0                                # Set real Time 0s
    drag_coeff = 0.4                            # drag coefficient
    mass_struct = 3                             # structure mass (kg)
    mass_pro = 0.5                              # propellent mass (kg)
    position = np.array([0,0,0])                # rocket initial posision (x,y,z (m))
    velocity = np.array([0,0,0])                # rocket initial velocity (x,y,z (m))
    angular_velocity = np.array([0,0,0])        # rocket initial angular velocity (r,p,y (rad/s))
    rocket_angle = [0,0,10]                     # rocket initial angle (r,p,y ('))
    motor_angle = np.array([0,0,0])             # rocket initial motor angle (r,p,y ('))
    rocket_length = 1.2                         # rocket length (m)
    mass_center = [0.6,0.1]                     # rocket mass center of structure, propellant (m from bottom)
    aerocenter = 0.5
    diameter = 0.09                             # rocket outside diameter (m)
    thrust = np.array([0,0,100])                # thrust (N)
    burnTime = 10                               # burnning time (s)

    rocket = Rocket_system(mass_struct,mass_pro,burnTime,drag_coeff,position,velocity,angular_velocity,\
                           rocket_angle,motor_angle,rocket_length,mass_center,aerocenter,diameter,thrust)

    rospy.init_node('simulation')
    pub = rospy.Publisher('data',Float32MultiArray,queue_size=10)
    rospy.Subscriber('Actuator',Float32MultiArray,rocket.readTvc)        # Tvc on

    fig = plt.figure(facecolor='w')
    # ax = fig.add_subplot(111,projection='3d')                 ## 3d plot
    ax = fig.add_subplot(1,1,1)                                 ## 2d plot
    # ax2 = ax.twinx()                                            ## set second y-axis

    rocket_shape = []                                           ## set rocket shape
    rocket_pos_list = np.empty((0,3))                           ## save all position for plotting
    rocket_velocity_list = np.empty((0,3))                      ## save all velocity
    rocket_angle_list = np.empty((0,3))                         ## save all rocket angle
    rocket_accel_list = np.empty((0,3))                         ## save all rocket accle

    Apogee = 'None'
    arrive_time = 0                                             ## Apogee time
    eject_num = 0                                               ## ejection number
    Max_acceleration = 0
    Max_velocity = 0
    Max_x = 0

    while realTime <= 10:                                      ## flight time (s)

        data = Float32MultiArray()
        angle_data = rocket.zeroparam[7:10]
        w_data = rocket.zeroparam[10:13]
        data.data = np.append(angle_data,w_data,axis=0)

        pub.publish(data)

        rospy.sleep(0.1)

        rocket.calcul_force_effect()
        rocket.zeroparam = rocket.force_effect[-1]
        print(rocket.zeroparam[5])


        rospy.sleep(0.01)

        rocket_angle_list = np.append(rocket_angle_list,np.array([[rocket.zeroparam[7],rocket.zeroparam[8],rocket.zeroparam[9]]]),axis=0)




        M2 = Transformer().body_to_earth(np.array([rocket.force_effect[-1,7], rocket.force_effect[-1,8], rocket.force_effect[-1,9]]))
        rocket_pos_vector = 20*M2@[0,0,1]                               ## rocket body vector in ground (half size of rocket)

        if rocket.zeroparam[2] < 0 and rocket.zeroparam[5] > 100 and eject_num == 0:    ## calculate Apogee / ejection
            Apogee = rocket.zeroparam[5]
            eject_num +=1

        if rocket.zeroparam[3] > Max_x:                             ## calculate Max x
            Max_x = rocket.zeroparam[3]

        if rocket.zeroparam[2] > Max_velocity:                      ## calculate Max velocity (z)
            Max_velocity = rocket.zeroparam[2]

        if rocket.accel[2] > Max_acceleration:                      ## calculate Max accleration (z)
            Max_acceleration = rocket.accel[2]

        # if math.ceil(rocket.zeroparam[9]) == 40:                    
        #     arrive_time = realTime

        rocket_accel_list = np.append(rocket_accel_list,np.array([[rocket.accel[0],rocket.accel[1],rocket.accel[2]]]),axis=0)
        rocket_velocity_list = np.append(rocket_velocity_list,np.array([[rocket.force_effect[-1,0],rocket.force_effect[-1,1],rocket.force_effect[-1,2]]]),axis=0)
        rocket_pos_list = np.append(rocket_pos_list,np.array([[rocket.force_effect[-1,3],rocket.force_effect[-1,4],rocket.force_effect[-1,5]]]),axis=0)

        rocket_pos_center = np.array([rocket.force_effect[-1,3],rocket.force_effect[-1,4],rocket.force_effect[-1,5]])
        rocket_shape.append([[rocket_pos_center[0]-rocket_pos_vector[0],rocket_pos_center[0]+rocket_pos_vector[0]],
                            [rocket_pos_center[1]-rocket_pos_vector[1],rocket_pos_center[1]+rocket_pos_vector[1]],
                            [rocket_pos_center[2]-rocket_pos_vector[2],rocket_pos_center[2]+rocket_pos_vector[2]]])
        # rocket shape : center of mass - body vector    ~~    center of mass + body vector

        realTime += 0.05


    ## make animation
    # def animate(i):
        # ax.clear()
        # ax2.clear()                                                    # initialization
        # ax.set_xlim(0, 45000)
        # ax.set_ylim(0, 45000)
        # ax.set_zlim(0, 1000)

        # ax.view_init(elev=90., azim=60)                               # set view angle
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

    # animate = animation.FuncAnimation(fig,animate, frames = 400, interval=1)                                # make animation
    ax.set_xlim(0,20)
    ax.set_xlabel('time')
    ax.set_ylim(-90,90)
    ax.set_ylabel('angle')
    # ax2.set_ylim(-100,2000)
    # ax2.set_ylabel('Vertical Velocity, acceleration')
    # x = np.linspace(0,10,201)
    x = np.arange(0,10,0.05)
    # ax.plot(x,x*0,linestyle='--')
    y = rocket_angle_list[:,2]
    # x = rocket_pos_list[:,0]
    # y = rocket_pos_list[:,2]
    ax.plot(x,y)
    plt.axhline(y=20)
    # ax2.plot(x,rocket_velocity_list[:,2],'b')
    # ax2.plot(x,rocket_accel_list[:,2],'g')
    # ax.text(1000,3000,'Apogee : %.1fm'%Apogee)
    # ax.text(1000,4000,'Max x : %.1fm'%Max_x)
    # ax.text(23,400,'Max velocity : %.1fm/s'%Max_velocity)
    # ax.text(23,500,'Max acceleration : %.1fm/s'%Max_acceleration)

    # plt.ylabel('plot')
    # plt.xlabel('time')
    # plt.axhline(y=50)
    plt.show()
    plt.rcParams['font.size'] = 10
    print('---saving---please wait---about 30 second---')
    # animate.save("Tvc(yow : 20', pich : 20').mp4",fps=20)              # save, 1 frame => 0.05s ( 30 fps = 1.5 real fps)
    print('---finish---check your folder which this file is in---')