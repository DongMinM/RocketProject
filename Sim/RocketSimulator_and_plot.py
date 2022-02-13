import math

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
        position = np.array([0,0,0]),
        velocity = np.array([0,0,0]),
        angular_velocity = np.array([0,0,0]),
        rocket_angle = [0,0,0],                    ## r , p , y
        motor_angle = [0,0,0],
        rocket_length = 1,
        mass_center = [0.45,0.1]
    ):
        self.mass_struct = mass_struct              ## structure mass
        self.mass_pro = mass_pro                    ## propellent mass
        self.t_b = t_b                              ## brunning time
        self.drag = drag_coeff                      ## drag coefficient
        self.position = position                    ## rocket body position         related to ground
        self.velocity = velocity                    ## rocket body velocity         related to ground
        self.angular_velocity = angular_velocity    ## rocket body angular velocity related to ground
        self.rocket_angle = rocket_angle            ## rocket body angle            related to ground
        self.motor_angle = motor_angle              ## rocket motor angle           related to rocket
        self.force_effect = []                      ## params by force effect
        self.mass = mass_pro+mass_struct            ## total rocket mass
        self.mass_center = mass_center
        self.rocket_length = rocket_length
        self.total_mass_center = 0.5
        self.totla_aero_center = 0.37
        self.T = 0
        self.angle_dot = [0,0,0]
        self.zeroparam = np.hstack((self.velocity, self.position, self.mass, self.rocket_angle,self.motor_angle,self.angular_velocity))
        self.w_dot = np.array([0,0,0])
        self.accel = np.empty((0,3))

    ## in burnning
    def force_burn(self,zeroparam,t,b):
        # T = [0,0,500/self.t_b*math.sin(math.pi/self.t_b*realTime)]      ## impulse 200Ns trust - sin shape
        T = np.array([0,0,195])
        vx, vy, vz, px, py, pz, m, psi, theta, phi,alpha,beta,gamma, wx, wy, wz = zeroparam
        self.total_mass_center = (self.mass_center[0]*self.mass_struct+self.mass_center[1]*(m-self.mass_struct))/m
        self.T = T

        if beta > 5:
            beta = 5
        elif beta < -5 :
            beta = -5

        if gamma > 5 :
            gamma = 5
        elif gamma < -5:
            gamma = -5
        
        # print(alpha,beta,gamma)
        # print(m)
        M1 = Transformer().body_to_earth([psi, theta, phi])       ## transform metrix rocket -> ground
        # print(psi,theta,phi)
        con_vec = Transformer().body_to_earth(np.array([alpha, beta, gamma]))             ## transform metrix motor -> rocket

        # print(1)
        T = con_vec@T
        T = M1@T                                                     ## trust in rocket system  
        # print(T)                                                      ## trust in ground system
        r_to_bottom = -self.total_mass_center*M1@[0,0,1]                                          
        r_to_top = (self.rocket_length-self.total_mass_center)*M1@[0,0,1]
          ## rocket body (unit)vector (top - center of mass)
        torque = np.cross(r_to_bottom,T) ## r,p,y                        ## torque = size*r unit vector X trust vector ( torque by trust)
        torque = np.round(torque,10)
        # print(torque)
        

        # print(torque)



        v = np.array([vx,  vy,  vz])                                        ## velocity
        
        p = np.array([px,  py,  pz])                                        ## position
        w = np.array([wx,  wy,  wz])                                        ## angular velocity
        g = np.array([ 0,   0, 9.8])                                        ## gravity
        v_aerodynamic = M1@[0,0,self.totla_aero_center-self.total_mass_center]
        # print(v_aerodynamic)
        b = b                                                               ## drag coefficient
        d = 0.16

        
        S = np.pi*d*d/4
        rho = 1.225*(1-2.256e-5*pz)**5.256
        # rho = 1.225
        D = -0.5*rho*S*Cd*np.linalg.norm(v)*v 
        torque_of_drag= np.cross(v_aerodynamic,D)
        # print(torque_of_drag)
        # print(v)
        # print(T)
        # v_total1 = v+np.cross(w,r_to_top)
        # v_total2 = v+np.cross(w,r_to_bottom)
        # D1 = -0.5*rho*S*Cd*np.linalg.norm(v_total1)*v_total1 
        # D2 = -0.5*rho*S*Cd*np.linalg.norm(v_total2)*v_total2 
        # D3 = -0.5*rho*S*Cd*np.linalg.norm(v)*v
        # torque_of_drag1= np.cross(r_to_top,D1)
        # torque_of_drag2=np.cross(r_to_bottom,D2)
        # print(1)

        wx_dot, wy_dot, wz_dot = (torque+torque_of_drag)/m                           ## differential equation of angular acceleration
        theta_dot, phi_dot, psi_dot = w                                     ## differential equation of angul of rocket body
        m_dot = -self.mass_pro/self.t_b                                     ## differential equation of propellent mass
        vx_dot, vy_dot, vz_dot = (T+D)/m -g                             ## differential equation of velocity of rocket
        if vz_dot<=0:
            vz_dot =0
        px_dot, py_dot, pz_dot = v                                          ## differential equation of position of rocket
        alpha_dot,beta_dot,gamma_dot = self.angle_dot                                          ## differential equation of motor angul (TVC)
        # print(self.angle_dot)
        # alpha_dot,beta_dot,gamma_dot = 0,0,0
        self.w_dot = np.array([wx_dot,wy_dot,wz_dot])
        self.accel = np.array([vx_dot,vy_dot,vz_dot])

        return np.array([vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot,psi_dot, theta_dot, phi_dot,alpha_dot,beta_dot, gamma_dot, wx_dot, wy_dot, wz_dot])
        ## return params
        
    ## free fall
    def force_free(self,zeroparam,t,b):
        vx, vy, vz, px, py, pz, m, psi, theta, phi, alpha, beta, gamma, wx, wy, wz= zeroparam
        v = np.array([vx,  vy,  vz])
        p = np.array([px,  py,  pz])
        g = np.array([ 0,   0, 9.8])
        w = np.array([wx,  wy,  wz])
        self.total_mass_center = (self.mass_center[0]*self.mass_struct+self.mass_center[1]*(m-self.mass_struct))/m
        # print(m)
        if pz < 0:
            vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot, psi_dot, theta_dot, phi_dot, alpha_dot, beta_dot, wx_dot, wy_dot, wz_dot = \
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        # else :
        M1 = Transformer().body_to_earth(np.array([psi, theta, phi]))
        r_to_bottom = -(self.rocket_length-self.total_mass_center)*M1@[0,0,1]
        r_to_top = (self.rocket_length-self.total_mass_center)*M1@[0,0,1]
        v_aerodynamic = M1@[0,0,self.totla_aero_center-self.total_mass_center]
        # print(v_aerodynamic)
        # print(v_aerodynamic)
        b = b
        d = 0.16
        S = np.pi*d*d/4
        # print(pz)
        rho = 1.225*(1-2.256e-5*pz)**5.256
        # v_total1 = v+np.cross(w,r_to_top)
        # v_total2 = v+np.cross(w,r_to_bottom)
        # D1 = -0.5*rho*S*Cd*np.linalg.norm(v_total1)*v_total1 
        # D2 = -0.5*rho*S*Cd*np.linalg.norm(v_total2)*v_total2 
        # D3 = -0.5*rho*S*Cd*np.linalg.norm(v)*v
        # torque_of_drag1= np.cross(r_to_top,D1)
        # torque_of_drag2=np.cross(r_to_bottom,D2)
        # print(torque_of_drag1)

        S = np.pi*d*d/4
        rho = 1.225*(1-2.256e-5*pz)**5.256
        # rho = 1.225
        D = -0.5*rho*S*Cd*np.linalg.norm(v)*v 
        # print(np.linalg.norm(v))
        torque_of_drag= np.cross(v_aerodynamic,D)
        # print(torque_of_drag)
        # print(v)
        
        # print('------------')
        wx_dot, wy_dot, wz_dot =  torque_of_drag/m                         
        theta_dot, phi_dot, psi_dot = w
        m_dot = 0
        vx_dot, vy_dot, vz_dot = D/m -g
        # print(vz_dot)
        px_dot, py_dot, pz_dot = v
        alpha_dot, beta_dot, gamma_dot = [0,0,0]
        self.w_dot = np.array([wx_dot,wy_dot,wz_dot])
        self.accel = np.array([vx_dot,vy_dot,vz_dot])
        # if pz < 0:
        #     vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot, psi_dot, theta_dot, phi_dot, alpha_dot, beta_dot, wx_dot, wy_dot, wz_dot = \
        #     [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        return np.array([vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot, psi_dot, theta_dot, phi_dot,alpha_dot,beta_dot,gamma_dot,wx_dot, wy_dot, wz_dot])


    def calcul_force_effect(self):
        ## in burnning
        if realTime < self.t_b:
            t1 = np.linspace(0,0.05,11)

            self.force_effect = odeint(self.force_burn,self.zeroparam,t1,args=(self.drag,))

        ## free fall
        else :
            t2 = np.linspace(0,0.05,11)
            # self.total_mass_center = (self.mass_center[0]*self.mass_struct+self.mass_center[1]*(m-self.mass_struct))/m
            # self.total_mass_center = (self.mass_center[0]*self.mass_struct+self.mass_center[1]*(self.zeroparam[6]-self.mass_struct))/self.zeroparam[6]
            self.force_effect = odeint(self.force_free,self.zeroparam,t2,args=(self.drag,))

    def callback(self,msg):
        # print(msg.data)
        self.angle_dot = msg.data
        # print(self.angle_dot)





if __name__ == '__main__':
    Cd = 0.122
    t = np.linspace(0,0.05,11)      # plotting time space
    drag_coeff = 0.3                # drag coefficient
    mass_struct = 2.343                 # structure mass
    mass_pro = 0.229                 # propellent mass
    burnTime = 2.1                  # burnning time
    realTime = 0
    rocket = Rocket_system(mass_struct,mass_pro,burnTime,drag_coeff)

    rospy.init_node('simulation')
    pub = rospy.Publisher('data',Float32MultiArray,queue_size=10)


    fig = plt.figure(facecolor='w')
    # ax = fig.add_subplot(111,projection='3d')                 ## 3d plot
    ax = fig.add_subplot(1,1,1)                                 ## 2d plot
    ax2 = ax.twinx()
    rocket_shape = []                                       
    rocket_pos_list = np.empty((0,3))                           ## save all position for plotting
    rocket_velocity_list = np.empty((0,3))
    Apogee = 'None'
    eject_num = 0
    rocket_angle_plot = np.empty((0,3))
    rocket_accel_list = np.empty((0,3))
    i = 0
    arrive_time = 0
    Max_acceleration = 0
    Max_velocity = 0
    rospy.Subscriber('Actuator',Float32MultiArray,rocket.callback)        # tvc on


    while realTime <= 40:
        rocket.calcul_force_effect()
        rocket.zeroparam = rocket.force_effect[-1]
        # for i in [10,11]:
        #         if rocket.zeroparam[i] >5:
        #             rocket.zeroparam[i] = 5
                    
        #         elif rocket.zeroparam[i] < -5:
        #             rocket.zeroparam[i] = -5
        # print(rocket.zeroparam)

        data = Float32MultiArray()
        data.data = np.append(rocket.zeroparam,rocket.w_dot,axis=0)
        pub.publish(data)
        # print(rocket.w_dot)
        rospy.sleep(0.01)

        # print(realTime)
        rocket_angle_plot = np.append(rocket_angle_plot,np.array([[rocket.zeroparam[7],rocket.zeroparam[8],rocket.zeroparam[9]]]),axis=0)

        realTime += 0.05



        M2 = Transformer().body_to_earth(np.array([rocket.force_effect[-1,7], rocket.force_effect[-1,8], rocket.force_effect[-1,9]]))
        rocket_pos_vector = 100*M2@[0,0,1]                               ## rocket body vector in ground (half size of rocket)

        if rocket.zeroparam[2] < 0 and rocket.zeroparam[5] > 100 and eject_num == 0:
            Apogee = rocket.zeroparam[5]
            eject_num +=1
        if rocket.zeroparam[2] > Max_velocity:
            Max_velocity = rocket.zeroparam[2]

        if rocket.accel[2] > Max_acceleration:
            Max_acceleration = rocket.accel[2]

        if math.ceil(rocket.zeroparam[9]) == 40:
            arrive_time = realTime

        rocket_accel_list = np.append(rocket_accel_list,np.array([[rocket.accel[0],rocket.accel[1],rocket.accel[2]]]),axis=0)
        rocket_pos_center = np.array([rocket.force_effect[-1,3],rocket.force_effect[-1,4],rocket.force_effect[-1,5]])
        rocket_velocity_list = np.append(rocket_velocity_list,np.array([[rocket.force_effect[-1,0],rocket.force_effect[-1,1],rocket.force_effect[-1,2]]]),axis=0)
        rocket_pos_list = np.append(rocket_pos_list,np.array([[rocket.force_effect[-1,3],rocket.force_effect[-1,4],rocket.force_effect[-1,5]]]),axis=0)
        rocket_shape.append([[rocket_pos_center[0]-rocket_pos_vector[0],rocket_pos_center[0]+rocket_pos_vector[0]],
                             [rocket_pos_center[1]-rocket_pos_vector[1],rocket_pos_center[1]+rocket_pos_vector[1]],
                             [rocket_pos_center[2]-rocket_pos_vector[2],rocket_pos_center[2]+rocket_pos_vector[2]]])
        # rocket shape : center of mass - body vector    ~~    center of mass + body vector
    def animate(i):
        ax.clear()
        ax2.clear()                                                                                          # initialization
        # ax.set_xlim(-1500, 1500)
        # ax.set_ylim(-1500, 1500)
        # ax.set_zlim(0, 1500)
        # ax.view_init(elev=90., azim=60)                                                   # set view angle
        ax.set_xlim(0,40)
        ax.set_xlabel('time')
        ax.set_ylim(-100,800)
        ax.set_ylabel('altitude')
        ax2.set_ylim(-30,150)
        ax2.set_ylabel('Vertical Velocity, acceleration')                                                               
        
        # x = rocket_shape[i][0]
        # y = rocket_shape[i][1]
        # z = rocket_shape[i][2]

        # ax.plot(rocket_pos_list[:i,0],rocket_pos_list[:i,1],rocket_pos_list[:i,2],'b-',label = '1st')       # plot line
        # time = i*0.05
        # ax.text(-100,-100,0,'Time = %.1fs'%time)                                                           # plot time
        # ax.text(-80,-80,100,'Altitude = %.1fm'%rocket_pos_list[i,2])

        # if rocket_velocity_list[i,2] < 0 and rocket_pos_list[i,2] > 100:
        #     ax.text(-100,-100,200,'Eject Altitude = %.1fm'%Apogee)

        x = np.linspace(0,i*0.05,i+1)
        y = 90-rocket_angle_plot[0:i+1,2]
        y = rocket_pos_list[0:i+1,2]
        y2 = rocket_velocity_list[0:i+1,2]
        y3 = rocket_accel_list[0:i+1,2]
        
        
        # plt.plot.axhline(y=50)
        # if rocket_angle_plot[i,2] > 40:
            # plt.axvline(x=arrive_time)
            # ax.text(2,45,'arrive time : %.1fs'%arrive_time,)

        # return ax.plot(x,y,z,color = 'k', lw = 2)  
        
        ax2.plot(x,x*0,linestyle = '--')
        ax2.plot(x,y2,'b',lw=2)
        ax2.plot(x,y3,'g',lw=2)
        return ax.plot(x,y,'r',lw=2)                                                         # plot rocket

    # animate = animation.FuncAnimation(fig,animate, frames = 800, interval=3)                                # make animation
    ax.set_xlim(0,40)
    ax.set_xlabel('time')
    ax.set_ylim(-100,800)
    ax.set_ylabel('altitude')
    ax2.set_ylim(-30,150)
    ax2.set_ylabel('Vertical Velocity, acceleration')
    x = np.linspace(0,40,801)
    # ax.plot(x,x*0,linestyle='--')
    # ax2.plot(x,x*0,linestyle='--')
    ax.plot(x,rocket_pos_list[:,2],'r')
    ax2.plot(x,rocket_velocity_list[:,2],'b')
    ax2.plot(x,rocket_accel_list[:,2],'g')
    ax.text(23,300,'Apogee : %.1fm'%Apogee)
    ax.text(23,400,'Max velocity : %.1fm/s'%Max_velocity)
    ax.text(23,500,'Max acceleration : %.1fm/s'%Max_acceleration)

    # plt.ylabel('plot')
    # plt.xlabel('time')
    # plt.axhline(y=50)
    plt.show()
    plt.rcParams['font.size'] = 10
    print('---saving---please wait---about 30 second---')
    # animate.save("Tvc(yow : 20', pich : 20').mp4",fps=20)                                                            # save, 1 frame => 0.05s ( 30 fps = 1.5 real fps)
    print('---finish---check your folder which this file is in---')