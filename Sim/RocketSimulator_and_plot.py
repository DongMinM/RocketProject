import math
from transform import Transformer
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import axes3d
from tvc2 import Tvc


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
        rocket_angle = [20,20,0],                    ## p, y, r
        motor_angle = [0,0,0],
        rocket_length = 1,
        mass_center = [0.5,0.1]
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
        self.T = 0
        self.angle_dot = [0,0,0]
        self.zeroparam = np.hstack((self.velocity, self.position, self.mass, self.rocket_angle,self.motor_angle,self.angular_velocity))
        self.w_dot = np.array([0,0,0])

    ## in burnning
    def force_burn(self,zeroparam,t,b):
        # T = [0,0,400/self.t_b*math.sin(math.pi/self.t_b*realTime)]      ## impulse 200Ns trust - sin shape
        T = [0,0,200]
        vx, vy, vz, px, py, pz, m, psi, theta, phi,alpha,beta,gamma, wx, wy, wz = zeroparam
        self.total_mass_center = (self.mass_center[0]*self.mass_struct+self.mass_center[1]*(self.zeroparam[6]-self.mass_struct))/self.zeroparam[6]
        self.T = T
        # print(alpha,beta)

        M1 = Transformer().body_to_earth(np.array([psi, theta, phi]))       ## transform metrix rocket -> ground
        con_vec = Transformer().body_to_earth(np.array([alpha, beta, 0]))             ## transform metrix motor -> rocket


        T = con_vec@T
        T = M1@T                                                       ## trust in rocket system                                                            ## trust in ground system
        r_to_bottom = -self.total_mass_center*M1@[0,0,1]                                            ## rocket body (unit)vector (top - center of mass)
        torque = np.cross(r_to_bottom,T) ## r,p,y                        ## torque = size*r unit vector X trust vector ( torque by trust)

        # print(torque)



        v = np.array([vx,  vy,  vz])                                        ## velocity
        
        p = np.array([px,  py,  pz])                                        ## position
        w = np.array([wx,  wy,  wz])                                        ## angular velocity
        g = np.array([ 0,   0, 9.8])                                        ## gravity
        b = b                                                               ## drag coefficient
        d = 0.16

        torque_of_g = np.cross((self.rocket_length/2-self.total_mass_center)*r_to_bottom,-g)
        
        S = np.pi*d*d/4
        # rho = 1.225*(1-2.256e-5*pz)**5.256
        rho = 1.225
        k1 = 0.5*rho*S*Cd*np.linalg.norm(v)
        k2 = 0.5*rho*S*Cd*np.linalg.norm(w)



        wx_dot, wy_dot, wz_dot = (torque-k2*w)/m+torque_of_g                            ## differential equation of angular acceleration
        psi_dot, theta_dot, phi_dot = w                                     ## differential equation of angul of rocket body
        m_dot = -self.mass_pro/self.t_b                                     ## differential equation of propellent mass
        vx_dot, vy_dot, vz_dot = (T -k1*v)/m -g                             ## differential equation of velocity of rocket
        if vz_dot<=0:
            vz_dot =0
        px_dot, py_dot, pz_dot = v                                          ## differential equation of position of rocket
        # alpha_dot,beta_dot,gamma_dot = self.angle_dot                                          ## differential equation of motor angul (TVC)
        
        alpha_dot,beta_dot,gamma_dot = self.angle_dot
        self.w_dot = np.array([wx_dot,wy_dot,wz_dot])

        return np.array([vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot,psi_dot, theta_dot, phi_dot,alpha_dot,beta_dot, gamma_dot, wx_dot, wy_dot, wz_dot])
        ## return params
        
    ## free fall
    def force_free(self,zeroparam,t,b):
        vx, vy, vz, px, py, pz, m, psi, theta, phi, alpha, beta, gamma, wx, wy, wz= zeroparam
        v = np.array([vx,  vy,  vz])
        p = np.array([px,  py,  pz])
        g = np.array([ 0,   0, 9.8])
        w = np.array([wx,  wy,  wz])

        # if pz < 0:
        #     vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot, psi_dot, theta_dot, phi_dot, alpha_dot, beta_dot, wx_dot, wy_dot, wz_dot = \
        #     [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        # else :
        M1 = Transformer().body_to_earth(np.array([psi, theta, phi]))
        r_to_bottom = -M1@[0,0,1]

        b = b
        d = 0.16
        S = np.pi*d*d/4
        # print(pz)
        rho = 1.225*(1-2.256e-5*pz)**5.256
        k1 = 0.5*rho*S*Cd*np.linalg.norm(v)
        k2 = 0.5*rho*S*Cd*np.linalg.norm(w)
        torque_of_g = np.cross((self.rocket_length/2-self.total_mass_center)*r_to_bottom,-g)
        # print('------------')
        wx_dot, wy_dot, wz_dot = torque_of_g-k2*w/m                                
        psi_dot, theta_dot, phi_dot = w
        m_dot = 0
        vx_dot, vy_dot, vz_dot = -k1*v/m -g
        px_dot, py_dot, pz_dot = v
        alpha_dot, beta_dot, gamma_dot = [0,0,0]
        # if pz < 0:
        #     vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot, psi_dot, theta_dot, phi_dot, alpha_dot, beta_dot, wx_dot, wy_dot, wz_dot = \
        #     [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        return np.array([vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot,psi_dot, theta_dot, phi_dot,alpha_dot,beta_dot,gamma_dot,wx_dot, wy_dot, wz_dot])


    def calcul_force_effect(self):
        ## in burnning
        if realTime < self.t_b:
            t1 = np.linspace(0,0.05,11)
            self.force_effect = odeint(self.force_burn,self.zeroparam,t1,args=(self.drag,))

        ## free fall
        else :
            t2 = np.linspace(0,0.05,11)
            self.total_mass_center = (self.mass_center[0]*self.mass_struct+self.mass_center[1]*(self.zeroparam[6]-self.mass_struct))/self.zeroparam[6]
            self.force_effect = odeint(self.force_free,self.zeroparam,t2,args=(self.drag,))






if __name__ == '__main__':
    Cd = 0.2
    t = np.linspace(0,0.05,11)      # plotting time space
    drag_coeff = 0.3                # drag coefficient
    mass_struct = 2                 # structure mass
    mass_pro = 0.15                 # propellent mass
    burnTime = 15                    # burnning time
    realTime = 0
    rocket = Rocket_system(mass_struct,mass_pro,burnTime,drag_coeff)


    fig = plt.figure(facecolor='w')
    # ax = fig.add_subplot(111,projection='3d')                 ## 3d plot
    ax = fig.add_subplot(1,1,1)                                 ## 2d plot
    rocket_shape = []                                       
    rocket_pos_list = np.empty((0,3))                           ## save all position for plotting
    rocket_velocity_list = np.empty((0,3))
    eject_altitude = 'None'
    eject_num = 0
    rocket_angle_plot = np.empty((0,3))
    i = 0
    while realTime<=20:
        rocket.calcul_force_effect()
        ## 여기에 TVC 함수 들어가야 함. psi, theta, phi, alpha, beta, angular velocity 등 주고 받으면 될듯
        rocket.zeroparam = rocket.force_effect[-1]
        # print(rocket.zeroparam[10:12])
        # print(rocket.zeroparam)
        # print(realTime)
        rocket_angle_plot = np.append(rocket_angle_plot,np.array([[rocket.zeroparam[7],rocket.zeroparam[8],rocket.zeroparam[9]]]),axis=0)
        # print(rocket_angle)
        if realTime < burnTime:
        # if i in list(range(1,600,10)):
        
        # if realTime in mylist:
            # print(realTime)
            # print(rocket.T)
            # print(rocket.zeroparam[12:15])
            # print(rocket.zeroparam[7:10])
            rocket.angle_dot = Tvc(rocket.zeroparam,rocket.w_dot,rocket.total_mass_center,rocket.T,[0,0,0]).run()
            # rocket.zeroparam[10],rocket.zeroparam[11]= Tvc(rocket.zeroparam,rocket.w_dot,rocket.total_mass_center,rocket.T,[0,0,0]).run()
            # print(rocket.angle_dot)
            # rocket.angle_dot[0] = rocket.angle_dot[0]
            # rocket.angle_dot[1] = rocket.angle_dot[1]
            # print(rocket.angle_dot)
            # print(angle_need)
        # print(rocket.zeroparam)
        # print(rocket.total_mass_center)
        if rocket.zeroparam[2] < 0 and rocket.zeroparam[5] > 100 and eject_num == 0:
            # print(1)
            eject_altitude = rocket.zeroparam[5]
            eject_num +=1
        i+=1

        realTime += 0.05

        M2 = Transformer().body_to_earth(np.array([rocket.force_effect[-1,7], rocket.force_effect[-1,8], rocket.force_effect[-1,9]]))
        rocket_pos_vector = 20*M2@[0,0,1]                               ## rocket body vector in ground (half size of rocket)
        # print(rocket_pos_vector)


        rocket_pos_center = np.array([rocket.force_effect[-1,3],rocket.force_effect[-1,4],rocket.force_effect[-1,5]])
        rocket_velocity_list = np.append(rocket_velocity_list,np.array([[rocket.force_effect[-1,0],rocket.force_effect[-1,1],rocket.force_effect[-1,2]]]),axis=0)
        rocket_pos_list = np.append(rocket_pos_list,np.array([[rocket.force_effect[-1,3],rocket.force_effect[-1,4],rocket.force_effect[-1,5]]]),axis=0)
        rocket_shape.append([[rocket_pos_center[0]-rocket_pos_vector[0],rocket_pos_center[0]+rocket_pos_vector[0]],
                             [rocket_pos_center[1]-rocket_pos_vector[1],rocket_pos_center[1]+rocket_pos_vector[1]],
                             [rocket_pos_center[2]-rocket_pos_vector[2],rocket_pos_center[2]+rocket_pos_vector[2]]])
        ## rocket shape : center of mass - body vector    ~~    center of mass + body vector
    # print(eject_plot_code)
    def animate(i):
        ax.clear()                                                                                          # initialization
        # ax.set_xlim(-500, 500)
        ax.set_xlim(0,20)
        ax.set_ylim(-50, 50)
        # ax.set_zlim(0, 500)
        # ax.view_init(elev=10., azim=60)                                                                    # set view angle


        # x = rocket_shape[i][0]
        # y = rocket_shape[i][1]
        # z = rocket_shape[i][2]

        # ax.plot(rocket_pos_list[:i,0],rocket_pos_list[:i,1],rocket_pos_list[:i,2],'b-',label = '1st')       # plot line
        # time = i*0.05
        # ax.text(-100,-100,0,'Time = %.1fs'%time)                                                           # plot time
        # ax.text(-80,-80,100,'Altitude = %.1fm'%rocket_pos_list[i,2])

        # # print(rocket_velocity_list[i,2],rocket_pos_list[i,2])
        # if rocket_velocity_list[i,2] < 0 and rocket_pos_list[i,2] > 100:
        #     ax.text(-100,-100,200,'Eject Altitude = %.1fm'%eject_altitude)
        # print(rocket_angle_plot[0:i+1,0])
        
        x = np.linspace(0,i*0.05,i+1)
        # x = 10
        # print(x)
        y = rocket_angle_plot[0:i+1,0]
        # y = rocket_angle_plot[i,0]

        # return ax.plot(x,y,z,color = 'k', lw = 2)  
        return ax.plot(x,y,lw=2)                                                         # plot rocket

    animate = animation.FuncAnimation(fig,animate, frames = 400, interval=3)                                # make animation
    
    plt.ylabel('plot')
    plt.xlabel('time')
    plt.show()
    plt.rcParams['font.size'] = 10
    print('---saving---please wait---about 30 second---')
    # animate.save('Rocket Simulation4.mp4',fps=20)                                                            # save, 1 frame => 0.05s ( 30 fps = 1.5 real fps)
    print('---finish---check your folder which this file is in---')