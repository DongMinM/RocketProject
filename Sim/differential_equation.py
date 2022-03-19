import numpy as np
from transform import Transformer
from scipy.integrate import odeint


class Differentail_equation:

    def __init__ (self,rocket):
        self.rocket = rocket

    ## in burnning
    def force_burn(self,zeroparam,t,alpha,beta,gamma):

        vx, vy, vz, px, py, pz, m, psi, theta, phi,wx, wy, wz = zeroparam
        T = self.rocket.T

        self.rocket.total_mass_center = (self.rocket.mass_center[0]*self.rocket.mass_struct+self.rocket.mass_center[1]*(m-self.rocket.mass_struct))/m

        M1 = Transformer().body_to_earth([psi, theta, phi])                             ## transform metrix rocket -> ground
        con_vec = Transformer().body_to_earth(np.array([alpha, beta, gamma]))           ## transform metrix motor -> rocket

        T = con_vec@T                                                                   ## transform trust motor -> rocket
        T = M1@T                                                                        ## transform trust rocket -> ground
                                                   
        r_to_bottom = -self.rocket.total_mass_center*M1@[0,0,1]                         ## vector from mass center to bottom                

        torque = np.cross(r_to_bottom,T)                                                ## calcul torque by trust




        v = np.array([vx,  vy,  vz])                                                    ## velocity
        p = np.array([px,  py,  pz])                                                    ## position
        w = np.array([wx,  wy,  wz])                                                    ## angular velocity
        g = np.array([ 0,   0, 9.8])                                                    ## gravity
        v_aerodynamic = M1@[0,0,self.rocket.total_aero_center-self.rocket.total_mass_center]          ## vector from mass center to aero center

        S = np.pi*self.rocket.diameter*self.rocket.diameter/4               ## cross-sectional area
        rho = 1.225*(1-2.256e-5*pz)**5.256                                  ## calculate atmospheric density
        D = -0.5*rho*S*self.rocket.Cd*np.linalg.norm(v)*v                   ## calculate drag force
        torque_of_drag= np.cross(v_aerodynamic,D)                           ## calculate torque of drag

        ''' differential equation ''' 
        '''w_dot = T / I
           angle_dot = w
           mass_dot = total mass / burnning time
           velocity_dot = F/m
           position_dot = velocity'''
        wx_dot, wy_dot, wz_dot = (torque+torque_of_drag)/(m*self.rocket.rocket_length**2)                  ## differential equation of angular acceleration
        theta_dot, phi_dot, psi_dot = w*180/np.pi                           ## differential equation of angul
        m_dot = -self.rocket.mass_pro/self.rocket.t_b                       ## differential equation of propellent mass
        vx_dot, vy_dot, vz_dot = (T+D)/m -g                                 ## differential equation of velocity of rocket
        if vz_dot<=0:                                                       ## rocket can't go under the ground
            vz_dot =0

        px_dot, py_dot, pz_dot = v                                          ## differential equation of position of rocket
        self.rocket.accel = np.array([vx_dot,vy_dot,vz_dot])                ## save rocket acceleration
        

        return np.array([vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot,psi_dot, theta_dot, phi_dot, wx_dot, wy_dot, wz_dot])

        
    ## free fall
    def force_free(self,zeroparam,t):
        vx, vy, vz, px, py, pz, m, psi, theta, phi,wx, wy, wz= zeroparam

        v = np.array([vx,  vy,  vz])                                                    ## 속도 (m/s)
        p = np.array([px,  py,  pz])                                                    ## 위치 (m/s)
        g = np.array([ 0,   0, 9.8])                                                    ## 중력 (m/s^2)
        w = np.array([wx,  wy,  wz])                                                    ## 각속도 (rad/s)
        self.rocket.total_mass_center = (self.rocket.mass_center[0] * self.rocket.mass_struct + self.rocket.mass_center[1] * (m-self.rocket.mass_struct)) / m
                                                                                        ## 무게 중심

        M1 = Transformer().body_to_earth(np.array([psi , theta , phi]))                 ## transform metrix rocket -> ground
        v_aerodynamic = M1@[ 0 , 0 , self.rocket.total_aero_center - self.rocket.total_mass_center]     ## aerodynamic center rocket -> ground


        S = np.pi*self.rocket.diameter*self.rocket.diameter/4                ## 단면적
        rho = 1.225*(1-2.256e-5*pz)**5.256                                   ## 대기 밀도
        D = -0.5*rho*S*self.rocket.Cd*np.linalg.norm(v)*v                    ## 항력
        torque_of_drag= np.cross(v_aerodynamic,D)                            ## 항력 에 의한 토크

        ''' 미분 방정식 '''
        '''w_dot = T / I
           angle_dot = w
           mass_dot = 0
           velocity_dot = F/m
           position_dot = velocity'''
        wx_dot, wy_dot, wz_dot =  torque_of_drag/(m*self.rocket.rocket_length**2)                        
        theta_dot, phi_dot, psi_dot = w*180/np.pi
        m_dot = 0
        vx_dot, vy_dot, vz_dot = D/m -g
        px_dot, py_dot, pz_dot = v
        
        self.rocket.accel = np.array([vx_dot,vy_dot,vz_dot])

        if pz < 0:
            vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot, psi_dot, theta_dot, phi_dot, alpha_dot, beta_dot, wx_dot, wy_dot, wz_dot = \
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        return np.array([vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot, psi_dot, theta_dot, phi_dot,wx_dot, wy_dot, wz_dot])


    def in_burnning(self,motor_angle):              ## in burnning, differential equation
        t1 = np.linspace(0,0.05,11)                 ## differential time => 0.005s
        self.params = odeint(self.force_burn,self.rocket.zeroparam,t1,tuple(motor_angle))       ## differential equation function
        return self.params

    def end_of_burnning(self):                      ## in free fall, differential equation
        t2 = np.linspace(0,0.05,11)                 ## differential time => 0.005s
        self.params = odeint(self.force_free,self.rocket.zeroparam,t2)      ## differntial equation function
        return self.params