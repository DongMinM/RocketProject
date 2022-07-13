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
        l_T = self.rocket.left_T
        r_T = -np.array(self.rocket.right_T)

        self.rocket.total_mass_center = (self.rocket.mass_center[0]*self.rocket.mass_struct+self.rocket.mass_center[1]*(m-self.rocket.mass_struct))/m

        M1 = Transformer().body_to_earth([psi, theta, phi])                             ## transform metrix rocket -> ground
        con_vec = Transformer().body_to_earth(np.array([alpha, beta, gamma]))           ## transform metrix motor -> rocket

        T = con_vec@T                                                                   ## transform trust motor -> rocket
        T = M1@T                                                                        ## transform trust rocket -> ground

        l_T = con_vec@l_T
        l_T = M1@l_T

        r_T = con_vec@r_T
        r_T = M1@r_T
        
                                                   
        r_to_bottom = -self.rocket.total_mass_center*M1@[0,0,1]                         ## vector from mass center to bottom                

        torque = np.cross(r_to_bottom,T)                                                ## calcul torque by trust

        rcs_to_bottom = (1-self.rocket.total_mass_center)*M1@[0,0,1]
        torque_l = np.cross(rcs_to_bottom,l_T)
        torque_r = np.cross(rcs_to_bottom,r_T)
        
  
        v = np.array([vx,  vy,  vz])                                                    ## velocity
        p = np.array([px,  py,  pz])                                                    ## position
        w = np.array([wx,  wy,  wz])                                                    ## angular velocity
        g = np.array([ 0,   0, 9.8])                                                    ## gravity
        v_aerodynamic = M1@[0,0,self.rocket.total_aero_center-self.rocket.total_mass_center]          ## vector from mass center to aero center

        S = np.pi*self.rocket.diameter*self.rocket.diameter/4               ## cross-sectional area
        if pz <= 35000:
            rho = 1.225*(1-1.8e-5*pz)**5.656                                  ## calculate atmospheric density
        else :
            rho = 0.001
        D = -0.5*rho*S*self.rocket.Cd*np.linalg.norm(v)*v                   ## calculate drag force
        torque_of_drag= np.cross(v_aerodynamic,D)                           ## calculate torque of drag

        Para = [0,0,0.5*rho*np.linalg.norm(v)*np.linalg.norm(v)*0.135*np.pi*self.rocket.Cd_para]
        torque_of_para = np.cross(M1@[0,0,2-self.rocket.total_mass_center],Para)

        self.rocket.total_drag = [np.linalg.norm(D)+np.linalg.norm(Para)]

        wx_dot, wy_dot, wz_dot = (torque+torque_of_drag+torque_of_para+torque_l+torque_r)/(0.5*m*self.rocket.rocket_length**2)                  ## differential equation of angular acceleration
        theta_dot, phi_dot, psi_dot = w*180/np.pi                           ## differential equation of angul
        m_dot = -self.rocket.mass_pro/self.rocket.t_b                       ## differential equation of propellent mass
        vx_dot, vy_dot, vz_dot = (T+D+Para+l_T+r_T)/m -g                                 ## differential equation of velocity of rocket
        
        if pz<=0 and vz_dot<=0:                                                       ## rocket can't go under the ground
            pz = 0

        px_dot, py_dot, pz_dot = v                                          ## differential equation of position of rocket
        self.rocket.accel = np.array([vx_dot,vy_dot,vz_dot])                ## save rocket acceleration
        

        return np.array([vx_dot, vy_dot, vz_dot, px_dot, py_dot, pz_dot, m_dot,psi_dot, theta_dot, phi_dot, wx_dot, wy_dot, wz_dot])

        


    def in_burnning(self,motor_angle):              ## in burnning, differential equation
        t1 = np.linspace(0,0.05,11)                 ## differential time => 0.005s
        return odeint(self.force_burn,self.rocket.zeroparam,t1,tuple(motor_angle))       ## differential equation function

