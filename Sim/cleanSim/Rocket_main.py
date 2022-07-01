import numpy as np
from Rocket_plan import Rocket_system
from transform import Transformer
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import axes3d
from matplotlib import gridspec

if __name__ == '__main__':


    '''Set initial status'''
    position = np.array([0,0,0])                # rocket initial posision (x,y,z (m))
    velocity = np.array([0,0,0])                # rocket initial velocity (x,y,z (m/s))
    rocket_angle = np.array([0,0,1])           # rocket initial angle (r,p,y ('))
    angular_velocity = np.array([0,0,0])        # rocket initial angular velocity (r,p,y (rad/s))

    '''Set rocket body'''
    rocket_length = 1.3                         # rocket length (m)
    diameter = 0.09                             # rocket outside diameter (m)
    mass_struct = 3                             # structure mass (kg)
    mass_center = [0.6,0.15]                     # rocket initial mass center of structure, propellant (meters from bottom)
    aerocenter = 0.45                            # rocket aero center (meters from bottom)
    drag_coeff = 0.5                            # drag coefficient

    '''Set motor'''
    mass_pro = 0.5                              # propellent mass (kg)
    thrust = np.array([0,0,100])                # thrust (N)
    burnTime = 3                                # burnning time (s)
    motor_angle = np.array([0,0,0])             # rocket initial motor angle (r,p,y ('))


    simTime = 30



    print("---START ROCKET SIMULATOR---")

    # initialization rocket class
    rocket = Rocket_system(mass_struct,mass_pro,burnTime,drag_coeff,position,velocity,angular_velocity,\
                           rocket_angle,motor_angle,rocket_length,mass_center,aerocenter,diameter,thrust)





    # Ready to plotting
    fig = plt.figure(figsize=(12,8),facecolor='w')
    gs = gridspec.GridSpec(nrows=6, # row 몇 개 
                       ncols=3, # col 몇 개 
                       height_ratios=[1, 3, 3, 3, 3, 1], 
                       width_ratios=[12, 6, 1]
                      )                             ## plotting figure
    ax = fig.add_subplot(gs[1:4,0],projection='3d')                   ## 3d plot                                           ## set second y-axis
    # ax2 = fig.add_subplot(gs[1,1])
    # ax3 = fig.add_subplot(gs[2,1])
    # ax4 = fig.add_subplot(gs[3,1])
    ax5 = fig.add_subplot(gs[4,1])


    ''' initialization  '''
    rocket_shape = []                                           ## initialize rocket shape
    rocket_pos_list = np.empty((0,3))                           ## save all position for plotting
    rocket_velocity_list = np.empty((0,3))                      ## save all velocity
    rocket_angle_list = np.empty((0,3))                         ## save all rocket angle
    rocket_accel_list = np.empty((0,3))                         ## save all rocket accel
    rocket_total_velocity_list = np.empty((0,1))                ## total velocity
    rocket_total_mass = np.empty((0,1))                         ## total mass
    thrust_list = np.empty((0,3))

    Apogee = 0                                                                                         
    Max_acceleration = 0
    Max_velocity = 0
    Max_x = 0


    ''' Main loop '''
    while rocket.realTime <= simTime:                                ## flight time (s)

        rocket.calculate_next_pos_of_rocket()                   ## calculate next params of rocket ( main simulator )
        rocket.zeroparam = rocket.params[-1]                    ## initialization zeroparam
        thrust_list = np.append(thrust_list,[rocket.T],axis=0)
        # print(rocket.zeroparam)

        # Apogee
        if rocket.zeroparam[5] > Apogee:                                ## calculate Apogee
            Apogee = rocket.zeroparam[5]
        else :
            # rocket.Cd_para=1.2
            rocket.total_aero_center = 1.3

        # Max pos X
        if rocket.zeroparam[3] > Max_x:                                 ## calculate Max x
            Max_x = rocket.zeroparam[3]

        # print(rocket.total_aero_center)
        # Max velocity (z)
        if rocket.zeroparam[2] > Max_velocity:                          ## calculate Max velocity (z)
            Max_velocity = rocket.zeroparam[2]
        # Max accel (z)
        if rocket.accel[2] > Max_acceleration:                          ## calculate Max accleration (z)
            Max_acceleration = rocket.accel[2]


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
        # print(rocket_total_velocity_list)

        M2 = Transformer().body_to_earth(np.array([rocket.params[-1,7], rocket.params[-1,8], rocket.params[-1,9]]))         # transform matrix rocket => ground
        rocket_pos_vector = 20*M2@[0,0,1]                               ## rocket body vector (20은 시뮬레이션상 로켓 크기이며, 보기 편하도록 조절해주시면 됩니다.)

        rocket_shape.append([[rocket_pos_center[0]-rocket_pos_vector[0],rocket_pos_center[0]+rocket_pos_vector[0]],
                             [rocket_pos_center[1]-rocket_pos_vector[1],rocket_pos_center[1]+rocket_pos_vector[1]],
                             [rocket_pos_center[2]-rocket_pos_vector[2],rocket_pos_center[2]+rocket_pos_vector[2]]])

        # rocket shape : center of mass - body vector    ~~    center of mass + body vector

        # time flow
        rocket.realTime += 0.05

    """ 3D animation """
    def animate(i):
        ax.clear()                                                  # initialization
        ax.set_xlim(0, 500)
        ax.set_ylim(0, 500)
        ax.set_zlim(0, 500)
        ax.grid(False)

        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_zticks([])

        ax.view_init(elev=0, azim=270)

        x1 = rocket_shape[i][0][0]
        x2 = rocket_shape[i][0][1]
        y1 = rocket_shape[i][1][0]
        y2 = rocket_shape[i][1][1]
        z1 = rocket_shape[i][2][0]
        z2 = rocket_shape[i][2][1]

        ax.plot(rocket_pos_list[:i,0],rocket_pos_list[:i,1],rocket_pos_list[:i,2],'b-',label = '1st')   

        time = i*0.05
        ax.text(100,100,100,'Time = %.2fs'%time)                                                           # plot time
        ax.text(100,100,250,'Altitude = %.1fm'%rocket_pos_list[i,2])
        ax.text(100,100,300,'Apogee = %.1fm'%Apogee)
        ax.text(100,100,200,'Velocity = %.1fm/s'%rocket_velocity_list[i,2])
        ax.text(100,100,150,'Thrust = %.1fN'%thrust_list[i,2])
        if rocket_pos_list[i,2]<30 and rocket_velocity_list[i,2] < 0:
            ax.text(100,100,50,'Parachute on')



        return ax.quiver(x1,y1,z1,x2-x1,y2-y1,z2-z1, color='k',lw = 2,arrow_length_ratio=0.2) 
        # 참고 : https://matplotlib.org/2.0.2/mpl_toolkits/mplot3d/tutorial.html#quiver
     
    animate = animation.FuncAnimation(fig,animate, frames = simTime*20, interval=1) 


    # """ 2D plotting """
    # ax2.set_xlim(0,simTime)                                                      ## set x line
    # ax2.set_xlabel('Time(s)',fontsize=10)                                                   ## set x label
    # ax2.set_ylim(-100,Max_velocity+50)                                                      ## set y line
    # ax2.set_ylabel('Velocity(m/s)',fontsize=10)                                             ## set y label
    # ax2.grid(True)
    # # ax2.tick_params(axis='both',labelsize=10)

    # ax3.set_xlim(0,simTime)                                                      ## set x line
    # ax3.set_xlabel('Time(s)',fontsize=10)                                                   ## set x label
    # ax3.set_ylim(-30,Max_acceleration+20)                                                      ## set y line
    # ax3.set_ylabel('Accel(m/s.2)',fontsize=10)                                             ## set y label
    # ax3.grid(True)



    # ax4.set_xlim(0,simTime)                                                      ## set x line
    # ax4.set_xlabel('Time(s)',fontsize=10)                                                   ## set x label
    # ax4.set_ylim(-10,Apogee+100)                                                      ## set y line
    # ax4.set_ylabel('altitude(m)',fontsize=10)                                             ## set y label
    # ax4.grid(True)


    ax5.set_xlim(0,simTime)                                                      ## set x line
    ax5.set_xlabel('Time(s)',fontsize=10)                                                   ## set x label
    ax5.set_ylim(-10,800)                                                      ## set y line
    ax5.set_ylabel('Thrust(N)',fontsize=10)                                             ## set y label
    ax5.grid(True)


    x = np.linspace(0,simTime,simTime*20)
    # ax2.plot(x,rocket_velocity_list[:,2])
    # ax3.plot(x,rocket_accel_list[:,2])
    # ax4.plot(x,rocket_pos_list[:,2])
    ax5.plot(x,thrust_list[:,2])

    # ax2.axhline(y=0,ls='--')
    # ax3.axhline(y=0,ls='--')
    # ax4.axhline(y=0,ls='--')
    ax5.axhline(y=0,ls='--')

    # ax2.axvline(x=rocket.arrive_ground,ls='--')
    # ax3.axvline(x=rocket.arrive_ground,ls='--')
    # ax4.axvline(x=rocket.arrive_ground,ls='--')
    ax5.axvline(x=rocket.arrive_ground,ls='--')

    # ax4.text(0,0,'Apogee : %.1fm'%Apogee)
    # ax2.text(0,-90,'Max_velocity : %.1fm/s'%Max_velocity)
    # ax3.text(0,-20,'Max_accel : %.1fm/s^2'%Max_acceleration)

    plt.tight_layout()
    plt.show()                                                              ## show picture
    # animate.save('TEST.mp4',fps=20)                                ## save, 1 frame => 0.05s ( 30 fps = 1.5 real fps)
