import numpy as np
from Rocket_System import Rocket_system
import rospy
from std_msgs.msg import Float32MultiArray
from transform import Transformer
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import axes3d

if __name__ == '__main__':


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
    aerocenter = 0.5                            # rocket aero center (meters from bottom)
    drag_coeff = 0.4                            # drag coefficient


    '''Set motor'''
    mass_pro = 0.5                              # propellent mass (kg)
    thrust = np.array([0,0,100])                # thrust (N)
    burnTime = 3.5                                # burnning time (s)
    motor_angle = np.array([0,0,0])             # rocket initial motor angle (r,p,y ('))










    # initialization rocket class
    rocket = Rocket_system(mass_struct,mass_pro,burnTime,drag_coeff,position,velocity,angular_velocity,\
                           rocket_angle,motor_angle,rocket_length,mass_center,aerocenter,diameter,thrust)

    rospy.init_node('simulation')                                                       # Set rospy node
    pub2missionplanner = rospy.Publisher('data',Float32MultiArray,queue_size=10)        # publish to missionplanner
    rospy.Subscriber('Actuator',Float32MultiArray,rocket.readTvc)                       # Tvc on (communicate with Tvc node)

    # Ready to plotting
    fig = plt.figure(facecolor='w')                             ## plotting figure
    # ax = fig.add_subplot(111,projection='3d')                   ## 3d plot
    ax = fig.add_subplot(1,1,1)                                 ## 2d plot
    # ax2 = ax.twinx()                                            ## set second y-axis
    
    ''' initialization  '''
    rocket_shape = []                                           ## initialize rocket shape
    rocket_pos_list = np.empty((0,3))                           ## save all position for plotting
    rocket_velocity_list = np.empty((0,3))                      ## save all velocity
    rocket_angle_list = np.empty((0,3))                         ## save all rocket angle
    rocket_accel_list = np.empty((0,3))                         ## save all rocket accel
    rocket_total_velocity_list = np.empty((0,1))                ## total velocity
    rocket_total_mass = np.empty((0,1))                         ## total mass
    
    Apogee = 0                                                                                         
    Max_acceleration = 0
    Max_velocity = 0
    Max_x = 0


    ''' Main loop '''
    while rocket.realTime <= 20:                                ## flight time (s)

        data = Float32MultiArray()
        angle_data = rocket.zeroparam[7:10]                     ## get angle
        w_data = rocket.zeroparam[10:13]                        ## get anguler velocity
        data.data = np.append(angle_data,w_data,axis=0)         ## publish rocket angle, w
        pub2missionplanner.publish(data)                        ## publish angle and anguler velocity

        # rospy.sleep(0.07)                                     

        rocket.calculate_next_pos_of_rocket()                   ## calculate next params of rocket ( main simulator )
        rocket.zeroparam = rocket.params[-1]                    ## initialization zeroparam


        # Apogee
        if rocket.zeroparam[5] > Apogee:                                ## calculate Apogee
            Apogee = rocket.zeroparam[5]
        # Max pos X
        if rocket.zeroparam[3] > Max_x:                                 ## calculate Max x
            Max_x = rocket.zeroparam[3]
        # Max velocity (z)
        if rocket.zeroparam[2] > Max_velocity:                          ## calculate Max velocity (z)
            Max_velocity = rocket.zeroparam[2]
        # Max accel (z)
        if rocket.accel[2] > Max_acceleration:                          ## calculate Max accleration (z)
            Max_acceleration = rocket.accel[2]


        ''' parameters,  index i means i*0.05 second
            rocket_total_mass[i,0] : mass at i*0.05 second
            rocket_velocity_list[i,2] : Vz at i*0.05 second'''
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

        M2 = Transformer().body_to_earth(np.array([rocket.params[-1,7], rocket.params[-1,8], rocket.params[-1,9]]))         # transform matrix rocket => ground
        rocket_pos_vector = 50*M2@[0,0,1]                               ## rocket body vector (20은 시뮬레이션상 로켓 크기이며, 보기 편하도록 조절해주시면 됩니다.)

        rocket_shape.append([[rocket_pos_center[0]-rocket_pos_vector[0],rocket_pos_center[0]+rocket_pos_vector[0]],
                             [rocket_pos_center[1]-rocket_pos_vector[1],rocket_pos_center[1]+rocket_pos_vector[1]],
                             [rocket_pos_center[2]-rocket_pos_vector[2],rocket_pos_center[2]+rocket_pos_vector[2]]])
        # rocket shape : center of mass - body vector    ~~    center of mass + body vector

        # time flow
        rocket.realTime += 0.05


    """ 3D animation """
    def animate(i):
        ax.clear()                                                  # initialization
        ax.set_xlim(0, 600)
        ax.set_ylim(0, 600)
        ax.set_zlim(0, 600)

        # ax.view_init(elev=90., azim=60)                             # view angle
        ax.set_xlabel('x')                                          # set x label
        ax.set_ylabel('y')                                          # set y label                                                          
        ax.set_zlabel('altitude')                                   # set z label

        x = rocket_shape[i][0]
        y = rocket_shape[i][1]
        z = rocket_shape[i][2]

        ax.plot(rocket_pos_list[:i,0],rocket_pos_list[:i,1],rocket_pos_list[:i,2],'b-',label = '1st')   

        time = i*0.05
        ax.text(100,100,100,'Time = %.1fs'%time)                                                           # plot time
        ax.text(100,100,200,'Altitude = %.1fm'%rocket_pos_list[i,2])
        
        return ax.plot(x,y,z,color = 'k', lw = 2)  
     
    # animate = animation.FuncAnimation(fig,animate, frames = 300, interval=1) 


    """ 2D plotting """
    ax.set_xlim(0,500)                                                      ## set x line
    # ax.set_xlabel('Time')                                                   ## set x label
    ax.set_ylim(0,500)                                                      ## set y line
    # ax.set_ylabel('Total mass')                                             ## set y label
    # ax2.set_ylim(-100,2000)                                               ## set second y line
    # ax2.set_ylabel('Vertical Velocity, acceleration')                     ## set second y label
    
    # x coordinate
    # x = np.linspace(0,25,501)                                             ## time space (s)
    # x = np.arange(0,15,0.05)                                              ## time space (s)
    x = rocket_pos_list[:,0]                                              ## rocket pos x (m)
    # ax.plot(x,x*0,linestyle='--')                                         ## Draw line y = 0

    # y coordinate
    # y = rocket_angle_list[:,2]                                              ## rocket angle (yaw)
    y = rocket_pos_list[:,2]                                                ## rocket pos z (m)
    # y = rocket_velocity_list[:,2]                                           ## rocket velocity Vz (m/s)
    # y = rocket_total_velocity_list[:,0]                                     ## rocket total velocity (m/s)
    # y = rocket_total_mass[:,0]                                              ## rocket mass (kg)
    # y = rocket_accel_list[:,2]                                              ## rocket acceleration Az (m/s^2)


    ax.plot(x,y)                                                          ## plot x,y function
    # plt.axhline(y=20,ls='--')                                             ## Draw line y = 70

    ax.text(300,300,'Apogee : %.1fm'%Apogee)                                        ## Write text (Apogee )
    # ax.text(1000,4000,'Max x : %.1fm'%Max_x)                                          ## Write text ( Max pos x )
    # ax.text(30,1000,'Max velocity : %.1f m/s'%np.max(rocket_total_velocity_list))     ## Write text ( total velocity )
    # ax.text(23,500,'Max acceleration : %.1fm/s'%Max_acceleration)                     ## Write text ( Max acceleration )

    print('---plotting---please wait------')
    plt.show()                                                              ## show picture
    plt.rcParams['font.size'] = 10                                          ## font size
    # animate.save("file name').mp4",fps=20)                                ## save, 1 frame => 0.05s ( 30 fps = 1.5 real fps)
    print('---finish---check your folder which this file is in---')