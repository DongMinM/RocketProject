from tkinter import *
from tracemalloc import start
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

class Gui:
    def __init__(self):
        rospy.init_node('groundstation')
        self.data = []
        rospy.Subscriber('SensorData',Float32MultiArray, self.read)

        self.Bool = ['No','Yes']

    def read(self,msg):
        self.data = msg.data
        self.data = np.around(self.data,2)
        # self.data : Vx, Vy, Vz, x, y, z Ax, Ay, Az, psi, theta, phi, wx, wy, wz
        # print(np.around(self.data,2))

    def run(self):
        self.require = 'None'
        self.set_gui()
        self.startWrite()
        self.win.mainloop()


    def set_gui(self):

        self.win = Tk()
        self.win.title("Data Hub")
        self.win.geometry('1000x300')

        self.text = Label(self.win,text='Status : Wait launch',font=('HeLvetica',30))
        self.text.place(x=250,y=30)

        self.text2 = Label(self.win,text='Wait launch or Click Button',font=('HeLvetica',30))
        self.text2.place(x=250,y=100)

        Velocity_read = Button(self.win,text='Velocity',bg='yellow',command=self.velocity_starter)
        Velocity_read.place(x=250,y=180)
        # Velocity_read.pack()

        Gps_read = Button(self.win,text='Gps',bg='yellow',command=self.gps_starter)
        Gps_read.place(x=350,y=180)

        accel_read = Button(self.win,text='Accle',bg='yellow',command=self.accle_starter)
        accel_read.place(x=430,y=180)
        
        reset = Button(self.win,text='Reset',bg='yellow',command=self.restart)
        reset.place(x=250,y=230)
    
        # Gps_read.pack() 


    def startWrite(self):
        print(123123123)
        if len(self.data) > 1:
            self.text.configure(text='Status. burn : '+self.Bool[int(self.data[0])]+', payload seperated : '+self.Bool[int(self.data[1])],font=('HeLvetica',20))
            if self.data[7] == 0:
                self.text.configure(text='Status : Finish flight')

            if self.require =='Velocity':
                self.text2.configure(text='Velocity : '+str(round((self.data[2]**2+self.data[3]**2+self.data[4]**2)**0.5,2))+'m/s',font=('HeLvetica',30))

            elif self.require =='GPS':
                self.text2.configure(text='Gps : '+'x : '+str(self.data[5])+'m/s, y : '+str(self.data[6])+'m/s, z : '+str(self.data[7])+'m/s',font=('HeLvetica',20))
                # self.text2.configure(text='3')

            elif self.require =='Accle':
                self.text2.configure(text='accle : '+'Ax : '+str(self.data[8])+'m/s2, Ay : '+str(self.data[9])+'m/s2, Az : '+str(self.data[10])+'m/s2',font=('HeLvetica',20))
                # self.text2.configure(text='3')

        else:
            self.text.configure(text='Status : Wait launch')
            self.text2.configure(text='Wait launch and Click Button')
        self.win.after(100,self.startWrite)


    def velocity_starter(self):
        self.require = 'Velocity'

    def gps_starter(self):
        self.require = 'GPS'

    def accle_starter(self):
        self.require = 'Accle'

    def restart(self):
        self.require = 'None'
        self.data = []

if __name__ == '__main__':
    gui = Gui()
    gui.run()
