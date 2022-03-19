from tkinter import *
from tracemalloc import start
import rospy
from std_msgs.msg import Float32MultiArray
class Gui:
    def __init__(self):
        rospy.init_node('Guitest')
        self.data = ['None','None']
        rospy.Subscriber('msgs',Float32MultiArray, self.read)

    def read(self,msg):
        self.data = msg.data
        print(self.data)

    def run(self):
        self.require = 'None'
        self.set_gui()
        self.startWrite()
        self.win.mainloop()


    def set_gui(self):

        self.win = Tk()
        self.win.title("Data Hub")
        self.win.geometry('1000x300')

        self.text = Label(self.win,text='0',font=('HeLvetica',50))
        self.text.place(x=300,y=30)

        self.text2 = Label(self.win,text='0',font=('HeLvetica',50))
        self.text2.place(x=300,y=120)

        Velocity_read = Button(self.win,text='Velocity',bg='yellow',command=self.velocity_starter)
        Velocity_read.place(x=300,y=200)
        # Velocity_read.pack()

        Gps_read = Button(self.win,text='Gps',bg='yellow',command=self.gps_starter)
        Gps_read.place(x=400,y=200)
        # Gps_read.pack() 


    def startWrite(self):
        self.text.configure(text=str(self.data[0])+"  "+str(self.data[1]))
        if self.require =='Velocity':
            self.text2.configure(text=str(self.data[0]))

        elif self.require =='GPS':
            self.text2.configure(text=str(self.data[1]))
        self.win.after(100,self.startWrite)


    def velocity_starter(self):
        self.require = 'Velocity'

    def gps_starter(self):
        self.require = 'GPS'


if __name__ == '__main__':
    gui = Gui()
    gui.run()