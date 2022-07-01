class Drone:
    def __init__(self):
        self.h = 0
        self.state = 'Land'

    def take_off(self):
        if self.h > 0:
            print('이미 드론이 공중에 있습니다.')
        else:
            self.h = 5
            self.state = 'InAir'
            print('====5m 고도로 이륙합니다.====')

    def up(self,amount):
        if self.state == 'InAir':
            self.h += amount
            print('====%.2fm로 상승합니다.===='%(self.h))
        else :
            print('잘못된 미션입니다.')

    def down(self,amount):
        if self.state == 'InAir':
            if self.h <= amount:
                print("Too low to down")
            else:
                self.h -= amount
                print('=====%.2fm로 하강합니다.===='%(self.h))
        else:
            print('잘못된 미션입니다.')

    def land(self):
        self.h = 0
        self.state='Land'
        print('====착륙합니다.====')


drone = Drone()
while True:
    mission = input("미션 입력 : ")
    if mission == 'up' or mission == 'down':
        amount = float(input('고도 입력 : '))



    if mission == 'take_off':
        drone.take_off()
    elif mission == 'up':
        drone.up(amount)
    elif mission == 'down':
        drone.down(amount)
    elif mission == 'land':
        drone.land()
    else:
        print('다시 입력해 주세요')