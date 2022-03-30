import RPi.GPIO as GPIO
from time import sleep  #time 라이브러리의 sleep함수 사용

servoPin          = 10   # 서보핀
servoPin2         = 12
SERVO_MAX_DUTY    = 10.3   # 서보의 최대(180도) 위치의 주기
SERVO_MIN_DUTY    = 2.2    # 서보의 최소(0도) 위치의 주기
SERVO_MAX_DUTY2   = 11
SERVO_MIN_DUTY2    = 2.4

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servoPin, GPIO.OUT)
GPIO.setup(servoPin2, GPIO.OUT)

servo = GPIO.PWM(servoPin, 50)  # 서보핀을 PWM 모드 50Hz로 사용하기 (50Hz > 20ms)
servo2 = GPIO.PWM(servoPin2, 50)
#servo.stop()
#servo2.stop()
#GPIO.cleanup()
servo.start(0)  # 서보 PWM 시작 duty = 0, duty가 0이면 서보는 동작하지 않는다.
servo2.start(0)


'''
서보 위치 제어 함수
degree에 각도를 입력하면 duty로 변환후 서보 제어(ChangeDutyCycle)
'''
def setServoPos(degree):
  # 각도는 180도를 넘을 수 없다.
  if degree > 180:
    degree = 180

  # 각도(degree)를 duty로 변경한다.
  duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
  # duty 값 출력
  #print("Degree: {} to {}(Duty)".format(degree, duty))

  # 변경된 duty값을 서보 pwm에 적용
  servo.ChangeDutyCycle(duty)

def setServo2Pos(degree):
  if degree > 180:
    degree = 180
  duty = SERVO_MIN_DUTY2+(degree*(SERVO_MAX_DUTY2-SERVO_MIN_DUTY2)/180.0)
  #print("Degree: {} to {}(Duty)".format(degree, duty))
  #print(degree-90)
  servo2.ChangeDutyCycle(duty)




if __name__ == "__main__":  
  # 서보 0도에 위치
  # setServoPos(0)
  # sleep(1) # 1초 대기
  # 90도에 위치
  # setServoPos(10)
  # sleep(1)
  # 50도..
  # setServoPos(0)
  # sleep(1)
  # 서보 PWM 정지
  while True:
        setServo2Pos(90)
        #setServoPos(90)
        sleep(0.1)
        setServo2Pos(80)
        #setServoPos(100)
        sleep(0.1)
  servo.stop()

  # GPIO 모드 초기화
  GPIO.cleanup()

