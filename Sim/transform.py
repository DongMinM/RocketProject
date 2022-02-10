import numpy as np
import matplotlib.pyplot as plt

class Transformer():
    def __init___(self):
        pass
    
    def body_to_earth(self, rocAtt):
        
        psi = rocAtt[0] * np.pi/180
        theta = rocAtt[1] * np.pi/180
        phi = rocAtt[2] * np.pi/180
        
        r = np.array([[np.cos(phi), -np.sin(phi), 0],       
                      [np.sin(phi),  np.cos(phi), 0],
                      [          0,            0, 1]])      #y
        # print('r : ',r)
        y = np.array([[ np.cos(theta), 0, np.sin(theta)],
                      [             0, 1,             0],
                      [-np.sin(theta), 0, np.cos(theta)]])      #p
        # print('y : ', y)
        p = np.array([[1,           0,            0],
                      [0, np.cos(psi), -np.sin(psi)],
                      [0, np.sin(psi),  np.cos(psi)]])      #r
        # print('p : ',p)
                     
        PP = np.array([[0, 0, -1],
                       [0, 1,  0],
                       [1, 0,  0]])
             
        transformation_mtx = y@p
                    
        return transformation_mtx
    
    def motor_angle_vector(self, alpha, beta):
        alpha = alpha * np.pi/180
        beta = beta * np.pi/180
        convential_vector = np.array([-np.sin(beta)*np.cos(alpha),
                                      -np.sin(beta)*np.sin(alpha),
                                      np.cos(beta)])
        return convential_vector