#!/usr/bin/env python
import math
import numpy as np


class OLS():
    def __init__(self, frequency, horizon, length_history, n):
        self.hz = frequency
        self.dt = 1.0/self.hz
        self.hor_l = horizon
        self.his_l = length_history
        self.n = n
       
       #initizalize matrixees, _h for hisoty _f for prediction
        self.x_h = np.zeros(self.his_l)
        self.y_h = np.zeros(self.his_l)
        self.x_f = np.zeros(self.hor_l)
        self.y_f = np.zeros(self.hor_l)
        #initialize time matrixes
        self.t_h = np.arange(0, self.his_l)*self.dt
        #print(self.t_h)
        self.t_f = np.arange(0, self.hor_l)*self.dt + self.t_h[self.his_l-1]+self.dt
        #print(self.t_f)

        self.t_h[self.his_l-1]
        self.t_h = self.t_h.transpose()
        self.t_f = self.t_f.transpose()

        #Prepare Matrix
        print
        self.T_h = np.ones((self.his_l, self.n+1))
        self.T_f = np.ones((self.hor_l, self.n+1))
        #print(self.T_h)
        for i in range(1,self.n+1):
            self.T_h[:,i] = self.t_h**i
            self.T_f[:,i] = self.t_f**i
        
        #Bstarr  = c*x_h
        #c = inv(T'*T)*Ti
        #numpy matrix .T is the transpose
        self.c_h = np.matmul(self.T_h.T, self.T_h)
        self.c_h = np.linalg.inv(self.c_h)
        self.c_h = np.matmul(self.c_h, self.T_h.T)
        #print(self.c_h)

    def read(self, x, y):
        self.x_h = x
        self.y_h = y

    def update(self): 
        Bstar_x = np.matmul(self.c_h,self.x_h)
        Bstar_y = np.matmul(self.c_h,self.y_h)
        X_tilde_f = np.matmul(self.T_f,Bstar_x)
        Y_tilde_f = np.matmul(self.T_f,Bstar_y)

        return X_tilde_f, Y_tilde_f