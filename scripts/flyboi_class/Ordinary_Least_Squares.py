#!/usr/bin/env python
import math
import numpy as np


class OLS():
    def __init__(self, history_frequency, future_history, horizon, length_history, n):
        self.his_hz = history_frequency
        self.fut_hz = future_history
        self.his_dt = 1.0/self.his_hz
        self.fut_dt = 1.0/self.fut_hz
        self.hor_l = horizon
        self.his_l = length_history
        self.n = n
       
       #initizalize matrixees, _h for hisoty _f for prediction
        self.x_h = np.zeros(self.his_l)
        self.y_h = np.zeros(self.his_l)
        self.z_h = np.zeros(self.his_l)
        self.x_f = np.zeros(self.hor_l)
        self.y_f = np.zeros(self.hor_l)
        self.z_f = np.zeros(self.hor_l)
        #initialize time matrixes
        self.t_h = np.arange(0, self.his_l)*self.his_dt
        self.t_f = np.arange(0, self.hor_l)*self.fut_dt + self.t_h[self.his_l-1]

        self.t_h[self.his_l-1]
        self.t_h = self.t_h.transpose()
        self.t_f = self.t_f.transpose()
        
        #print(self.t_h)
        #print(self.t_f)

        #Prepare Matrix
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

    def read(self, x, y, z):
        self.x_h = x
        self.y_h = y
        self.z_h = z

    def update(self): 
        Bstar_x = np.matmul(self.c_h,self.x_h)
        Bstar_y = np.matmul(self.c_h,self.y_h)
        Bstar_z = np.matmul(self.c_h,self.z_h)
        X_tilde_f = np.matmul(self.T_f,Bstar_x)
        Y_tilde_f = np.matmul(self.T_f,Bstar_y)
        Z_tilde_f = np.matmul(self.T_f,Bstar_z)

        X_tilde_f[0] = self.x_h[self.his_l-1]
        Y_tilde_f[0] = self.y_h[self.his_l-1]
        Z_tilde_f[0] = self.z_h[self.his_l-1]

        return X_tilde_f, Y_tilde_f, Z_tilde_f