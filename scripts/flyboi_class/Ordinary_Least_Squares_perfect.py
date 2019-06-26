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
        
        self.X_tilde_f = np.zeros(self.hor_l)
        self.Y_tilde_f = np.zeros(self.hor_l)
        self.Z_tilde_f = np.zeros(self.hor_l)

        #initialize time matrixes
        self.t_h = np.arange(0, self.his_l)*self.his_dt
        self.t_f = np.arange(0, self.hor_l)*self.fut_dt + self.t_h[self.his_l-1]
        self.t_h = self.t_h.transpose()
        self.t_f = self.t_f.transpose()
        


    def read(self, x, y, z):
        self.x_h = x
        self.y_h = y
        self.z_h = z

    def update(self): 

        for i in range(0, self.hor_l):
            self.X_tilde_f[i] = 5*math.sin(.3*self.t_f[i])
            self.Y_tilde_f[i] = .2*(self.t_f[i])
            self.Z_tilde_f[i] = 5

        self.X_tilde_f[0] = self.x_h[self.his_l-1]
        self.Y_tilde_f[0] = self.y_h[self.his_l-1]
        self.Z_tilde_f[0] = self.z_h[self.his_l-1]

        return self.X_tilde_f, self.Y_tilde_f, self.Z_tilde_f