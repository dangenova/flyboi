#!/usr/bin/env python
import math
import numpy as np
import Ordinary_Least_Squares as ols
import matplotlib.pyplot as plt

class PathCreation():
    def __init__(self, history_length, horizon, frequency ,n):
        self.hz = frequency
        self.dt = 1.0/self.hz
        self.hor_l = horizon
        self.his_l = history_length

        #initialize matrix
        #there is no overlap in t_h and t_f
        self.t_h = np.arange(0, self.his_l)*self.dt
        self.t_f = np.arange(0, self.hor_l)*self.dt + self.t_h[self.his_l-1]+self.dt
        self.x_h = np.zeros(self.his_l)
        self.y_h = np.zeros(self.his_l)
        self.x_f = np.zeros(self.hor_l)
        self.y_f = np.zeros(self.hor_l)
        self.x_tilde = np.zeros(self.hor_l)
        self.y_tilde = np.zeros(self.hor_l)

        #initalize OLS
        self.OLS = ols.OLS(self.hz, self.hor_l, self.his_l, n)

        print('t-h')
        print(self.t_h)
        print('t-f')
        print(self.t_f)

    
    
    def sine_path(self, period, amplitude, variable, time):
        if variable == "x":
            if time == "h":
                for i in range(0, self.his_l):
                    self.x_h[i] = amplitude*math.sin(period*self.t_h[i])
                print('x-h')
                print(self.x_h)
            elif time == "f":
                for i in range(0, self.hor_l):
                    self.x_f[i] = amplitude*math.sin(period*self.t_f[i])
                print('x-f')
                print(self.x_f)
            else:
                error = 'time variable input wrong-me'
        elif variable == "y":
            if time == "h":
                for i in range(0, self.his_l):
                    self.y_h[i] = amplitude*math.sin(period*self.t_h[i])
            elif time == "f":
                for i in range(0, self.hor_l):
                    self.y_f[i] = amplitude*math.sin(period*self.t_f[i])
            else:
                error = 'time input wrong-me'
                print(error)
        else:
            error = 'time input wrong-me'
            print(error)
        
        

    
    def linear_path(self, slope, variable, time):
        if variable == "x":
            if time == "h":
                for i in range(0, self.his_l):
                    self.x_h[i] = slope*self.t_h[i]
            elif time == "f":
                for i in range(0, self.hor_l):
                    self.x_f[i] = slope*self.t_f[i]
            else:
                error = 'time variable input wrong-me'
        elif variable == "y":
            if time == "h":
                for i in range(0, self.his_l):
                    self.y_h[i] = slope*self.t_h[i]
            elif time == "f":
                for i in range(0, self.hor_l):
                    self.y_f[i] = slope*self.t_f[i]
            else:
                error = 'time input wrong-me'
                print(error)
        else:
            error = 'time input wrong-me'
            print(error)
    
    def call_ols(self):
        self.OLS.read(self.x_h, self.y_h)
        self.x_tilde, self.y_tilde =self.OLS.update()
    
    def graph(self):
        plt.figure(1)
        plt.plot(self.t_h, self.x_h, 'k--', self.t_f, self.x_f, 'k--', self.t_f, self.x_tilde, 'r')
        plt.show()
        plt.figure(2)
        plt.plot(self.t_h, self.y_h, 'k--', self.t_f, self.y_f, 'k--', self.t_f, self.y_tilde, 'r')
        plt.show()



if __name__ == '__main__': 
    
    history_length = 40
    horizon = 10
    hz = 10
    n = 6

    x = 'x'
    y = 'y'
    h = 'h'
    f = 'f'
    
    path = PathCreation(history_length, horizon, hz, n)
    path.sine_path(1,1,x,h)
    path.linear_path(1,y,h)
    path.sine_path(1,1,x,f)
    path.linear_path(1,y,f)
    path.call_ols()
    path.graph()



    