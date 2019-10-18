#!/usr/bin/env python
import math
import numpy as np
import rospy


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

        self.t_h = self.t_h.transpose()
        self.t_f = self.t_f.transpose()

        #Prepare Matrix these had n+1
        self.T_h = np.ones((self.his_l, self.n))
        self.T_f = np.ones((self.hor_l, self.n))
        #print(self.T_h)
        for i in range(1,self.n):
            self.T_h[:,i] = self.t_h**i
            self.T_f[:,i] = self.t_f**i
        
        #Bstarr  = c*x_h
        #c = inv(T'*T)*T'
        #numpy matrix .T is the transpose
        c_h_1 = np.matmul(self.T_h.T, self.T_h)
        c_h_2 = np.linalg.inv(c_h_1)
        self.c_h = np.matmul(c_h_2, self.T_h.T)

    def read(self, x, y, z, roll, pitch, yaw):
        self.x_h = x
        self.y_h = y
        self.z_h = z
        self.phi = roll
        self.theta = pitch
        self.psi = yaw

    def update(self):
        Bstar_x = np.matmul(self.c_h,self.x_h)
        Bstar_y = np.matmul(self.c_h,self.y_h)
        Bstar_z = np.matmul(self.c_h,self.z_h)
        Bstar_vx = np.ones(self.n)
        Bstar_vy = np.ones(self.n)
        Bstar_vz = np.ones(self.n)
        Bstar_ax = np.ones(self.n)
        Bstar_ay = np.ones(self.n)
        Bstar_az = np.ones(self.n)

        for i in range(0,self.n):
            if(i<self.n-1):
                Bstar_vx[i] = (i+1)*Bstar_x[i+1]
                Bstar_vy[i] = (i+1)*Bstar_y[i+1]
                Bstar_vz[i] = (i+1)*Bstar_z[i+1]
            else:
                Bstar_vx[i] = 0
                Bstar_vy[i] = 0
                Bstar_vz[i] = 0
            
            if(i<self.n-2):
                Bstar_ax[i] = (i+1)*(i+2)*Bstar_x[i+2]
                Bstar_ay[i] = (i+1)*(i+2)*Bstar_y[i+2]
                Bstar_az[i] = (i+1)*(i+2)*Bstar_z[i+2]
            else:
                Bstar_ax[i] = 0
                Bstar_ay[i] = 0
                Bstar_az[i] = 0

        # print(Bstar_x)
        # print(Bstar_vx)
        # print(Bstar_ax)

        X_tilde_f = np.matmul(self.T_f,Bstar_x)
        Y_tilde_f = np.matmul(self.T_f,Bstar_y)
        Z_tilde_f = np.matmul(self.T_f,Bstar_z)

        
        vX_tilde_f = np.matmul(self.T_f,Bstar_vx)
        vY_tilde_f = np.matmul(self.T_f,Bstar_vy)
        vZ_tilde_f = np.matmul(self.T_f,Bstar_vz)
        
        aX_tilde_f = np.matmul(self.T_f,Bstar_ax)
        aY_tilde_f = np.matmul(self.T_f,Bstar_ay)
        aZ_tilde_f = np.matmul(self.T_f,Bstar_az)

        # self.wvX_tilde_f = np.diff(X_tilde_f)/self.fut_dt
        # #wvX_tilde_f = np.append(wvX_tilde_f, wvX_tilde_f[self.hor_l-2])
        # self.wvY_tilde_f = np.diff(Y_tilde_f)/self.fut_dt
        # #wvY_tilde_f = np.append(wvY_tilde_f, wvY_tilde_f[self.hor_l-2])
        # self.wvZ_tilde_f = np.diff(Z_tilde_f)/self.fut_dt
        # #wvZ_tilde_f = np.append(wvZ_tilde_f, wvZ_tilde_f[self.hor_l-2])

        # # self.bvX_tilde_f,  self.bvY_tilde_f,  self.bvZ_tilde_f = self.WorldToBody(
        # #     self.phi, self.theta, self.psi, self.wvX_tilde_f, self.wvY_tilde_f, self.wvZ_tilde_f
        # # )

        # self.waX_tilde_f = np.diff(self.wvX_tilde_f)/self.fut_dt
        # #waX_tilde_f = np.append(waX_tilde_f, waX_tilde_f[self.hor_l-2])
        # self.waY_tilde_f = np.diff(self.wvY_tilde_f)/self.fut_dt
        # #waY_tilde_f = np.append(waY_tilde_f, waY_tilde_f[self.hor_l-2])
        # self.waZ_tilde_f = np.diff(self.wvZ_tilde_f)/self.fut_dt
        # #waZ_tilde_f = np.append(waZ_tilde_f, waZ_tilde_f[self.hor_l-2])

        # # self.baX_tilde_f,  self.baY_tilde_f,  self.baZ_tilde_f = self.WorldToBody(
        # #     self.phi, self.theta, self.psi, self.waX_tilde_f, self.waY_tilde_f, self.waZ_tilde_f
        # # )

        # # rospy.loginfo('x tilde is %s', X_tilde_f)
        # # rospy.loginfo('vx tilde is %s', self.bvX_tilde_f)
        # # rospy.loginfo('ax tilde is %s', self.baX_tilde_f)


        X_tilde_f[0] = self.x_h[self.his_l-1]
        Y_tilde_f[0] = self.y_h[self.his_l-1]
        Z_tilde_f[0] = self.z_h[self.his_l-1]

        return X_tilde_f, Y_tilde_f, Z_tilde_f, vX_tilde_f, vY_tilde_f, vZ_tilde_f, aX_tilde_f, aY_tilde_f, aZ_tilde_f
    
    # def getVeloctityHorizon(self):
    #     return self.wvX_tilde_f,  self.wvY_tilde_f,  self.wvZ_tilde_f 
    
    # def getAccelerationHorizon(self):
    #     return self.waX_tilde_f,  self.waY_tilde_f,  self.waZ_tilde_f

    def WorldToBody(self, phi, theta, psi, x, y, z):
        #input needs to be in deg

        sp = math.sin(phi)
        cp = math.cos(phi)
        st = math.sin(theta)
        ct = math.cos(theta)
        ss = math.sin(psi)
        cs = math.cos(psi)

        x_body = ct*cs*x + ct*ss*y -st*z
        y_body = (sp*st*cs-cp*ss)*x + (sp*st*ss+cp*cs)*y + sp*ct*z
        z_body = (cp*st*cs+sp*ss)*x + (cp*st*ss-sp*cs)*y+cp*ct*z

        return x_body, y_body, z_body