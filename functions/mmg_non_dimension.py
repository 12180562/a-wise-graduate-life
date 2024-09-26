import numpy as np
import pandas as pd
# import matplotlib.pyplot as plt
import math
import rospy
class MMG:
    def __init__(self, u,v,r,heading,L,scale):

        """notice: wake, thrust deduction, rudder and propeller force are calculated in scale that is setted in model test.
        if we want to scaling, we have to calculate in same scale in model test.
        so, in this code, chanege the scale for calculate wake, thrust deduction and etc"""
        self.Model = rospy.get_param('Model') # 다른 class 및 module에서 파라미터 호출하기 위함

        self.U = math.sqrt(u**2+v**2)
        self.u = u
        self.v = v
        self.r = r
        self.r_deg = r*180/math.pi
        # self.delta =delta
        self.rps = 6.15
        self.heading = heading

        # rps full
        self.n1 = 6.5741
        self.n0 = -0.0111

        # wake coefficient
        self.w1 = -0.0106
        self.w0 = 0.2809

        # thrust deduction coefficient
        self.t1 = 0.003
        self.t0 = 0.158 

        # wake correction coefficients from static drift test
        self.C_p_plus = 4.168
        self.C_p_minus = 3.741

        # rudder inflow coefficient from static rudder test
        self.kappa = 0.554
        self.epsilon = 1.048

        # flow strengthening coefficient from drift & rudder test
        self.gamma_plus = 0.623
        self.gamma_minus = 0.380

        # ship geometry information
        self.density = 1025.0
        self.x_p_ = 0.49 # we dont have information about propeller location. it is just astimated value
        self.LBP = L
        self.d = 9.75
        self.propeller_diameter = 6.60
        self.disp = 28607
        self.m = self.disp*self.density
        self.WSA = 5596.3
        self.k_zz = 0.25*self.LBP
        self.I_zz = self.k_zz**2*self.m
        self.kinematic_viscousity = 1.10966e-6
        self.A_r = 26.80

        # propeller open water coefficient
        self.K_t2 = -0.0047
        self.K_t1 = -0.4997
        self.K_t0 = 0.5293

        # model information
        self.scale = scale
        self.model_propeller_diameter = self.propeller_diameter/self.scale
        self.model_LBP = self.LBP/self.scale
        self.model_d = self.d/self.scale
        self.model_density = 998.2
        self.model_m = self.m/(self.scale**3)
        self.model_WSA = self.WSA/(self.scale**2)
        self.model_k_zz = 0.25*self.model_LBP
        self.model_I_zz = self.model_k_zz**2*self.model_m
        self.x_G_ = -0.0098
        self.I_zz_ = self.model_I_zz/(0.5*self.model_density*self.model_LBP**5)
        self.model_A_r = self.A_r/(self.scale**2)
        self.f_alpha = 2.616
        self.rudder_rate = 0.0275
        self.X_0 = -6.7e-3
        self.X_uuu = -6.6e-3
        self.X_uu = 2.12e-2
        self.X_u = -1.94e-2
        self.X_vv = -6.964e-3
        self.X_vr = 4.738e-3
        self.X_rr = -6.413e-4
        self.X_vvvv = 4.607e-2
        self.Y_v = -1.100e-2
        self.Y_r = 3.673e-3
        self.Y_vvv = -1.057e-1
        self.Y_rrr = 3.942e-4
        self.Y_vvr = 4.926e-3
        self.Y_vrr = -3.252e-2
        self.N_v = -7.949e-3
        self.N_r = -2.618e-3
        self.N_vvv = 2.861e-4
        self.N_rrr = -1.503e-3
        self.N_vvr = -2.736e-2
        self.N_vrr = -4.000e-3
        self.X_udot = -6.525e-4
        self.Y_vdot = -1.206e-2
        self.Y_rdot = -1.229e-3
        self.N_vdot = -2.019e-4
        self.N_rdot = -9.844e-4
        self.one_minus_t_r = 0.729
        self.one_plus_alpha_h = 0.851
        self.xr_plus_ahxh = -0.740
        self.C_r0 = 0.7841e-3
        self.C_r4 = 607e-3
        self.C_r3 = -223.26e-3
        self.C_r2 = 39.022e-3
        self.C_r1 = -4.646e-3
    
        # coefficient that is taken in model test
        self.eta = 0.95
        self.lr = -0.70

        self.u_ = self.u/self.U
        self.v_ = self.v/self.U
        self.r_ = self.r/(self.U/self.model_LBP)
        self.r_deg_ = self.r_deg/(self.U/self.model_LBP)

        # for cramer's rule
        self.model_m_ = self.model_m/((0.5*self.model_density*(self.model_LBP**3)))
        self.A_matrix = np.array([[self.model_m_-self.Y_vdot, -self.Y_rdot+self.model_m_*self.x_G_],[self.model_m_*self.x_G_-self.N_vdot, self.I_zz_-self.N_rdot]])

    def resistance_test(self, u):
        Re = u*self.model_LBP/self.kinematic_viscousity
        Fr = u/math.sqrt(9.81*self.model_LBP)
        C_f = 0.075/((math.log10(Re)-2)**2)
        C_r = self.C_r4*Fr**4+self.C_r3*Fr**3+self.C_r2*Fr**2+self.C_r1*Fr+self.C_r0
        C_t = C_f + C_r
        R = C_t*self.model_density*(u**2)*self.model_WSA*0.5
        R_ = R/(0.5*self.model_density*self.model_LBP**2*self.U**2)

        n = (self.n1*Fr + self.n0) * math.sqrt(self.scale)
        w = 0.23 #self.w0 + self.w1*u
        t = 0.162 #self.t0 + self.t1*u

        return R_, w, t, n

    def POW_test(self,J_p):
        K_t = self.K_t0 + self.K_t1*J_p + self.K_t2*(J_p**2)

        return K_t
    
    def propeller_inflow_velocity(self,w,u_,v_,r_,rps):
        v_p_ = -(v_+ self.x_p_*r_)

        if v_p_ < 0:
            C_p = self.C_p_plus
        elif v_p_ > 0:
            C_p = self.C_p_minus
        else:
            C_p = 0

        w_p = w*math.exp(-C_p*v_p_**2)
        u_p_ = (1-w_p)*u_
        J_p = u_p_*self.U/(self.model_propeller_diameter*rps)

        return u_p_,v_p_,J_p
    
    def rudder_inflow_velocity(self,u_p_,v_,r_,K_t,J_p):
        u_r_=self.epsilon * u_p_ * math.sqrt(self.eta * (1 + self.kappa *(math.sqrt(1 + 8* K_t/(math.pi*(J_p**2)))-1))**2 + (1 - self.eta))

        beta = -(v_+self.lr*r_)

        if beta < 0:
            gamma = self.gamma_plus
        elif beta > 0:
            gamma = self.gamma_minus
        else:
            gamma = 0

        v_r_ = gamma*(beta)

        return u_r_, v_r_
    
    def thrust_and_rudder_force(self,U,K_t,u_r,v_r,rps,delta,t):
        delta = -delta
        X_P = (1-t)*K_t*(rps**2)*(self.model_propeller_diameter**4)
        X_P_ = X_P/(0.5*self.model_LBP**2*(U**2))

        alpha_r = delta - math.atan2(v_r,u_r)
        F_N_ = -0.5*self.model_A_r*(u_r**2+v_r**2)*self.f_alpha*math.sin(alpha_r)/(0.5*self.model_LBP**2)
        X_R_ = self.one_minus_t_r*F_N_*math.sin(delta)
        Y_R_ = self.one_plus_alpha_h*F_N_*math.cos(delta)
        N_R_ = self.xr_plus_ahxh*F_N_*math.cos(delta)

        return X_P_, X_R_, Y_R_, N_R_

    def hull_force_without_acceleration(self,R_,v_,r_):
        X_H_ = (self.X_vv*(v_**2)) + (self.X_vr*(v_*r_)) + (self.X_rr*(r_**2)) + (self.X_vvvv*(v_**4)) - R_
        Y_H_ = (self.Y_v*v_) + (self.Y_r*r_) + (self.Y_vvv*(v_**3)) + (self.Y_rrr*(r_**3)) + (self.Y_vvr*(v_**2)*r_) + (self.Y_vrr*v_*(r_**2))
        N_H_ = (self.N_v*v_) + (self.N_r*r_) + (self.N_vvv*(v_**3)) + (self.N_rrr*(r_**3)) + (self.N_vvr*(v_**2)*r_) + (self.N_vrr*v_*(r_**2))

        return X_H_,Y_H_,N_H_
    
    def acceleration(self,X_H_,Y_H_,N_H_,X_P_,X_R_,Y_R_,N_R_,u_,v_,r_):
        u_dot_ = (X_H_ + X_P_ + X_R_ + (v_*r_*self.model_m_) + self.model_m_*self.x_G_*(r_**2))/(self.model_m_-self.X_udot)
        B_matrix = np.array([Y_H_+Y_R_-u_*r_*self.model_m_,N_H_+N_R_-self.model_m_*self.x_G_*u_*r_])
        v_dot_and_r_dot = np.linalg.solve(self.A_matrix,B_matrix)
        v_dot_ = v_dot_and_r_dot[0]
        r_dot_ = v_dot_and_r_dot[1]

        return u_dot_,v_dot_,r_dot_

    def main(self,delta,n):
        R_, w, t, _ = self.resistance_test(self.u)
        u_p_,v_p_,J_p = self.propeller_inflow_velocity(w,self.u_,self.v_,self.r_,self.rps)
        K_t = self.POW_test(J_p)
        u_r_, v_r_ = self.rudder_inflow_velocity(u_p_,self.v_,self.r_,K_t,J_p)
        X_P_, X_R_, Y_R_, N_R_ = self.thrust_and_rudder_force(self.U,K_t,u_r_,v_r_,n,delta,t)
        X_H_,Y_H_,N_H_ = self.hull_force_without_acceleration(R_,self.v_,self.r_)
        u_dot_,v_dot_,r_dot_ = self.acceleration(X_H_,Y_H_,N_H_,X_P_,X_R_,Y_R_,N_R_,self.u_,self.v_,self.r_)
        u_dot = u_dot_*(self.U**2/self.model_LBP)
        v_dot = v_dot_*(self.U**2/self.model_LBP)
        r_dot = r_dot_*((self.U**2/self.model_LBP**2))

        return np.array([[u_dot],[v_dot],[r_dot]])

