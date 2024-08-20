import sys, os
from math import *
import numpy as np
from numpy import deg2rad,rad2deg
# import rospy

class CRI:
    def __init__(self, L, B, Xo, Yo, Xt, Yt, Co, Ct, Vo, Vt, ship_scale):
        self.ship_scale = ship_scale
        self.L = L / self.ship_scale    #타선의 길이 [m] from pram
        self.B = B / self.ship_scale     #타선의 폭 [m]
        self.Xo = Xo    #자선 x좌표  [m]
        self.Yo = Yo    #자선 y좌표  [m] 
        self.Xt = Xt    #타선 x좌표  [m]
        self.Yt = Yt    #타선 y좌표  [m]
        self.Co = Co    #자선 Heading angle [rad]
        self.Ct = Ct    #타선 Heading angle [rad]
        self.Vo = Vo    #자선 속도   [knots]
        self.Vt = Vt    #타선 속도   [knots]
        self.ratio = 1852 / self.ship_scale #1852/110  #1 해리는 1852m
        # self.ratio = (12*self.L) / self.ship_scale #1852/110  #1 해리는 1852m

    def RD(self):
        '''Relative Distance, 자선과 타선 사이의 상대 거리'''
        result = sqrt(((self.Xt - self.Xo) ** 2) + ((self.Yt - self.Yo) ** 2)) + 0.0001
        # print(result)
        return result

    def TB(self):
        '''True Bearing, 자선의 위치 기준 타선의 절대 방위, rad'''
        Xot = self.Xt - self.Xo
        Yot = self.Yt - self.Yo
        result = atan2(Yot, Xot) % (2*pi)
        return result

    def RB(self):
        '''Relative Bearing, 자선의 Heading angle에 대한 타선의 방위, rad'''
        if self.TB() - self.Co >= 0:
            result = self.TB() - self.Co
        else:
            result = self.TB() - self.Co + (2 * pi)
        return result
    
    def HAD(self):
        '''Heading angle difference, rad'''
        result = self.Ct - self.Co
        if result < 0 :
            result += 2*pi
        return result

    def Vox(self):
        '''자선 x방향 속도'''
        result = self.Vo * cos(self.Co)
        return result

    def Voy(self):
        '''자선 y방향 속도'''
        result = self.Vo * sin(self.Co)
        return result

    def Vtx(self):
        '''타선 x방향 속도'''
        result = self.Vt * cos(self.Ct)
        return result

    def Vty(self):
        '''타선 y방향 속도'''
        result = self.Vt * sin(self.Ct)
        return result

    def Vrx(self):
        result = self.Vtx() - self.Vox()
        return result

    def Vry(self):
        result = self.Vty() - self.Voy()
        return result

    def RV(self):
        '''Relative Velocity, 자선에 대한 타선의 상대속도'''
        result = sqrt(pow(self.Vrx(), 2) + pow(self.Vry(), 2)) + 0.001
        return result

    def RC(self):
        '''Relative speed heading direction, 상대속도(RV)의 방향'''
        result = atan2(self.Vry(), self.Vrx()) % (2*pi)
        return result

    def tcpa(self):
        result = (self.RD() * cos(self.RC() - self.TB() - pi))/self.RV()
        return result

    def dcpa(self):
        result = sqrt(pow(self.RD(), 2) + pow((self.tcpa() * self.RV()), 2))
        return result

    def d1(self):
        '''Safe approaching distance'''
        RB = np.rad2deg(self.RB())
        if 0 <= RB < 112.5:
            result = self.ratio * (1.1 - 0.2 * (self.RB()/pi))
        elif 112.5 <= RB < 180:
            result = self.ratio * (1.0 - 0.4 * (self.RB()/pi))
        elif 180 <= RB < 247.5:
            result = self.ratio * (1.0 - 0.4 * ((2 * pi - self.RB())/pi))
        else:
            result = self.ratio * (1.1 - 0.2 * ((2 * pi - self.RB())/pi))
        # result = self.ratio * (1.1 - 0.2 * (self.RB()/pi))
        # print(self.RB())
        return result

    def d2(self):
        '''Safe passing distance'''
        result = 2 * self.d1()
        return result

    def UDCPA(self):
        '''#d1, d2의 범위에 따른 DCPA의 계수'''
        if abs(self.dcpa()) <= self.d1():
            result = 1
        elif self.d2() < abs(self.dcpa()):
            result = 0
        else:
            result = 0.5 - 0.5 * sin((pi/(self.d2() - self.d1())) * (abs(self.dcpa()) - (self.d1() + self.d2())/2))
        return result

    def D1(self):
        '''Distance of action'''
        result = 12 * self.L
        return result

    def D2(self):
        '''Distance of last action'''
        result = self.ratio * (1.7 * cos(self.RB() - np.deg2rad(19))) + sqrt(4.4 + 2.89 * pow(cos(self.RB() - np.deg2rad(19)), 2))
        return result

    def UD(self):
        '''D1, D2의 범위에 따른 Relative distance의 계수'''
        if self.RD() <= self.D1():
            result = 1
        elif self.D2() < self.RD():
            result = 0
        else:
            result = pow((self.D2() - self.RD())/(self.D2() - self.D1()), 2)
        return result

    def t1(self):
        '''Collision time'''
        D1 = self.D1()
        if abs(self.dcpa()) <= D1:
            result = sqrt(pow(D1, 2) - pow(self.dcpa(), 2)) / self.RV()
        else:
            result = (D1 - abs(self.dcpa())) / self.RV()
        return result

    def t2(self):
        '''Avoidance time'''
        # D2 = 12 * self.ratio  # 원래 이렇게 되어 있었음
        # D2 = 12 * 12  # 동훈 논문
        D2 = self.D2()  # 다른 논문
        if abs(self.dcpa()) <= D2:
            result = sqrt(pow(D2, 2) - pow(self.dcpa(), 2)) / self.RV()
        else:
            result = (D2 - abs(self.dcpa())) / self.RV()
        return result

    def UTCPA(self):
        '''t1, t2의 범위에 따른 TCPA의 계수'''
        if self.tcpa() < 0:
            result = 0
        else:
            if self.tcpa() <= self.t1():
                result = 1
            elif self.t2() < self.tcpa():
                result = 0
            else:
                result = pow(((self.t2() - abs(self.tcpa()))/(self.t2() - self.t1())), 2)
        return result

    def UB(self):
        '''Relative bearing에 대한 계수 UB'''
        result = 0.5 * (cos(self.RB() - np.deg2rad(19)) + sqrt((440/289) + pow(cos(self.RB() - np.deg2rad(19)), 2))) - (5/17)
        return result

    def K(self):
        '''Speed factor'''
        if self.Vt == 0 or self.Vo == 0:
            result = 0.001
        else:
            result = self.Vt / self.Vo
        return result

    def sinC(self):
        '''Collision angle, UK의 계산에 사용'''
        result = abs(sin(abs(self.Ct - self.Co)))
        return result

    def UK(self):
        '''Speed factor에 대한 계수 UK'''
        result = 1 / (1 + (2 / (self.K() * sqrt(pow(self.K(), 2) + 1 + (2 * self.K() * self.sinC())))))
        return result

    def CRI(self):
        '''충돌위험도지수, UDCPA, UTCPA, UD, UB, UK 5개의 파라미터에 가중치를 곱하여 계산'''
        result = 0.4 * self.UDCPA() + 0.367 * self.UTCPA() + 0.133 * self.UD() + 0.067 * self.UB() + 0.033 * self.UK() #원래 값
        # result = 0.4457 * self.UDCPA() + 0.2258 * self.UTCPA() + 0.1408 * self.UD() + 0.1321 * self.UB() + 0.0556 * self.UK() #원준 수정 값
        return round(result, 3)

    def encounter_classification(self):
        HAD = np.rad2deg(self.HAD())
        RB = np.rad2deg(self.RB())

        if 0 <= RB <= 22.5 or 337.5 <= RB <= 360:
            if 157.5 <= HAD <= 202.5:
                return "Head-on"
            elif 67.5 <= HAD < 157.5:
                return "Port crossing"
            elif 202.5 < HAD <= 292.5:
                return "Starboard crossing"
            else:
                return "Overtaking"


        elif 22.5 < RB <= 90:
            if 157.5 <= HAD <= 202.5:
                return "Head-on"
            elif 67.5 <= HAD < 157.5:
                return "Safe"
            elif 202.5 < HAD <= 292.5:
                return "Starboard crossing"
            else:
                return "Overtaking"


        elif 90 < RB <= 112.5:
            if 67.5 <= HAD < 202.5:
                return "Safe"
            elif 202.5 < HAD <= 292.5:
                return "Starboard crossing"
            else:
                return "Overtaking"


        elif 247.5 <= RB < 270:
            if 157.5 <= HAD <= 292.5:
                return "Safe"
            elif 67.5 <= HAD < 157.5:
                return "Port crossing"
            else:
                return "Overtaking"


        elif 270 <= RB < 337.5:
            if 157.5 <= HAD <= 202.5:
                return "Head-on"
            elif 67.5 <= HAD < 157.5:
                return "Port crossing"
            elif 202.5 < HAD <= 292.5:
                return "Safe"
            else:
                return "Overtaking"

        else:
            if 0 <= HAD <= 67.5 or 292.5 <= HAD <= 360:
                if self.Vt > self.Vo:
                    return "Overtaking"
                else:
                    return "Safe"
            else:
                return "Safe"

    def CoE(self):
        '''Coefficients of encounter situations'''
        # a = self.encounter_classification()
        # print(a)
        if self.encounter_classification() == "Head-on":
            s = abs(2 - (self.Vo - self.Vt)/self.Vo)
            t = 0.2
        elif self.encounter_classification() == "Starboard crossing" or self.encounter_classification() == "Port crossing":
            s = 2 - self.HAD()/pi
            t = self.HAD()/pi
        elif self.encounter_classification() == "Overtaking":
            s = 1
            t = 0.2
        else:
            s = abs(1 + (self.Vo - self.Vt)/self.Vo)
            t = abs(0.5 + (self.Vo - self.Vt)/self.Vo)
        return s, t

    def ship_domain(self):
        if self.Vo <= 0.0: 
            self.Vo = 0.1

        KAD = pow(10, (0.3591 * log10(self.Vo) + 0.0952))  ## 논문에서 보면 지수함수를 사용
        KDT = pow(10, (0.5411 * log10(self.Vo) - 0.0795))  ## 논문에서 보면 지수함수를 사용 -> 
        AD = self.L * KAD
        DT = self.L * KDT

        s, t = self.CoE()

        R_fore = self.L + (0.67 * (1 + s) * sqrt(pow(AD,2) + pow(DT/2,2)))
        R_aft = self.L + (0.67 * sqrt(pow(AD,2) + pow(DT/2,2)))
        R_stbd = self.B + DT * (1 + t)
        R_port = self.B + (0.75 * DT * (1 + t))

        return R_fore, R_aft, R_stbd, R_port

    def Rf(self):
        SD = self.ship_domain()
        result = SD[0]
        return result

    def Ra(self):
        SD = self.ship_domain()
        result = SD[1]
        return result

    def Rs(self):
        SD = self.ship_domain()
        result = SD[2]
        return result

    def Rp(self):
        SD = self.ship_domain()
        result = SD[3]
        return result

    #Ship domain distance
    def SD_dist(self):
        RB = np.rad2deg(self.RB())
        Rf, Ra, Rs, Rp = self.Rf(), self.Ra(), self.Rs(), self.Rp()
        if 0 <= RB < 90:
            result = sqrt(pow(Rf,2)/(pow(sin(RB),2) + pow(cos(RB),2) * (pow(Rf,2)/pow(Rs,2))))
        elif 90 <= RB < 180:
            result = sqrt(pow(Ra,2)/(pow(sin(RB),2) + pow(cos(RB),2) * (pow(Ra,2)/pow(Rs,2))))
        elif 180 <= RB < 270:
            result = sqrt(pow(Ra,2)/(pow(sin(RB),2) + pow(cos(RB),2) * (pow(Ra,2)/pow(Rp,2))))
        else:
            result = sqrt(pow(Rf,2)/(pow(sin(RB),2) + pow(cos(RB),2) * (pow(Rf,2)/pow(Rp,2))))

        return result
    
    # 선박 안전 영역을 4가지 방향에 대해서 계산
    # 타선을 원점으로 하는 4개의 점이 생성됨
    # 생성된 4개의 점을 순서쌍으로 만들고 자선과 두 점 사이의 각이 최대인 경우의 왼쪽 바운더리와 오른쪽 바운더리를 반환
    # 아웃풋은 왼쪽 바운더리와 오른쪽 바운더리
    def SD_dist_new(self):
        Rf, Ra, Rs, Rp = self.Rf(), self.Ra(), self.Rs(), self.Rp()

        param = 4
        Xot = self.Xt-self.Xo
        Yot = self.Yt-self.Yo
        Rf_position = np.array([param*Rf*cos(deg2rad(self.Ct)),param*Rf*sin(deg2rad(self.Ct))]) + np.array([Xot,Yot])
        Ra_position = np.array([param*Ra*cos(deg2rad(self.Ct)+pi),param*Ra*sin(deg2rad(self.Ct)+pi)]) + np.array([Xot,Yot])
        Rs_position = np.array([param*Rs*cos(deg2rad(self.Ct)-pi/2),param*Rs*sin(deg2rad(self.Ct)-pi/2)]) + np.array([Xot,Yot])
        Rp_position = np.array([param*Rp*cos(deg2rad(self.Ct)+pi/2),param*Rp*sin(deg2rad(self.Ct)+pi/2)]) + np.array([Xot,Yot])

        Rf_rad = atan2(Rf_position[1],Rf_position[0])
        Ra_rad = atan2(Ra_position[1],Ra_position[0])
        Rs_rad = atan2(Rs_position[1],Rs_position[0])
        Rp_rad = atan2(Rp_position[1],Rp_position[0])

        R_rad_list = [Rf_rad,Ra_rad,Rs_rad,Rp_rad]
        new_rad_list = []

        max_angle = 0

        for j in R_rad_list:
            for k in R_rad_list:
                if abs(j - k) > pi:
                    angle = abs(abs(j-k) - 2*pi)
                else:
                    angle = abs(j-k)

                if angle > max_angle:
                    max_angle = angle
                    new_rad_list = [j,k]

        if abs(new_rad_list[0]-new_rad_list[1]) > pi:
            right_bound = max(new_rad_list)
            left_bound = min(new_rad_list)

        else:
            right_bound = min(new_rad_list)
            left_bound = max(new_rad_list)

        return right_bound, left_bound