from numpy import deg2rad, rad2deg
from math import *
from numpy import rad2deg

import numpy as np
import pymap3d as pm
import pyproj

class CRI:
    def __init__(self, latitude, longitude, latOfObject, longOfObject, cog, cogOfObject, sog, sogOfObject, parameter):
        self.L = parameter['ship_L']
        self.B = parameter['ship_B']
        self.Xo = latitude
        self.Yo = longitude
        self.Xt = latOfObject
        self.Yt = longOfObject
        self.Co = cog 
        self.Ct = cogOfObject
        self.Vo = sog 
        self.Vt = sogOfObject
        self.ratio = parameter['cri_ratio']

    def RD(self):
        result = sqrt(((self.Xt - self.Xo) ** 2) + ((self.Yt - self.Yo) ** 2)) + 0.0001
        return result

    def TB(self):
        Xot = self.Xt - self.Xo
        Yot = self.Yt - self.Yo
        result = atan2(Yot, Xot) % (2*pi)
        return result

    def RB(self):
        if self.TB() - self.Co >= 0:
            result = self.TB() - self.Co
        else:
            result = self.TB() - self.Co + (2 * pi)
        return result

    def HAD(self):
        result = self.Ct - self.Co
        if result < 0 :
            result += 2*pi
        return result

    def Vox(self):
        result = self.Vo * cos(self.Co)
        return result

    def Voy(self):
        result = self.Vo * sin(self.Co)
        return result

    def Vtx(self):
        result = self.Vt * cos(self.Ct)
        return result

    def Vty(self):
        result = self.Vt * sin(self.Ct)
        return result

    def Vrx(self):
        result = self.Vtx() - self.Vox()
        return result

    def Vry(self):
        result = self.Vty() - self.Voy()
        return result

    def RV(self):
       
        result = sqrt(pow(self.Vrx(), 2) + pow(self.Vry(), 2)) + 0.001
        return result

    def RC(self):
       
        result = atan2(self.Vry(), self.Vrx()) % (2*pi)
        return result

    def tcpa(self):
        result = (self.RD() * cos(self.RC() - self.TB() - pi))/self.RV()
        return result

    def dcpa(self):
        result = sqrt(pow(self.RD(), 2) + pow((self.tcpa() * self.RV()), 2))
        return result

    def d1(self):
        result = self.ratio * (1.1 - 0.2 * (self.RB()/pi))
        return result

    def d2(self):
       
        result = 2 * self.d1()
        return result

    def UDCPA(self):
       
        if abs(self.dcpa()) <= self.d1():
            result = 1
        elif self.d2() < abs(self.dcpa()):
            result = 0
        else:
            result = 0.5 - 0.5 * sin(((pi/(self.d2() - self.d1())) * abs(self.dcpa())) - ((self.d1() + self.d2())/2))
        return result

    def D1(self):
       
        result = 12 * self.L
        return result

    def D2(self):
       
        result = self.ratio * (1.7 * cos(self.RB() - np.deg2rad(19))) + sqrt(4.4 + 2.89 * pow(cos(self.RB() - np.deg2rad(19)), 2))
        return result

    def UD(self):
       
        if self.RD() <= self.D1():
            result = 1
        elif self.D2() < self.RD():
            result = 0
        else:
            result = pow((self.D2() - self.RD())/(self.D2() - self.D1()), 2)
        return result

    def t1(self):
       
        D1 = self.D1()
        if abs(self.dcpa()) <= D1:
            result = sqrt(pow(D1, 2) - pow(self.dcpa(), 2)) / self.RV()
        else:
            result = (D1 - abs(self.dcpa())) / self.RV()
        return result

    def t2(self):
       
        D2 = 12 * self.ratio
        if abs(self.dcpa()) <= D2:
            result = sqrt(pow(D2, 2) - pow(self.dcpa(), 2)) / self.RV()
        else:
            result = (D2 - abs(self.dcpa())) / self.RV()
        return result

    def UTCPA(self):
       
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
        result = 0.4 * self.UDCPA() + 0.367 * self.UTCPA() + 0.133 * self.UD() + 0.067 * self.UB() + 0.033 * self.UK()
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
        if self.Vo == 0.0: 
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
    

class Inha_dataProcess:
    """inha_module의 data 송신을 위해 필요한 함수들이 정의됨"""
    def __init__(self, idOfObject, latitude, longitude, cog, sog, latOfObject, longOfObject, cogOfObject, sogOfObject, parameter):

        self.idOfObject = idOfObject
        self.Pos_X = latitude
        self.Pos_Y = longitude
        self.Vel_U = sog * 0.5144
        self.Heading = cog
        self.waypoint_dict = dict()
        
        self.latOfObject = latOfObject
        self.longOfObject = longOfObject
        self.cogOfObject = cogOfObject
        self.sogOfObject = sogOfObject
        self.ship_dic= {}
        self.SD_param = parameter['SD_param']
        self.ship_L = parameter['ship_L']
        self.ship_B = parameter['ship_B']
        self.parameter = parameter

    def os_info(self):
        os_latitude = self.Pos_X
        os_longitude = self.Pos_Y
        os_cog = self.Heading
        os_sog = self.Vel_U
        
        OS_list = {
            'shipID' : int,
            'Pos_X' : os_latitude,
            'Pos_Y' : os_longitude,
            'Vel_U' : os_sog,
            'Heading' : os_cog,
            }
        
        return OS_list
    
    def ts_info(self):
        TS_list = dict()
        ts_idOfObject = self.idOfObject 
        ts_latOfObject = self.latOfObject
        ts_longOfObject = self.longOfObject
        ts_cogOfObject = self.cogOfObject
        ts_sogOfObject = self.sogOfObject

        for i in range(len(self.idOfObject)):
            index_ship = self.idOfObject[i]
            TS_list[index_ship] ={
                'ship_ID' : int(ts_idOfObject[i]),
                'Pos_X' : ts_latOfObject[i],
                'Pos_Y' : ts_longOfObject[i],
                'Vel_U' : ts_sogOfObject[i]*0.5144,
                'Heading' : ts_cogOfObject[i],
            }
        return TS_list


    def CRI_cal(self, OS, TS):
        cri = CRI(
            OS['Pos_X'],
            OS['Pos_Y'],
            TS['Pos_X'],
            TS['Pos_Y'],
            deg2rad(OS['Heading']),
            deg2rad(TS['Heading']),
            OS['Vel_U'],
            TS['Vel_U'],
            self.parameter,
        )

        RD = cri.RD()
        TB = rad2deg(cri.TB())
        RB = rad2deg(cri.RB())

        Vox = cri.Vox()
        Voy = cri.Voy()
        Vtx = cri.Vtx()
        Vty = cri.Vty()

        DCPA = cri.dcpa()
        TCPA = cri.tcpa()

        UDCPA = cri.UDCPA()
        UTCPA = cri.UTCPA()
        UD = cri.UD()
        UB = cri.UB()
        UK = cri.UK()

        enc = cri.encounter_classification()

        Rf = cri.Rf()
        Ra = cri.Ra()
        Rs = cri.Rs()
        Rp = cri.Rp()
        SD_dist = cri.SD_dist()

        cri_value = cri.CRI()

        return RD, TB, RB, Vox, Voy, Vtx, Vty, DCPA, TCPA, UDCPA, UTCPA, UD, UB, UK, enc, Rf, Ra, Rs, Rp, SD_dist, cri_value

    def U_to_vector_V(self, U, deg):
        ''' Heading angle을 지구좌표계 기준의 속도벡터로 변환

        Return:
            Vx, Vy  [m/s]     
        '''
        psi = deg2rad(deg) ##강제로 xy좌표에서 NED좌표로 변환
        
        V_x = U * cos(psi)
        V_y = U * sin(psi)

        return V_x, V_y

    def waypoint_generator(self, OS, V_selected):
  
        wp_x = []
        wp_y = []
        OS_X = np.array([OS['Pos_X'], OS['Pos_Y']])
        for i in range(1, 16):
            wp = OS_X + V_selected * np.array([i])
            
            wp_x.append(wp[0]) 
            wp_y.append(wp[1])

        return wp_x, wp_y
 

    def desired_value_assumption(self, V_des):
        
        U_des = sqrt(V_des[0]**2 + V_des[1]**2)
        target_head = rad2deg(atan2(V_des[1], V_des[0]))
        desired_heading = target_head
        desired_spd = U_des
        
        desired_spd_list = []
        desired_heading_list = []
                
        for i in range(15):
            desired_heading_list.append(desired_heading)
            desired_spd_list.append(desired_spd)      

        return desired_spd_list, desired_heading_list

    def TS_info_supplement(self, OS_list, TS_list):   

        TS_ID = TS_list.keys()
        for ts_ID in TS_ID:
            RD, TB, RB, Vox, Voy, Vtx, Vty, DCPA, TCPA, UDCPA, UTCPA, UD, UB, UK, enc, Rf, Ra, Rs, Rp, SD_dist, cri_value = self.CRI_cal(OS_list, TS_list[ts_ID])

            TS_list[ts_ID]['RD'] = RD 
            TS_list[ts_ID]['TB'] = TB  
            TS_list[ts_ID]['RB'] = RB

            TS_list[ts_ID]['V_x'] = Vtx
            TS_list[ts_ID]['V_y'] = Vty

            TS_list[ts_ID]['DCPA'] = DCPA
            TS_list[ts_ID]['TCPA'] = TCPA

            TS_list[ts_ID]['UDCPA'] = UDCPA
            TS_list[ts_ID]['UTCPA'] = UTCPA
            TS_list[ts_ID]['UD'] = UD
            TS_list[ts_ID]['UB'] = UB
            TS_list[ts_ID]['UK'] = UK

            TS_list[ts_ID]['status'] = enc

            TS_list[ts_ID]['Rf'] = Rf
            TS_list[ts_ID]['Ra'] = Ra
            TS_list[ts_ID]['Rs'] = Rs
            TS_list[ts_ID]['Rp'] = Rp
            TS_list[ts_ID]['mapped_radius'] = SD_dist * self.SD_param

            TS_list[ts_ID]['CRI'] = cri_value


        return TS_list


class VO_module:
    def __init__(self, parameter):
        # NOTE: It is not clear what min and max of speed could be.
        self.min_targetSpeed = parameter['min_targetSpeed']
        self.max_targetSpeed = parameter['max_targetSpeed']
        self.num_targetSpeedCandidates = parameter['num_targetSpeedCandidates']

        # NOTE: It is not clear what min and max of heading angle could be.
        self.min_targetHeading_deg_local = parameter['min_targetHeading_deg_local']
        self.max_targetHeading_deg_local = parameter['max_targetHeading_deg_local']
        self.num_targetHeadingCandidates = parameter['num_targetHeadingCandidates']

        self.weight_alpha = parameter['weight_focusObs']
        self.weight_aggresiveness = parameter['weight_agressivness']
        self.cri_param = parameter['cri_param']
        self.time_horizon = parameter['timeHorizon']
        self.delta_t = parameter['delta_t']

        self.rule = parameter['Portside_rule']
        self.errorCode = None
        self.error = False
        
        
    def __is_all_vels_collidable(self, vel_all_annotated, shipID_all):
        for vel_annotated in vel_all_annotated:
            isVelCollidable = False
            for shipID in shipID_all:
                if (vel_annotated[shipID] == 'inCollisionCone'):
                   isVelCollidable = True
                   break
            if not isVelCollidable:
                return False

        return True

    def __is_all_vels_avoidable(self, vel_all_annotated, shipID_all):
        for vel_annotated in vel_all_annotated:
            for shipID in shipID_all:
                if (vel_annotated[shipID] == 'inCollisionCone'):
                   return False

        return True

    
    def __remove_annotation(self, vel_annotated_all):
        vels = []
        
        for vel_annotated in vel_annotated_all:
            vels.append(vel_annotated['vel'])
            
        return vels

    def __annotate_vels(self, reachableVel_global_all, RVOdata_all, TS):
        # Make the `reachableVel_global_all` dictionary
        reachableVel_global_all_annotated = []

        for reachableVel_global in reachableVel_global_all:
            
            reachableVel_global_annotated = {'vel': reachableVel_global}

            for RVOdata in RVOdata_all:
                '''
                `RVOdata` structure:
                    RVOdata = {
                        "TS_ID",
                        "LOSdist", 
                        "mapped_radius", 
                        "vA",
                        "vB",
                        "boundLineAngle_left_rad_global", 
                        "boundLineAngle_right_rad_global", 
                        "collisionConeShifted_local",
                        "CRI"
                        }
                '''
                # NOTE: vA2B_RVO is the relative velocity from the agent A to B
                #       on the RVO configuration space, not the VO configuration space
                vA2B_RVO = reachableVel_global - RVOdata['collisionConeTranslated']

                angle_vA2B_RVO_rad_global = atan2(
                    vA2B_RVO[1],
                    vA2B_RVO[0],
                    )
                
                if self.__is_in_left(
                    velVecAngle_rad_global=angle_vA2B_RVO_rad_global, 
                    boundLineAngle_left_rad_global=RVOdata['boundLineAngle_left_rad_global'],
                    boundLineAngle_right_rad_global=RVOdata['boundLineAngle_right_rad_global'],
                    LOSangle_rad_global=0.5*(RVOdata['boundLineAngle_left_rad_global']+RVOdata['boundLineAngle_right_rad_global']),
                    ):

                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inLeft'
                
                elif self.__is_in_right(
                    velVecAngle_rad_global=angle_vA2B_RVO_rad_global, 
                    boundLineAngle_left_rad_global=RVOdata['boundLineAngle_left_rad_global'],
                    boundLineAngle_right_rad_global=RVOdata['boundLineAngle_right_rad_global'],
                    LOSangle_rad_global=0.5*(RVOdata['boundLineAngle_left_rad_global']+RVOdata['boundLineAngle_right_rad_global']),
                    ):

                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inRight'
                
                elif self.__is_within_time_horizon(
                    velVecAngle_rad_global=angle_vA2B_RVO_rad_global, 
                    boundLineAngle_left_rad_global=RVOdata['boundLineAngle_left_rad_global'], 
                    boundLineAngle_right_rad_global=RVOdata['boundLineAngle_right_rad_global'],
                    velVecNorm=np.linalg.norm(vA2B_RVO),
                    shortestRelativeDist=RVOdata['LOSdist']-RVOdata['mapped_radius'],
                    timeHorizon=RVOdata['CRI']*self.cri_param,
                    # timeHorizon=self.time_horizon
                    ):

                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inTimeHorizon'
                
                elif self.__is_in_collision_cone(
                    velVecAngle_rad_global=angle_vA2B_RVO_rad_global, 
                    boundLineAngle_left_rad_global=RVOdata['boundLineAngle_left_rad_global'], 
                    boundLineAngle_right_rad_global=RVOdata['boundLineAngle_right_rad_global'],
                    velVecNorm=np.linalg.norm(vA2B_RVO),
                    shortestRelativeDist=RVOdata['LOSdist']-RVOdata['mapped_radius'],
                    timeHorizon=RVOdata['CRI']*self.cri_param,
                    # timeHorizon=self.time_horizon
                    ):
                    
                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inCollisionCone'

                else:
                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inCollisionCone'

            reachableVel_global_all_annotated.append(reachableVel_global_annotated)

        return reachableVel_global_all_annotated

    def __take_vels(self, vel_all_annotated, annotation, shipID_all):
        vel_hasAnnotation_all_annotated = []
        for vel_annotated in vel_all_annotated:
            hasAnnotation = True
            for shipID in shipID_all:
                if vel_annotated[shipID] not in annotation:
                    hasAnnotation = False
                    break
            if hasAnnotation:
                vel_hasAnnotation_all_annotated.append(vel_annotated)

        return vel_hasAnnotation_all_annotated

    def __is_in_between(self, theta_given, theta_left, theta_right):
        if abs(theta_right - theta_left) <= pi:
            if theta_left < 0 and theta_right < 0:
                if theta_given > 0:
                    theta_given -= 2*pi
            elif theta_left > 0 and theta_right > 0:
                if theta_given < 0:
                    theta_given += 2*pi
            if theta_right <= theta_given <= theta_left:    
                return True
            else :
                return False
        else:
            if (theta_left < 0) and (theta_right > 0):
                ## 각도 보정 
                theta_left += 2*pi
                if theta_given <0:
                    theta_given += 2*pi

                if theta_right <= theta_given <= theta_left:
                    return True
                else :
                    return False
            
            if (theta_left > 0) and (theta_right <0):
                theta_right += 2*pi            
                if theta_given < 0:
                    theta_given += 2*pi
                    
                if theta_left <= theta_given <= theta_right:
                    return True
                else:
                    return False

    def __is_in_left(
        self,  
        velVecAngle_rad_global, 
        boundLineAngle_left_rad_global, 
        boundLineAngle_right_rad_global, 
        LOSangle_rad_global,
        ):
        
        velVecAngle_translated_rad_global = velVecAngle_rad_global - LOSangle_rad_global
        while velVecAngle_translated_rad_global < 0:
            velVecAngle_translated_rad_global += 2*pi
        velVecAngle_translated_rad_global %= (2*pi)

        if self.__is_in_between(
            velVecAngle_rad_global, 
            boundLineAngle_left_rad_global,
            boundLineAngle_right_rad_global,
            ):
            return False
        elif (0 < velVecAngle_translated_rad_global <= pi):
            return True
        elif (pi < velVecAngle_translated_rad_global <= 2*pi):
            return False

    def __is_in_right(
        self, 
        velVecAngle_rad_global, 
        boundLineAngle_left_rad_global, 
        boundLineAngle_right_rad_global, 
        LOSangle_rad_global,
        ):
        
        velVecAngle_translated_rad_global = velVecAngle_rad_global - LOSangle_rad_global
        while velVecAngle_translated_rad_global < 0:
            velVecAngle_translated_rad_global += 2*pi
        velVecAngle_translated_rad_global %= (2*pi)

        if self.__is_in_between(
            velVecAngle_rad_global, 
            boundLineAngle_left_rad_global, 
            boundLineAngle_right_rad_global,
            ):
            return False
        elif (0 < velVecAngle_translated_rad_global <= pi):
            return False
        elif (pi < velVecAngle_translated_rad_global <= 2*pi):
            return True

    def __is_within_time_horizon(
        self, 
        velVecAngle_rad_global, 
        boundLineAngle_left_rad_global, 
        boundLineAngle_right_rad_global, 
        velVecNorm,
        shortestRelativeDist, 
        timeHorizon,
        ):
        if self.__is_in_between(
            velVecAngle_rad_global,
            boundLineAngle_left_rad_global,
            boundLineAngle_right_rad_global,
            ) and (velVecNorm <= shortestRelativeDist / timeHorizon):
            return True
        else:
            return False

    def __is_in_collision_cone(
        self, 
        velVecAngle_rad_global, 
        boundLineAngle_left_rad_global, 
        boundLineAngle_right_rad_global, 
        velVecNorm,
        shortestRelativeDist, 
        timeHorizon,
        ):
        if self.__is_in_between(
            velVecAngle_rad_global,
            boundLineAngle_left_rad_global,
            boundLineAngle_right_rad_global,
            ) and (velVecNorm > shortestRelativeDist / timeHorizon):
            return True
        else:
            return False

    def __generate_vel_candidates(self, targetSpeed_all, targetHeading_rad_global_all, OS, static_obstacle_info, static_point_info):
        reachableVelX_global_all = np.zeros(self.num_targetSpeedCandidates * self.num_targetHeadingCandidates)
        reachableVelY_global_all = np.zeros(self.num_targetSpeedCandidates * self.num_targetHeadingCandidates)
        
        idx = int(0)
        for targetHeading_rad_global in targetHeading_rad_global_all:
            for targetSpeed in targetSpeed_all:

                reachableVelX_global_all[idx] = targetSpeed * cos(targetHeading_rad_global)
                reachableVelY_global_all[idx] = targetSpeed * sin(targetHeading_rad_global)

                idx += 1

        reachableVelX_global_all = np.reshape(reachableVelX_global_all, newshape=(-1, 1))
        reachableVelY_global_all = np.reshape(reachableVelY_global_all, newshape=(-1, 1))

        reachableVel_global_all = np.concatenate(
            (reachableVelX_global_all, reachableVelY_global_all),
             axis=-1,
             )
        

        reachableVel_global_all_after_obstacle = self.__delete_vector_inside_obstacle(reachableVel_global_all, OS, static_obstacle_info,static_point_info)
        

        return reachableVel_global_all_after_obstacle


    def __delete_vector_inside_obstacle(self, reachableVel_global_all, OS, static_obstacle_info, static_point_info):

        pA = np.array([OS['Pos_X'], OS['Pos_Y']])

        reachableVel_global_all_copy = np.copy(reachableVel_global_all)
        static_OB_data = static_obstacle_info
        static_point_data = static_point_info
        
        pA = np.array([OS['Pos_X'], OS['Pos_Y']])
        delta_t = self.delta_t # constant
        detecting_radious = 100

        #initial number for while
        obstacle_number = 0
        point_number = 0

        # obstacle_radious
        radious = 5

        while (obstacle_number) != len(static_OB_data):
            obstacle_point_x = [static_OB_data[obstacle_number],static_OB_data[obstacle_number+2]] 
            obstacle_point_y = [static_OB_data[obstacle_number+1],static_OB_data[obstacle_number+3]] 

            if obstacle_point_x[0] > obstacle_point_x[1]:
                obstacle_point_x.reverse()
                obstacle_point_y.reverse()

            if (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) > 0:
                slope = 9999

            elif (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) < 0:
                slope =-9999

            else: 
                slope = (obstacle_point_y[1]-obstacle_point_y[0])/(obstacle_point_x[1]-obstacle_point_x[0])
                
            reachableVel_global_all_after_delta_t = reachableVel_global_all * delta_t 
            result = []

            for i in range(len(reachableVel_global_all_after_delta_t)): 
                after_delta_t_x = reachableVel_global_all_after_delta_t[i][0]+pA[0]
                after_delta_t_y = reachableVel_global_all_after_delta_t[i][1]+pA[1]
                if (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) < 0:
                    vector_slope = 9999

                elif (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) > 0:
                    vector_slope = -9999
                else:
                    vector_slope = (pA[1]-after_delta_t_y)/(pA[0]-after_delta_t_x)

                if self.get_crosspt(slope, 
                        vector_slope,
                        obstacle_point_x[0], 
                        obstacle_point_y[0],
                        obstacle_point_x[1], 
                        obstacle_point_y[1],
                        pA[0], 
                        pA[1], 
                        after_delta_t_x, 
                        after_delta_t_y):

                    component = reachableVel_global_all[i]
                    component_list = component.tolist()
                    result.append(component_list)
                else:
                    pass

            reachableVel_global_all = np.array(result)

            obstacle_number = obstacle_number+4

        while point_number != len(static_point_data):
        
            point_x = static_point_data[point_number]
            point_y = static_point_data[point_number+1]

            reachableVel_global_all_after_delta_t = reachableVel_global_all * delta_t 
            result = []

            for i in range(len(reachableVel_global_all_after_delta_t)-1): 
                after_delta_t_x = reachableVel_global_all_after_delta_t[i][0]+pA[0]
                after_delta_t_y = reachableVel_global_all_after_delta_t[i][1]+pA[1]

                if (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) < 0:
                    vector_slope = 9999

                elif (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) > 0:
                    vector_slope = -9999
                else:
                    vector_slope = (pA[1]-after_delta_t_y)/(pA[0]-after_delta_t_x)

                if self.get_crosspt_circle(vector_slope,
                        point_x, 
                        point_y, 
                        radious, 
                        pA[0], 
                        pA[1], 
                        after_delta_t_x, 
                        after_delta_t_y ):
                    
                    component = reachableVel_global_all[i]
                    component_list = component.tolist()
                    result.append(component_list)

                else:
                    pass

            reachableVel_global_all = np.array(result)

            point_number = point_number+2

        if len(reachableVel_global_all) == 0:
            reachableVel_global_all = reachableVel_global_all = np.array([reachableVel_global_all_copy[-1,:]])
            print("------ All of vector candidates have collision risk with static obstacle ------")
            self.errorCode = 310
            self.error = True
        else:
            pass

        return reachableVel_global_all
    
    def if_all_vector_collidable(self, OS, effective_static_OB, detecting_radious, reachableVel_global_all_copy):
        space_number = 20
        pA = [OS['Pos_X'], OS['Pos_Y']]
        vector_radian_plus = 0
        vector_radian_minus = 0
        vector_space_radian = pi/space_number
        detecting_vector_list = []
        detecting_vector_list_in_right = []
        detecting_vector_list_in_left = []
        for i in range(space_number):
            vector_radian_plus = vector_radian_plus + vector_space_radian * i
            vector_radian_minus = vector_radian_minus - vector_space_radian * i
            vector_point_plus = [(pA[0]+detecting_radious*cos(vector_radian_plus)),(pA[0]+detecting_radious*sin(vector_radian_plus))]
            vector_point_minus = [(pA[0]+detecting_radious*cos(vector_radian_minus)),(pA[0]+detecting_radious*sin(vector_radian_minus))]
            vector_slope_plus = tan(vector_radian_plus)
            vector_slope_minus = tan(vector_radian_minus)
            obstacle_number = 0

            while (obstacle_number) != len(effective_static_OB):
                obstacle_point_x = [effective_static_OB[obstacle_number],effective_static_OB[obstacle_number+2]] 
                obstacle_point_y = [effective_static_OB[obstacle_number+1],effective_static_OB[obstacle_number+3]] 

                if obstacle_point_x[0] > obstacle_point_x[1]:
                    obstacle_point_x.reverse()
                    obstacle_point_y.reverse()

                if (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) > 0:
                    slope = 9999

                elif (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) < 0:
                    slope =-9999

                else: 
                    slope = (obstacle_point_y[1]-obstacle_point_y[0])/(obstacle_point_x[1]-obstacle_point_x[0])

                if self.get_crosspt(slope, 
                        vector_slope_plus,
                        obstacle_point_x[0], 
                        obstacle_point_y[0],
                        obstacle_point_x[1], 
                        obstacle_point_y[1],
                        pA[0], 
                        pA[1], 
                        vector_point_plus[0], 
                        vector_point_plus[1]):

                    detecting_vector_list_in_right.append(vector_radian_plus)

                elif self.get_crosspt(slope, 
                        vector_slope_minus,
                        obstacle_point_x[0], 
                        obstacle_point_y[0],
                        obstacle_point_x[1], 
                        obstacle_point_y[1],
                        pA[0], 
                        pA[1], 
                        vector_point_minus[0], 
                        vector_point_minus[1]):

                    detecting_vector_list_in_left.append(vector_radian_minus)
                else:
                    pass

            if len(detecting_vector_list) == 0:
                reachableVel_global_all = np.array([reachableVel_global_all_copy[182,:]])

            else:
                if len(detecting_vector_list_in_right) >= len(detecting_vector_list_in_left):
                    reachableVel_global_all = np.array([reachableVel_global_all_copy[182,:]])
                    # select vector that is in right
                else:
                    reachableVel_global_all = np.array([reachableVel_global_all_copy[123,:]])
                    # select vector that is in left

            return reachableVel_global_all

    def get_crosspt(self, slope, vector_slope, start_x, start_y,end_x, end_y, OS_pos_x, OS_pos_y, after_delta_t_x, after_delta_t_y):

        x_point = [start_x, end_x]
        y_point = [start_y, end_y]

        if (slope) == (vector_slope): 
            return True

        else:
            cross_x = (start_x * slope - start_y - OS_pos_x * vector_slope + OS_pos_y) / (slope - vector_slope)
            cross_y = slope * (cross_x - start_x) + start_y

            if OS_pos_x <= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if OS_pos_x <= cross_x <= after_delta_t_x and OS_pos_y <= cross_y <= after_delta_t_y:

                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x >= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if after_delta_t_x <= cross_x <= OS_pos_x and OS_pos_y <= cross_y <= after_delta_t_y:

                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x <= after_delta_t_x and OS_pos_y >= after_delta_t_y:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if OS_pos_x <= cross_x <= after_delta_t_x and  after_delta_t_y <= cross_y <= OS_pos_y:

                        return False
                    else:
                        return True
                else:
                    return True

            else:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if after_delta_t_x <= cross_x <= OS_pos_x and after_delta_t_y <= cross_y <= OS_pos_y:

                        return False
                    else:
                        return True
                else:
                    return True
                
    def get_crosspt_circle(self, vector_slope, point_x, point_y, radious, OS_pos_x, OS_pos_y, after_delta_t_x, after_delta_t_y ):

        point = [point_x,point_y]
        r = radious

        vector_point_x = [OS_pos_x,after_delta_t_x]
        vector_point_y = [OS_pos_y,after_delta_t_y]

        slope = vector_slope

        a = slope
        b = -1
        c = -slope*vector_point_x[0]+vector_point_y[0]

        xp = point[0]
        yp = point[1]

        A = 1+(a**2/b**2)
        B = (-2*xp)+(2*a*c/b**2)+(2*a*yp/b)
        C = (xp**2)+(c**2/b**2)+(2*c*yp/b)+(yp**2)-(r**2)

        discriminant = (B**2)-(4*A*C)

        if discriminant >= 0:
            cross_x_1 = (-B+sqrt(discriminant))/(2*A)
            cross_y_1 = slope*(cross_x_1-vector_point_x[0])+vector_point_y[0]


            if OS_pos_x <= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (xp-r) <= cross_x_1 <= (xp+r) and (yp-r) <= cross_y_1 <= (yp+r):
                    if OS_pos_x <= cross_x_1 <= after_delta_t_x and OS_pos_y <= cross_y_1 <= after_delta_t_y:
                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x >= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (xp-r) <= cross_x_1 <= (xp+r) and (yp-r) <= cross_y_1 <= (yp+r):
                    if after_delta_t_x <= cross_x_1 <= OS_pos_x and OS_pos_y <= cross_y_1 <= after_delta_t_y:
                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x <= after_delta_t_x and OS_pos_y >= after_delta_t_y:
                if (xp-r) <= cross_x_1 <= (xp+r) and (yp-r) <= cross_y_1 <= (yp+r):
                    if OS_pos_x <= cross_x_1 <= after_delta_t_x and  after_delta_t_y <= cross_y_1 <= OS_pos_y:
                        return False
                    else:
                        return True
                else:
                    return True

            else:
                if (xp-r) <= cross_x_1 <= (xp+r) and (yp-r) <= cross_y_1 <= (yp+r):
                    if after_delta_t_x <= cross_x_1 <= OS_pos_x and after_delta_t_y <= cross_y_1 <= OS_pos_y:
                        return False
                    else:
                        return True
                else:
                    return True

        else:
            return True


    def __select_vel_inside_RVOs(self, reachableCollisionVel_global_all, RVOdata_all, V_des):
        # Compute the minimum time to collision(tc) for every velocity candidates
        velCandidates_dict = dict()

        for reachableCollisionVel_global in reachableCollisionVel_global_all:
            
            velCandidates_dict[tuple(reachableCollisionVel_global)] = dict()
            tc_min = 99999.9 # initialize the time to collision to a large enough

            # Compute the minimum time to collision(tc) for a velocity candidate
            for RVOdata in RVOdata_all:

                vA2B_RVO = reachableCollisionVel_global - RVOdata['collisionConeTranslated']

                angle_vA2B_RVO_rad_global = atan2(
                    vA2B_RVO[1],
                    vA2B_RVO[0],
                    )

                # Consider only collidable agent.
                # NOTE: tc will maintain a large initial value if no collision 
                #       for the corresponding agent.
                if self.__is_in_between(
                RVOdata['boundLineAngle_right_rad_global'],
                angle_vA2B_RVO_rad_global, 
                RVOdata['boundLineAngle_left_rad_global'],
                ):

                    angle_at_pA = abs(angle_vA2B_RVO_rad_global - 0.5 * (RVOdata['boundLineAngle_right_rad_global'] + RVOdata['boundLineAngle_left_rad_global']))   
                    angle_at_pA %= (2*pi)
                    if angle_at_pA > pi:
                        angle_at_pA = abs(angle_at_pA - (2*pi))                

                    if (abs(RVOdata['LOSdist'] * sin(angle_at_pA)) > RVOdata['mapped_radius']):
                        angle_at_collisionPoint = 0.0    
                    else:
                        angle_at_collisionPoint = asin(
                            abs(RVOdata['LOSdist'] * sin(angle_at_pA)) / RVOdata['mapped_radius']
                            )

                    dist_collision = abs(RVOdata['LOSdist'] * cos(angle_at_pA)) - abs(RVOdata['mapped_radius'] * cos(angle_at_collisionPoint))

                    if dist_collision < 0: dist_collision = 0
                    tc = dist_collision / np.linalg.norm([vA2B_RVO])
                    
                    # NOTE: Avoid zero division for the penalty calculation
                    if tc < 0.00001: tc = 0.00001

                    if tc < tc_min: tc_min = tc

            # Store the minimum time to collision for each velocity candidate
            velCandidates_dict[tuple(reachableCollisionVel_global)]['tc_min'] = tc_min

            # Comput and store the penalty for each velocity candidate
            velCandidates_dict[tuple(reachableCollisionVel_global)]['penalty'] = self.weight_aggresiveness / tc_min + np.linalg.norm([V_des - reachableCollisionVel_global])
        # Take the velocity that has the minimum penalty
        vA_post = min(velCandidates_dict, key=lambda k : velCandidates_dict[k]['penalty'])
        return vA_post

    def __choose_velocity(self, V_des, RVOdata_all, OS, TS, static_obstacle_info, static_point_info): 
        # Generate target speed candidates
        # NOTE: We generate the target velocity candidates manually, not deriving from mmg and feasible acc.
        targetSpeed_all = np.linspace(
            start=self.min_targetSpeed, 
            stop=self.max_targetSpeed, 
            num=self.num_targetSpeedCandidates,
            )

        TS_ID = TS.keys()
        for ts_ID in TS_ID:
            status = TS[ts_ID]['status']

        # Generate target heading angle candidates
        min_targetHeading_rad_local = np.deg2rad(self.min_targetHeading_deg_local)
        max_targetHeading_rad_local = np.deg2rad(self.max_targetHeading_deg_local)
        heading_rad = np.deg2rad(OS['Heading'])

        min_targetHeading_rad_global = heading_rad + min_targetHeading_rad_local
        max_targetHeading_rad_global = heading_rad + max_targetHeading_rad_local

        targetHeading_rad_global_all = np.linspace(
            start=min_targetHeading_rad_global, 
            stop=max_targetHeading_rad_global, 
            num=self.num_targetHeadingCandidates,
            )

        # Generate target velocity vector candidates
        reachableVel_global_all = self.__generate_vel_candidates(
            targetSpeed_all, 
            targetHeading_rad_global_all,
            OS,
            static_obstacle_info,
            static_point_info
            )
        
        # Annotate the velocities - 'in time horizon', 'in left', 'in right', 'in collision cone'
        reachableVel_all_annotated = self.__annotate_vels(
            reachableVel_global_all, 
            RVOdata_all, 
            TS,
            )

    

        isAllVelsCollidable = self.__is_all_vels_collidable(
            vel_all_annotated=reachableVel_all_annotated, 
            shipID_all=TS.keys(),
            )

        isAllVelsAvoidable = self.__is_all_vels_avoidable(
            vel_all_annotated=reachableVel_all_annotated, 
            shipID_all=TS.keys(),
            )

        # When no avoidance velocities
        if isAllVelsCollidable:
            velCandidates = self.__remove_annotation(reachableVel_all_annotated)
            # NOTE: `_select_vel_inside_RVOs()` returns a velocity where 
            #       all the reachable velocities are inside the collision cones
            vA_post = self.__select_vel_inside_RVOs(
                velCandidates, 
                RVOdata_all, 
                V_des,
                )

        # When no collision velocities
        elif isAllVelsAvoidable:
            velCandidates = self.__remove_annotation(reachableVel_all_annotated)
            vA_post = min(
                velCandidates,
                key= lambda v: np.linalg.norm(v - V_des),
                )
            

        # When partially have avoidance velocities
        else:
            avoidanceVel_all_annotated = self.__take_vels(
                vel_all_annotated=reachableVel_all_annotated,
                annotation=['inLeft', 'inRight', 'inTimeHorizon'],
                shipID_all=TS.keys(),
                )

            avoidanceAllRightVel_all_annotated = self.__take_vels(  
                vel_all_annotated=reachableVel_all_annotated,       
                annotation=['inLeft'],                              
                shipID_all=TS.keys(),
                )


            if avoidanceAllRightVel_all_annotated:
                velCandidates = self.__remove_annotation(avoidanceAllRightVel_all_annotated)
            else:
                velCandidates = self.__remove_annotation(avoidanceVel_all_annotated)
            vA_post = min(
                velCandidates,
                key= lambda v: np.linalg.norm(v - V_des),
                )


        return vA_post 

    def __extract_RVO_data(self, OS, TS):

        vA = np.array([OS['V_x'], OS['V_y']])
        pA = np.array([OS['Pos_X'], OS['Pos_Y']])

        RVOdata_all = []
        pub_collision_cone = []
        TS_ID = TS.keys()

        for ts_ID in TS_ID:

            vB = np.array([TS[ts_ID]['V_x'], TS[ts_ID]['V_y']])
            pB = np.array([TS[ts_ID]['Pos_X'], TS[ts_ID]['Pos_Y']])

            CRI = TS[ts_ID]['CRI']
            status = TS[ts_ID]['status']

            RVOapexPos_global = pA + (1 - self.weight_alpha) * vA + self.weight_alpha * vB
            LOSdist = np.linalg.norm([pA - pB]) 
            LOSangle_rad = atan2(pB[1] - pA[1], pB[0] - pA[0])
            
            boundLineAngle_left_rad_global = LOSangle_rad + atan2(TS[ts_ID]['mapped_radius'],LOSdist)
            boundLineAngle_right_rad_global = LOSangle_rad - atan2(TS[ts_ID]['mapped_radius'],LOSdist)
            
            collisionConeTranslated = (1 - self.weight_alpha) * vA + self.weight_alpha * vB
            
            RVOdata = {
                "TS_ID": ts_ID,
                "LOSdist": LOSdist,
                "mapped_radius": TS[ts_ID]['mapped_radius'],
                "vA": vA,
                "vB": vB,
                "boundLineAngle_left_rad_global" : boundLineAngle_left_rad_global,
                "boundLineAngle_right_rad_global" : boundLineAngle_right_rad_global,
                "collisionConeTranslated": collisionConeTranslated,
                "CRI": CRI,
                }
            RVOdata_all.append(RVOdata)
            bound_left_view = [
                cos(boundLineAngle_left_rad_global)* int(LOSdist)/2,
                sin(boundLineAngle_left_rad_global)* int(LOSdist)/2,
                ]
            bound_right_view = [
                cos(boundLineAngle_right_rad_global)* int(LOSdist)/2,
                sin(boundLineAngle_right_rad_global)* int(LOSdist)/2,
                ]

            pub_collision_cone.append(RVOapexPos_global[0])
            pub_collision_cone.append(RVOapexPos_global[1])
            pub_collision_cone.append(bound_left_view[0] + RVOapexPos_global[0])
            pub_collision_cone.append(bound_left_view[1] + RVOapexPos_global[1])
            pub_collision_cone.append(bound_right_view[0] + RVOapexPos_global[0])
            pub_collision_cone.append(bound_right_view[1] + RVOapexPos_global[1])
            

        return RVOdata_all, pub_collision_cone

    def VO_update(self, OS_original, TS_original, V_des, static_obstacle_info, static_point_info):   

        RVOdata_all, pub_collision_cone = self.__extract_RVO_data(
            OS_original,
            TS_original,
            )

        V_opt = self.__choose_velocity(V_des, RVOdata_all, OS_original, TS_original,static_obstacle_info, static_point_info)

        return V_opt, pub_collision_cone

    def vectorV_to_goal(self, OS, goal, V_max):
        Pos_x = OS['Pos_X']
        Pos_y = OS['Pos_Y']
        dif_x = [goal[0] - Pos_x, goal[1] - Pos_y]
        norm = np.linalg.norm(dif_x)
        V_des = [dif_x[k] * V_max / norm for k in range(2)]

        return V_des


class kass_inha:
    def __init__(self, Update_parameter):
        self.available_info = dict()
        self.unavailable_info = dict()
        self.latitude = 0.0 
        self.longitude = 0.0
        self.idOfObject = []
        self.latOfObject = []
        self.longOfObject = []
        self.cog = 0.0
        self.cogOfObject = []
        self.sog = 0.0
        self.sogOfObject = []
        self.latOfWayPoint = []
        self.longOfWayPoint = []
        self.waypoint_idx = 0
        self.standard_speed = Update_parameter['standard_speed']
        
        self.ship_ID = []
        self.ship_L = Update_parameter['ship_L']
        self.ship_B = Update_parameter['ship_B']
        self.ship_scale = Update_parameter['ship_scale']
        self.target_speed = Update_parameter['target_speed']
        self.waypoint_idx = 0
        self.len_waypoint_info = 0
        
        self.ts_spd_dict = dict()
        self.TS_WP_index = []
        self.target_heading_list = []
        
        self.static_obstacle_info = []
        self.static_point_info = []
        self.waypoint_info = dict()
        
        self.error = False
        self.errorCode = None
        self.pub_list = []
        self.parameter = Update_parameter
        self.eOfobject = []
        self.nOfobject = []
        self.eOfwaypoint = []
        self.nOfwaypoint = []

        self.encounter = None
        self.encounterMMSI = []

    def latlong_to_utm(self, latitude, longitude, northern_hemisphere=True):
        utm_zone = pyproj.Proj(proj='utm', zone=52, datum='WGS84')
        utm_east,utm_north  = utm_zone(longitude, latitude)

        return utm_north, utm_east
    
    def utm_to_latlong(self, utm_north, utm_east, northern_hemisphere=True):
        utm_zone = pyproj.Proj(proj='utm', zone=52, datum='WGS84', ellps='WGS84')
        longitude,latitude  = utm_zone(utm_east, utm_north, inverse=True)

        return latitude, longitude


    def enu_convert(self,gnss,origin):
        e, n, u = pm.geodetic2enu(gnss[0], gnss[1], gnss[2], origin[0], origin[1], origin[2])
        return n, e, u
    
    def gnss_convert(self,enu,origin):
        lat,long,alt = pm.enu2geodetic(enu[0], enu[1], enu[2], origin[0], origin[1], origin[2])
        return lat, long, alt

    def setParamUpdate(self, Update_Parameter):
        self.parameter = Update_Parameter
        

    def input_check(self, inha_input):
        error = False
        errorCode = None
        if len(inha_input) < 11:
            print("------insufficient number of input------")
            errorCode = 300
            error = True
        else:
            pass
        return errorCode,error
    
    def OS_check(self, inha_input):
        error = False
        errorCode = None
        latitude = inha_input["latitude"]
        longitude = inha_input["longitude"]
        if len([latitude]) != len([longitude]):
            print("------OS position data is missing------")
            errorCode = 301
            error = True
        else:
            pass
        return errorCode,error
    
    def TS_check(self, inha_input):
        error = False
        errorCode = None
        idOfObject = inha_input["idOfObject"]
        latOfObject = inha_input["latOfObject"]
        longOfObject = inha_input["longOfObject"]
        if len(idOfObject) != len(latOfObject):
            print("------TS position data is missing------")
            error = True
            errorCode = 301
        elif len(idOfObject) != len(longOfObject):
            print("------TS position data is missing------")
            error = True
            errorCode = 301
        else:
            pass
        return errorCode,error
    
    def wp_check(self, inha_input):
        error = False
        errorCode = None
        latOfWayPoint = inha_input["latOfWayPoint"]
        longOfWayPoint = inha_input["longOfWayPoint"]
        if len(latOfWayPoint) != len(longOfWayPoint):
            print("------Waypoints are missing------")
            error = True
        else:
            pass
        return errorCode,error
    
    def error_check(self, inha_input):
        error = False
        errorCode = None
        errorCode,error = self.input_check(inha_input)
        if errorCode == None:
            errorCode,error = self.OS_check(inha_input)
            if errorCode == None:
                errorCode,error = self.TS_check(inha_input)
                if errorCode == None:
                    errorCode,error = self.wp_check(inha_input)
                else: 
                    return errorCode,error
            else:
                return errorCode,error
        else:
            return errorCode,error
        return errorCode, error

    def wp_callback(self, latOfWayPoint, longOfWayPoint):
        self.waypoint_info['waypoint_x'] = latOfWayPoint
        self.waypoint_info['waypoint_y'] = longOfWayPoint

        return self.waypoint_info


    def op_callback(self):
        self.Pos_X = self.latitude
        self.Pos_Y = self.longitude
        self.Vel_U = self.sog
        self.ship_ID = self.idOfObject
        raw_psi = np.asanyarray(self.cog)
        self.Heading = raw_psi % 360

        return self.Pos_X, self.Pos_Y, self.Vel_U, self.Heading       
    

    def path_out_publish(self, pub_list):
        path_out_inha = dict()
        path_out_inha['modifyWayPoint'] = pub_list[0]
        path_out_inha['numOfWayPoint']  = pub_list[1]
        path_out_inha['latOfWayPoint'] = pub_list[2]
        path_out_inha['longOfWayPoint'] = pub_list[3]
        path_out_inha['speedOfWayPoint'] = pub_list[4]
        path_out_inha['ETAOfWayPoint'] = pub_list[5]
        path_out_inha['EDAOfWayPoint'] = pub_list[6]
        path_out_inha['error'] = pub_list[7]
        path_out_inha['errorCode'] = pub_list[8]
        path_out_inha['targetSpeed'] = round(pub_list[9], 3)
        path_out_inha['targetCourse'] = round(pub_list[10], 3)
        path_out_inha['cri'] = pub_list[11]
        path_out_inha['V_selected'] = pub_list[12]

        path_out_inha['encounter'] = pub_list[13]
        path_out_inha['encounterMMSI'] = pub_list[14]
        
        return path_out_inha
    
    def eta_eda_assumption(self, WP, OS, target_speed):
        ''' 목적지까지 도달 예상 시간 및 거리
        
        Return:
            eta [t], eda [m]
        '''       
        eta = []
        eda = []
        for i in range(len(self.latOfWayPoint)):
            eda_x = WP[0][i] - OS['Pos_X']
            eda_y = WP[1][i] - OS['Pos_Y']
            distance = sqrt(eda_x**2 + eda_y**2)
            eda.append(round(distance, 3))
            time = distance / target_speed
            eta.append(round(time, 3))
        
        return eta, eda


    def kass_inha(self, inha_input):
        self.errorCode, self.error = self.error_check(inha_input)

        if self.errorCode != None:
            print(f"------error {self.errorCode}------")
            return self.pub_list
        else:
            self.latitude = inha_input["latitude"]
            self.longitude = inha_input["longitude"]
            self.idOfObject = inha_input["idOfObject"]
            self.latOfObject = inha_input["latOfObject"]
            self.longOfObject = inha_input["longOfObject"]
            self.sog = inha_input["sog"]
            self.cog = inha_input["cog"]
            self.degreeOfAxis = inha_input['degreeOfAxis']

            if self.sog >= self.standard_speed:
                self.heading = self.cog
            else:
                self.heading = self.degreeOfAxis[2]
            
            self.cogOfObject = inha_input["cogOfObject"]
            self.sogOfObject = inha_input["sogOfObject"]
            self.latOfWayPoint = inha_input["latOfWayPoint"]
            self.longOfWayPoint = inha_input["longOfWayPoint"]
            self.waypoint_idx = inha_input['nWptsID'] % len(self.latOfWayPoint)

            self.latitude,self.longitude = self.latlong_to_utm(self.latitude,self.longitude)
            self.eOfobject = []
            self.nOfobject = []
            self.eOfwaypoint = []
            self.nOfwaypoint = []

            for i in range(len(self.latOfObject)):
                n, e = self.latlong_to_utm(self.latOfObject[i],self.longOfObject[i])
                self.eOfobject.append(e)
                self.nOfobject.append(n)

            for j in range(len(self.latOfWayPoint)):
                n, e = self.latlong_to_utm(self.latOfWayPoint[j],self.longOfWayPoint[j])
                self.eOfwaypoint.append(e)
                self.nOfwaypoint.append(n)

            self.latOfObject = self.nOfobject
            self.longOfObject = self.eOfobject
            self.latOfWayPoint = self.nOfwaypoint
            self.longOfWayPoint = self.eOfwaypoint
            

            Local_PP = VO_module(self.parameter)

            t = 0
            waypointIndex = self.waypoint_idx
            target_speed = self.target_speed * 0.5144
            ship_L = self.ship_L

            waypoint_info = self.wp_callback(self.latOfWayPoint, self.longOfWayPoint)
            self.op_callback()

            self.idOfObject_copy =[]
            self.latOfObject_copy = []
            self.longOfObject_copy = []
            self.cogOfObject_copy = []
            self.sogOfObject_copy = []

            for i in range(len(self.idOfObject)):
                distance = sqrt((self.latitude-self.latOfObject[i])**2+(self.longitude-self.longOfObject[i])**2)
                if distance <= self.parameter['detecting_distance']:
                    self.idOfObject_copy.append(self.idOfObject[i])
                    self.latOfObject_copy.append(self.latOfObject[i])
                    self.longOfObject_copy.append(self.longOfObject[i])
                    self.cogOfObject_copy.append(self.cogOfObject[i])
                    self.sogOfObject_copy.append(self.sogOfObject[i])

                    self.encounter = True
                    self.encounterMMSI.append(self.idOfObject[i])

            if len(self.idOfObject_copy) == 0:
                self.encounter = False
                self.encounterMMSI = []

            self.idOfObject = self.idOfObject_copy
            self.latOfObject = self.latOfObject_copy
            self.longOfObject = self.longOfObject_copy
            self.cogOfObject = self.cogOfObject_copy
            self.sogOfObject = self.sogOfObject_copy
            

            if len(self.idOfObject) != 0:
                inha = Inha_dataProcess(self.idOfObject,
                                        self.latitude,
                                        self.longitude,
                                        self.heading,
                                        self.sog,
                                        self.latOfObject,
                                        self.longOfObject,
                                        self.cogOfObject,
                                        self.sogOfObject,
                                        self.parameter
                                        )
                
                wpts_x_os = list(waypoint_info['waypoint_x'])
                wpts_y_os = list(waypoint_info['waypoint_y'])
                Local_goal = [wpts_x_os[waypointIndex], wpts_y_os[waypointIndex]]

                OS_list = inha.os_info()
                TS_list = inha.ts_info()
                OS_Vx, OS_Vy = inha.U_to_vector_V(OS_list['Vel_U'], OS_list['Heading'])

                OS_list['V_x'] = OS_Vx
                OS_list['V_y'] = OS_Vy

                local_goal_EDA = sqrt((Local_goal[0]-OS_list['Pos_X'])**2 + (Local_goal[1]-OS_list['Pos_Y'])**2)
                V_des = Local_PP.vectorV_to_goal(OS_list, Local_goal, target_speed)


                TS_list = inha.TS_info_supplement(
                        OS_list, 
                        TS_list,
                        )
                    
                TS_DCPA_temp = []
                TS_TCPA_temp = []
                TS_UDCPA_temp = []
                TS_UTCPA_temp = []
                TS_UD_temp = []
                TS_UB_temp = []
                TS_UK_temp = []
                TS_CRI_temp = []
                TS_Rf_temp = []
                TS_Ra_temp = []
                TS_Rs_temp = []
                TS_Rp_temp = []
                TS_ENC_temp = []

                for ts_ID in self.idOfObject:

                    temp_DCPA = TS_list[ts_ID]['DCPA']
                    TS_DCPA_temp.append(temp_DCPA)

                    temp_TCPA = TS_list[ts_ID]['TCPA']
                    TS_TCPA_temp.append(temp_TCPA)

                    temp_UDCPA = TS_list[ts_ID]['UDCPA']
                    TS_UDCPA_temp.append(temp_UDCPA)
                    
                    temp_UTCPA = TS_list[ts_ID]['UTCPA']
                    TS_UTCPA_temp.append(temp_UTCPA)

                    temp_UD = TS_list[ts_ID]['UD']
                    TS_UD_temp.append(temp_UD)

                    temp_UB = TS_list[ts_ID]['UB']
                    TS_UB_temp.append(temp_UB)

                    temp_UK = TS_list[ts_ID]['UK']
                    TS_UK_temp.append(temp_UK)

                    temp_cri = TS_list[ts_ID]['CRI']
                    TS_CRI_temp.append(temp_cri)

                    temp_Rf = TS_list[ts_ID]['Rf']
                    TS_Rf_temp.append(temp_Rf)

                    temp_Ra = TS_list[ts_ID]['Ra']
                    TS_Ra_temp.append(temp_Ra)

                    temp_Rs = TS_list[ts_ID]['Rs']
                    TS_Rs_temp.append(temp_Rs)

                    temp_Rp = TS_list[ts_ID]['Rp']
                    TS_Rp_temp.append(temp_Rp)

                    temp_enc = TS_list[ts_ID]['status']


                    TS_ENC_temp.append(temp_enc)

                               

                V_selected, pub_collision_cone = Local_PP.VO_update(OS_list,
                                                                    TS_list,
                                                                    V_des,
                                                                    self.static_obstacle_info,
                                                                    self.static_point_info
                                                                    )


                # if not isinstance(V_selected, list):
                #     V_selected = V_selected.tolist()


            else:
                inha = Inha_dataProcess(self.idOfObject,
                        self.latitude,
                        self.longitude,
                        self.heading,
                        self.sog,
                        self.latOfObject,
                        self.longOfObject,
                        self.cogOfObject,
                        self.sogOfObject,
                        self.parameter
                        )
                wpts_x_os = list(waypoint_info['waypoint_x'])
                wpts_y_os = list(waypoint_info['waypoint_y'])
                Local_goal = [wpts_x_os[waypointIndex], wpts_y_os[waypointIndex]]
                OS_list = {
                    'shipID' : int,
                    'Pos_X' : self.latitude,
                    'Pos_Y' : self.longitude,
                    'Vel_U' : self.sog,
                    'Heading' : self.heading,
                    }
                V_selected = Local_PP.vectorV_to_goal(OS_list, Local_goal, target_speed)

                TS_CRI_temp = []
                
            if 'tolist' in dir(V_selected):
                V_selected = V_selected.tolist()


            wp = inha.waypoint_generator(OS_list, V_selected)
            wp_eta_eda = (self.latOfWayPoint, self.longOfWayPoint)
            wp_x = wp[0]
            wp_y = wp[1]
            wp_x_gnss = []
            wp_y_gnss = []
            speedOfWayPoint = []


            for i in range(15):
                speedOfWayPoint.append(self.sog)  
                

            for i in range(len(wp_x)):
                wp_in_gnss = self.utm_to_latlong(wp_x[i], wp_y[i])
                wp_x_gnss.append(wp_in_gnss[0])
                wp_y_gnss.append(wp_in_gnss[1])
            
            eta, eda = self.eta_eda_assumption(wp_eta_eda, OS_list, target_speed)
            temp_spd, temp_heading_deg = inha.desired_value_assumption(V_selected)
            desired_spd_list = temp_spd
            desired_heading_list = temp_heading_deg
            desired_spd = desired_spd_list[0] / 0.5144
            desired_heading = desired_heading_list[0]

            if t%10 ==0:
                pass

            t += 1
                    
            if -180 <= desired_heading < 0:
                desired_heading = desired_heading + 360
            
            else:
                desired_heading = desired_heading


            OS_pub_list = [
                False,
                len(wp_x),
                wp_x_gnss, 
                wp_y_gnss,  
                speedOfWayPoint, 
                eta, 
                eda, 
                self.error, 
                self.errorCode,  
                round(desired_spd,3),
                round(desired_heading,3),
                TS_CRI_temp,
                V_selected,
                self.encounter,
                self.encounterMMSI
                ]
            
            path_out_inha = self.path_out_publish(OS_pub_list)


            return path_out_inha