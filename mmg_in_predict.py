#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from functions.ShipSimulation import ShipSimulation
from functions.InfoLoader import InfoLoader

from udp_msgs.msg import frm_info

import numpy as np
import rospy
from math import sqrt
from collections import deque
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
    
class UKF:
    def __init__(self, shipName):
        rospy.Subscriber('/AIS_data', frm_info, self.OP_callback)
        self.OS_pub = rospy.Publisher('/frm_info', frm_info, queue_size=10)

        self.dt = rospy.get_param('ukf_dt')  # 샘플링 시간
        self.mmg_dt = rospy.get_param("mmg_dt")
        self.window_size = rospy.get_param("window_size")
        self.last_measurement = None
        self.last_heading = None
        self.ukf_initialized = False

        self.Pos_X  = (0.00001, 0.00001, 0.00001)
        self.Pos_Y  = (0.00001, 0.00001, 0.00001)
        self.Vel_U  = (0.00001, 0.00001, 0.00001)
        self.delta_deg = (0.00001, 0.00001, 0.00001)
        self.Heading = (0.00001, 0.00001, 0.00001)
        # 이동평균 결과를 저장할 리스트
        self.averages = [0,0,0,0]

        self.ship_ID = [0,0,0]
        self.shipName = shipName

        # ship 이름에서 숫자를 추출
        ship_number = int(self.shipName[4:])  # 'ship' 뒤의 숫자를 가져옴
        index = ship_number - 1  # 인덱스로 사용하기 위해 1을 뺌

        # 인덱스를 사용하여 shipid에서 ID를 가져오기
        self.ship_id = self.ship_ID[index]

        self.shipState_all = dict()
        self.shipInstance_all = dict()
        self.shipState_after_predict = dict()
        self.predicted_values = []
        self.initialize_ukf()

    def OP_callback(self, operation):
        self.ship_ID = list(operation.m_nShipID)

        self.Pos_X  = operation.m_fltPos_X
        self.Pos_Y  = operation.m_fltPos_Y
        self.Vel_U  = operation.m_fltVel_U

        self.delta_deg = operation.m_fltRudderAngleFeedSTBD # deg.

        raw_psi = np.asanyarray(operation.m_fltHeading)
        self.Heading = raw_psi % 360

    def frm_info_publish(self, ship_ID, Pos_X, Pos_Y, vel, psi_deg, delta_deg, start_time):
        kriso = frm_info()
        kriso.header.stamp = rospy.Time.now() - start_time
        kriso.header.frame_id = "ship_info"

        kriso.m_nShipID  = ship_ID
        kriso.m_fltPos_X = Pos_X
        kriso.m_fltPos_Y = Pos_Y
        kriso.m_fltVel_U = vel
        kriso.m_fltHeading = psi_deg
        kriso.m_fltRudderAngleFeedSTBD = delta_deg

        self.OS_pub.publish(kriso)

    def initialize_ukf(self):
        '''Alpha (α):
        alpha는 시그마 포인트의 분포를 결정하는 스케일링 파라미터입니다. alpha는 0과 1 사이의 작은 양수로 설정되며, 일반적으로 매우 작은 값(예: 1e-3)을 사용합니다.
        alpha가 작을수록 시그마 포인트들은 평균 근처에 더 가깝게 위치하며, alpha가 커지면 시그마 포인트들은 멀리 퍼지게 됩니다. 이는 곧 필터의 추정치가 측정치에 대해 얼마나 신뢰를 두는지에 영향을 미칩니다.
        alpha를 너무 크게 설정하면 필터가 불안정해질 수 있으며, 너무 작게 설정하면 필터가 새로운 측정치에 너무 느리게 반응할 수 있습니다.

        Beta (β):
        beta는 사전 분포에 대한 지식을 통합하는 파라미터입니다. 특히, beta는 상태 변수의 분포가 가우시안 분포에서 벗어날 때 사용됩니다.
        beta가 2일 경우 이는 시스템의 초기 분포가 정확히 가우시안(정규 분포)임을 의미합니다. 다른 값은 비가우시안 분포를 가정할 때 사용됩니다.
        beta를 조정함으로써 필터의 성능을 비선형성이 강한 시스템에 맞출 수 있습니다. 일반적으로, beta는 0 또는 2 근처의 값을 사용하지만, 시스템의 특성에 따라 최적의 값을 찾기 위한 실험이 필요할 수 있습니다.
        
        Kappa (κ):
        kappa는 보통 0 또는 3-n (여기서 n은 상태 변수의 차원)으로 설정됩니다. kappa는 시그마 포인트 생성시 중심 포인트의 가중치를 조정합니다.
        kappa를 조정함으로써 필터의 안정성과 정확성을 향상시킬 수 있습니다. 특히, kappa를 사용하여 비선형 시스템의 특성을 더 잘 반영할 수 있습니다.'''
        sigma_points = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2., kappa=0)
        self.ukf = UnscentedKalmanFilter(dim_x=4, dim_z=4, dt=self.dt, fx=self.state_transition, hx=self.measurement_function, points=sigma_points)
        self.ukf.x = np.array([0., 0., 0., 0.])  # 초기 상태 추정치
        self.ukf.P = np.eye(4) * 10000.  # 초기 공분산 행렬
        # self.ukf.P *= 10  # 초기 공분산 행렬

        self.ukf.R = np.eye(4) * .5  # 측정 노이즈
        self.ukf.Q = np.eye(4) * .5  # 프로세스 노이즈

        # Get the initial states
        paramStr_shipID = "shipInfo_all/" + self.shipName + "_info/ship_ID"
        paramStr_shipScale = "shipInfo_all/" + self.shipName + "_info/ship_scale"
        paramStr_initStartX = "shipInfo_all/" + self.shipName + "_info/initial_start_x"
        paramStr_initStartY = "shipInfo_all/" + self.shipName + "_info/initial_start_y"
        paramStr_initStartU = "shipInfo_all/" + self.shipName + "_info/initial_start_U"
        paramStr_initStartPsi = "shipInfo_all/" + self.shipName + "_info/initial_start_psi"
        paramStr_LBP = "shipInfo_all/" + self.shipName  + "_info/ship_L"

        self.shipState_all[self.shipName] = dict()
        self.shipState_all[self.shipName]["shipID"] = rospy.get_param(paramStr_shipID)
        self.shipState_all[self.shipName]["scale"] = rospy.get_param(paramStr_shipScale)
        self.shipState_all[self.shipName]["X"] = rospy.get_param(paramStr_initStartX)
        self.shipState_all[self.shipName]["Y"] = rospy.get_param(paramStr_initStartY)
        self.shipState_all[self.shipName]["U"] = rospy.get_param(paramStr_initStartU) * 0.5144 / sqrt(self.shipState_all[self.shipName]["scale"])
        self.shipState_all[self.shipName]["u"] = self.shipState_all[self.shipName]["U"]
        self.shipState_all[self.shipName]["v"] = 0
        self.shipState_all[self.shipName]["psi_deg"] = rospy.get_param(paramStr_initStartPsi)
        self.shipState_all[self.shipName]["delta_deg"] = 0.0
        self.shipState_all[self.shipName]["LBP"] = rospy.get_param(paramStr_LBP)

        # Get the ships instances
        self.shipInstance_all[self.shipName] = ShipSimulation(
            self.shipState_all[self.shipName]["X"],
            self.shipState_all[self.shipName]["Y"],
            self.shipState_all[self.shipName]["U"],
            self.shipState_all[self.shipName]["u"],
            self.shipState_all[self.shipName]["v"],
            self.shipState_all[self.shipName]["psi_deg"],
            self.shipState_all[self.shipName]["delta_deg"],
            self.shipState_all[self.shipName]["LBP"],
            self.shipState_all[self.shipName]["scale"],
            self.mmg_dt,
            )

    def state_transition(self, x, dt):
        self.shipState_after_predict[self.shipName] = {**{'shipID': self.ship_id} , **self.shipInstance_all[self.shipName].moving_ships(x[3], abs(x[2]))}

        x_new = np.zeros_like(x)
        x_new[0] = self.shipState_after_predict[self.shipName]['X']
        x_new[1] = self.shipState_after_predict[self.shipName]['Y']
        x_new[2] = self.shipState_after_predict[self.shipName]['U']  # 속도는 변하지 않는다고 가정
        # x_new[3] = a  # 방향은 변하지 않는다고 가정
        x_new[3] = self.shipState_after_predict[self.shipName]['psi_deg']  # 방향은 변하지 않는다고 가정psi_deg

        return x_new

    def measurement_function(self, x):
        return x

    def update_ukf(self, lat, long, speed, heading):

        measurement = [lat, long, speed, heading]

        if not self.ukf_initialized:
            self.ukf.x = measurement  # 초기 상태 추정치 설정
            self.ukf_initialized = True  # UKF가 초기화되었음을 표시
            rospy.loginfo("UKF initialized with first measurement: %s", self.ukf.x)
            
        # 측정값이 변경되었는지 확인
        if self.last_measurement is not None and np.array_equal(measurement, self.last_measurement):

            self.last_heading = self.ukf.x[3]

            # 측정값이 변경되지 않았다면 predict만 수행
            self.ukf.predict()
            self.ukf.update(measurement)

            self.predicted_values.append(self.ukf.x[0])
            self.predicted_values.append(self.ukf.x[1])
            self.predicted_values.append(self.ukf.x[2])
            self.predicted_values.append(self.ukf.x[3])

        else:
            # 측정값이 변경되었다면 예측 및 업데이트 수행
            if self.last_measurement is not None:
                # 각도 변화량 계산 및 업데이트 로직 적용
                self.last_heading = self.last_measurement[3]  # 이전 헤딩 값을 업데이트
            self.last_measurement = measurement

            self.ukf.predict()
            self.ukf.update(measurement)

            # self.predicted_values = []
        # print(self.shipName ,self.predicted_values)
        kalman_gain = self.ukf.K
        # rospy.loginfo("Kalman Gain: %s", kalman_gain)
        # rospy.loginfo("Covariance Matrix: %s", self.ukf.P)

        x_window = deque(maxlen=self.window_size)
        y_window = deque(maxlen=self.window_size)
        speed_window = deque(maxlen=self.window_size)
        heading_window = deque(maxlen=self.window_size)

        for i in range(0, len(self.predicted_values), 4):
            x, y, speed, heading = self.predicted_values[i:i+4]
            
            x_window.append(x)
            y_window.append(y)
            speed_window.append(speed)
            heading_window.append(heading)
            
            current_x_avg = sum(x_window) / len(x_window)
            current_y_avg = sum(y_window) / len(y_window)
            current_speed_avg = sum(speed_window) / len(speed_window)
            current_heading_avg = sum(heading_window) / len(heading_window)
            
            self.averages[0] = current_x_avg
            self.averages[1] = current_y_avg
            self.averages[2] = current_speed_avg
            self.averages[3] = current_heading_avg
            # print(self.ukf.x)
        # return self.averages
        return self.ukf.x
    
def main():
    rospy.init_node('ship_predict', anonymous=False)
    update_rate = rospy.get_param("update_rate")
    rate = rospy.Rate(update_rate) # 10 Hz renew
    
    # ukf = UKF("ship1")
    uncertain_ships = {}

    shipsInfo = InfoLoader(rospy.get_param("shipInfo_all"))

    first_loop = True  # 첫 번째 루프 실행 여부를 추적하는 변수
    predicted_state = []

    start_time = rospy.Time.now()

    while not rospy.is_shutdown():

        shipID_all = []
        Pos_X_all = []
        Pos_Y_all = []
        Vel_U_all = []
        Heading_deg_all = []
        delta_deg_all = []

        if first_loop:
            for shipName in shipsInfo.shipName_all:
                # shipID = ukf.shipState_all[shipName]['shipID']
                # 각 선박 ID에 대해 독립적인 UKF 인스턴스 생성 및 저장
                uncertain_ships[shipName] = UKF(shipName)

            first_loop = False  # 첫 번째 루프가 실행된 후에는 이 조건을 더 이상 만족시키지 않음

        if len(uncertain_ships[shipName].Pos_X) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/AIS_data` topic =========")
                rate.sleep()
                continue
    
        # print(uncertain_ships['ship3'].Pos_X[1],
        #                                                             uncertain_ships['ship2'].Pos_Y[2],
        #                                                             uncertain_ships['ship2'].Vel_U[2],
        #                                                             uncertain_ships['ship2'].Heading[2])
        count = 0
        for shipName in shipsInfo.shipName_all:
            shipID = uncertain_ships[shipName].shipState_all[shipName]['shipID']

            if shipName == 'ship1':
                shipID_all.append(1000)
                Pos_X_all.append(uncertain_ships[shipName].Pos_X[count])
                Pos_Y_all.append(uncertain_ships[shipName].Pos_Y[count])
                Vel_U_all.append(uncertain_ships[shipName].Vel_U[count])
                Heading_deg_all.append(uncertain_ships[shipName].Heading[count])
                delta_deg_all.append(0.0)

            else:
                predicted_state = uncertain_ships[shipName].update_ukf(uncertain_ships[shipName].Pos_X[count],
                                                                    uncertain_ships[shipName].Pos_Y[count],
                                                                    uncertain_ships[shipName].Vel_U[count],
                                                                    uncertain_ships[shipName].Heading[count])
                shipID_all.append(shipID)
                Pos_X_all.append(predicted_state[0])
                Pos_Y_all.append(predicted_state[1])
                Vel_U_all.append(predicted_state[2])
                Heading_deg_all.append(predicted_state[3])
                delta_deg_all.append(0.0)
            # print(shipID)

            count += 1

        uncertain_ships[shipName].frm_info_publish(
                shipID_all, 
                Pos_X_all, 
                Pos_Y_all, 
                Vel_U_all, 
                Heading_deg_all, 
                delta_deg_all, 
                start_time, 
            )
        print(uncertain_ships['ship3'].ukf.x)
        rate.sleep()
        
    rospy.spin()

if __name__ == '__main__':
    main()
