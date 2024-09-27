import numpy as np

class UKF:
    def __init__(self):
        self.Q = np.diag([0.01, 0.01, 0.01, 0.01])  # 시스템 노이즈 공분산
        self.R = np.diag([0.1, 0.1, 0.01, 0.1])  # 관측 노이즈 공분산
        self.x = np.array([0, 0, 0, 0])  # x, y, heading, speed 초기 값
        self.P = 100 * np.eye(4)  # 초기 공분산
        self.n = 4 # 상태 벡터의 크기
        self.m = 4  # 관측 벡터의 크기

    # Sigma Points 함수
    def sigma_points(self, xm, P, kappa):
        n = len(xm)
        Xi = np.zeros((n, 2 * n + 1))
        W = np.zeros(2 * n + 1)

        Xi[:, 0] = xm
        W[0] = kappa / (n + kappa)

        U = np.linalg.cholesky((n + kappa) * P)

        for k in range(n):
            Xi[:, k + 1] = xm + U[:, k]
            Xi[:, n + k + 1] = xm - U[:, k]
            W[k + 1] = 1 / (2 * (n + kappa))
            W[n + k + 1] = 1 / (2 * (n + kappa))

        return Xi, W

    # Unscented Transform 함수
    def unscented_transform(self, Xi, W, noise_cov):
        n, kmax = Xi.shape

        xm = np.zeros(n)
        for k in range(kmax):
            xm += W[k] * Xi[:, k]

        xcov = np.zeros((n, n))
        for k in range(kmax):
            diff = (Xi[:, k] - xm).reshape(n, 1)
            xcov += W[k] * np.dot(diff, diff.T)

        xcov += noise_cov

        return xm, xcov

    # 상태 변환 함수
    def fx(self, x, dt):
        speed = x[2]
        heading = x[3]
        
        x_new = x[0] + speed * dt * np.cos(np.deg2rad(heading))
        y_new = x[1] + speed * dt * np.sin(np.deg2rad(heading))

        # 헤딩 값을 0~360도로 정상화
        heading_new = (heading + 360) % 360

        return np.array([x_new, y_new, speed, heading_new])

    # 관측 변환 함수
    def hx(self, x):
        return np.array([x[0], x[1], x[2], x[3]])
    
    # 예측만 수행하는 함수
    def predict(self, dt):
        kappa = 0  # 시그마 포인트 스케일링 매개변수

        # 시그마 포인트 계산
        Xi, W = self.sigma_points(self.x, self.P, kappa)

        # 시그마 포인트들에 상태 변환 적용
        fXi = np.zeros((self.n, 2 * self.n + 1))
        for k in range(2 * self.n + 1):
            fXi[:, k] = self.fx(Xi[:, k], dt)

        # 상태 예측 및 공분산 계산
        xp, Pp = self.unscented_transform(fXi, W, self.Q)

        # 상태와 공분산 갱신 (예측만 수행)
        self.x = xp
        self.P = Pp

        # 예측된 상태 반환
        return self.x, self.P
    
    # UKF 상태 업데이트 함수
    def update(self, z, dt):
        kappa = 0  # 시그마 포인트 스케일링 매개변수

        # 시그마 포인트 계산
        Xi, W = self.sigma_points(self.x, self.P, kappa)

        # 시그마 포인트들에 상태 변환 적용
        fXi = np.zeros((self.n, 2 * self.n + 1))
        for k in range(2 * self.n + 1):
            fXi[:, k] = self.fx(Xi[:, k], dt)

        # 상태 예측 및 공분산 계산
        xp, Pp = self.unscented_transform(fXi, W, self.Q)

        # 시그마 포인트들에 관측 변환 적용
        hXi = np.zeros((self.m, 2 * self.n + 1))
        for k in range(2 * self.n + 1):
            hXi[:, k] = self.hx(fXi[:, k])

        # 관측 예측 및 공분산 계산
        zp, Pz = self.unscented_transform(hXi, W, self.R)

        # 상태와 관측 간의 상관 공분산 계산
        Pxz = np.zeros((self.n, self.m))
        for k in range(2 * self.n + 1):
            Pxz += W[k] * np.dot((fXi[:, k] - xp).reshape(self.n, 1), (hXi[:, k] - zp).reshape(1, self.m))

        # 칼만 이득 계산
        K = np.dot(Pxz, np.linalg.inv(Pz))

        # 상태와 공분산 갱신
        self.x = xp + np.dot(K, (z - zp))
        self.P = Pp - np.dot(K, np.dot(Pz, K.T))

        # 현재 상태 반환
        return self.x, self.P