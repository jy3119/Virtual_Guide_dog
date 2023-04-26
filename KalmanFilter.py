class KalmanFilter:
    def __init__(self, q =0.01, r=0.1, p=1, x=0):
        self.q = q
        self.r = r
        self.p = p
        self.x = x

    def filter(self, z):
        x_hat = self.x
        p_hat = self.p + self.q

        k = p_hat / (p_hat + self.r)
        self.x = x_hat + k * (z - x_hat)
        self.p = (1 - k) * p_hat

        return self.x