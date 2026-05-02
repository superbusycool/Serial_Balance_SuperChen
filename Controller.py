class PID:
    def __init__(self, p,i,d,integral_limit,output_limit):
        # self.kp, self.ki, self.kd = pid_params
        self.kp = p
        self.ki = i
        self.kd = d
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.Iout = 0
        self.error = 0
        self.prev_error = 0
        self.Pout = 0
        self.i_term = 0
        self.Dout = 0
        self.Output = 0
    
    def calc(self, current, target):
        self.error = target - current
        
        self.Pout = self.kp * self.error
        self.i_term = self.ki * self.error
        self.Dout = self.kd * (self.error - self.prev_error)
        self.Iout += self.i_term


        # 积分项限幅，防止积分过大导致系统不稳定
        if self.Iout > self.integral_limit:
            self.Iout = self.integral_limit
            self.i_term = 0
        elif self.Iout < -self.integral_limit:
            self.Iout = -self.integral_limit
            self.i_term = 0


        self.prev_error = self.error
        
        self.Output = self.Pout + self.Iout + self.Dout

        if self.Output > self.output_limit:
            self.Output = self.output_limit
        elif self.Output < -self.output_limit:
            self.Output = -self.output_limit
        
        return self.Output
    
    def clear(self):

        self.error = 0
        self.i_term = 0
        self.Dout = 0
        self.Iout = 0
        self.Pout = 0
        self.Output = 0
        self.prev_error = 0