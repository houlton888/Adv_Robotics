from XRPLib.differential_drive import DifferentialDrive
from machine import Timer
import time, math, gc, os
from XRPLib.imu import IMU
drivetrain = DifferentialDrive.get_default_differential_drive()

gc.collect() # empty RAM

# Data collection
data = [] # temp. storage
data_interval = 100 #ms
filename = "data.csv"

if filename in os.listdir():
    os.remove(filename)
    print(f"{filename} deleted.")
else:
    print(f"{filename} does not exist.")

class KalmanFilter:
    def __init__(self, data_interval):
        self.xrp_imu = IMU.get_default_imu()
        self.track_width = 15.5 #cm
        self.wheel_diam = 6 #cm
        self.RPMtoCMPS = (math.pi * self.wheel_diam) / 60     # Covert from RPM to cm/s
        self.CMPStoRPM = 60 / (math.pi * self.wheel_diam)     # Covert from cm/s to RPM
        self.x = 0
        self.x_kin = 0
        self.y = 0
        self.y_kin = 0
        self.theta = 0
        self.x_theta_kin = 0
        self.last_data_time = time.ticks_ms()
        self.timer = Timer()
        self.data_interval = data_interval
        # State estimates

        self.x_x = self.x_y = self.x_theta = 0.0
        self.z_x = self.z_y = self.z_theta = 0.0

        # Covariance matrices
        self.P = [
            [0.1, 0.0, 0.0],
            [0.0, 0.1, 0.0],
            [0.0, 0.0, 0.1]
        ]

        self.Q = [
            [0.01, 0.0, 0.0],
            [0.0, 0.01, 0.0],
            [0.0, 0.0, 0.01]
        ]

        self.R = [
            [0.05, 0.0, 0.0],
            [0.0, 0.05, 0.0],
            [0.0, 0.0, 0.01]
        ]

    def get_gyro_rps(self):
        return self.xrp_imu.get_acc_gyro_rates()[1][2] * (math.pi/180000)
    
    def multiply_matrices(self, A, B):
        # Check if multiplication is possible
        if len(A[0]) != len(B):
            raise ValueError("Number of columns in A must match number of rows in B.")

        # Initialize result matrix with zeros
        result = [[0 for _ in range(len(B[0]))] for _ in range(len(A))]

        # Perform matrix multiplication
        for i in range(len(A)):
            for j in range(len(B[0])):
                for k in range(len(B)):
                    result[i][j] += A[i][k] * B[k][j]

        return result
    
    def add_matrices(self, A, B):
        # Check if dimensions match
        if len(A) != len(B) or len(A[0]) != len(B[0]):
            raise ValueError("Matrices must have the same dimensions.")

        # Add the matrices
        result = [
            [A[i][j] + B[i][j] for j in range(len(A[0]))]
            for i in range(len(A))
        ]

        return result
    
    def update_step(self):
        dt = self.data_interval/1000
        gyro_z_rad_per_s = self.get_gyro_rps()
        self.z_theta += gyro_z_rad_per_s * dt

        # Kalman update for theta
        innovation = self.z_theta - self.x_theta
        P_theta = self.P[2][2]
        R_theta = self.R[2][2]
        K_theta = P_theta / (P_theta + R_theta)

        self.theta = self.x_theta + K_theta * innovation
        self.x_theta = self.theta
        self.P[2][2] = (1 - K_theta) * self.P[2][2]

    def update(self):
        Vr = drivetrain.right_motor.get_speed()*self.RPMtoCMPS
        Vl = -1*drivetrain.left_motor.get_speed()*self.RPMtoCMPS

        dt = self.data_interval/1000

        # Below keeps track of x,y,theta just for kinematic estimates:
        if Vr != Vl:
            R = (self.track_width/2)*(Vr+Vl)/(Vr-Vl)
            w = (Vr - Vl)/self.track_width
            self.x_kin = self.x_kin - R*math.sin(self.x_theta_kin) + R*math.sin(self.x_theta_kin+w*dt)
            self.y_kin = self.y_kin + R*math.cos(self.x_theta_kin) - R*math.cos(self.x_theta_kin+w*dt)
            self.x_theta_kin = self.x_theta_kin + w*dt
        else:
            self.x_kin = self.x_kin + Vr*math.cos(self.x_theta_kin)*dt
            self.y_kin = self.y_kin + Vr*math.sin(self.x_theta_kin)*dt
            self.x_theta_kin = self.x_theta_kin

        # This is for the kalman filter. Repeated so can see how kalman differes from kinemtaic for the same motions
        if Vr != Vl:
            R = (self.track_width/2)*(Vr+Vl)/(Vr-Vl)
            w = (Vr - Vl)/self.track_width
            self.x = self.x - R*math.sin(self.theta) + R*math.sin(self.theta+w*dt)
            self.y = self.y + R*math.cos(self.theta) - R*math.cos(self.theta+w*dt)
            self.x_theta = self.x_theta + w*dt
        else:
            self.x = self.x + Vr*math.cos(self.theta)*dt
            self.y = self.y + Vr*math.sin(self.theta)*dt
            self.x_theta = self.x_theta

        self.P = self.add_matrices(self.P, self.Q)

        self.update_step()
        # Store every data_interval, robot pose in data[]
        current_time = time.ticks_ms() 

        # UNLIKE KINEMTICS LAB, WE WILL ONLY RECORD FINAL POSITIONS, SO I BELOW COMMENTED OUT:

        # if time.ticks_diff(current_time, self.last_data_time) >= data_interval:
        #     data.append([
        #         self.x,
        #         self.y,
        #         self.x_theta
        #     ])
        #   print(self.theta)
        #   self.last_data_time = current_time

    def start_timer(self):
        self.timer.init(period=int(self.data_interval), 
                        mode=Timer.PERIODIC,
                        callback=lambda t: self.update())
        print("[System] Timer started")

    def print_store_final_pos(self):
        kin_final_pos = [self.x_kin, self.y_kin, self.x_theta_kin]
        kalman_final_pos = [self.x, self.y, self.theta]
        print("final Kinematic positions: "+str(kin_final_pos))
        print("final kalman filter positions: "+str(kalman_final_pos))

        with open(filename, "w") as f:
            f.write("final Kinematic positions: ")
            row = kin_final_pos
            f.write(f"{row[0]},{row[1]},{row[2]}\n")
            f.write("final kalman filter positons: ")
            row = kalman_final_pos
            f.write(f"{row[0]},{row[1]},{row[2]}\n")
            print("inside open func")
        f.flush()
        os.sync()


kinematics = KalmanFilter(data_interval)
kinematics.start_timer()

# Program robot driving behavior
# ...
drivetrain.set_speed(25,25)
time.sleep(3)
drivetrain.turn(720, max_effort=1, timeout=5)
drivetrain.set_speed(25,25)
time.sleep(5)

drivetrain.set_effort(0,0)
drivetrain.stop()
# After experiment, write into file

kinematics.print_store_final_pos()

# DOWNLOAD FILE FROM PICO TO COMPUTER
print("experiment completed")
drivetrain.stop()
