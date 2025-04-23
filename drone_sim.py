import numpy as np
import time
import csv

class PID:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.out_min, self.out_max = out_min, out_max

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        out = self.kp*error + self.ki*self.integral + self.kd*derivative
        out = max(min(out, self.out_max), self.out_min)
        self.prev_error = error
        return out

def simulate():
    dt, t_end, z_target = 0.01, 10.0, 5.0
    mass, gravity = 1.0, 9.81

    z, vz = 0.0, 0.0
    pid = PID(2.0, 0.5, 1.0, 0.0, 20.0)

    with open("flight_data_py.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time","z","vz","thrust"])
        t = 0.0
        while t <= t_end:
            error = z_target - z
            thrust = pid.update(error, dt)
            az = (thrust - mass*gravity)/mass
            vz += az * dt
            z  += vz * dt
            writer.writerow([f"{t:.3f}", f"{z:.3f}", f"{vz:.3f}", f"{thrust:.3f}"])
            t += dt

if __name__ == "__main__":
    start = time.perf_counter()
    simulate()
    elapsed = time.perf_counter() - start
    print(f"Python simulation en {elapsed:.3f} s")
