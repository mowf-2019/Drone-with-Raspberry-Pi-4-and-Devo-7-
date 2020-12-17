import numpy as np
import math

class linear_Acc:
    def __init__(self,Ts):
        self.SamplePeriod = Ts
        self.Kp = 1
        self.Quaternion=[1, 0, 0, 0]

    def AHRS(self, Gyroscope, Accelerometer):
        q = self.Quaternion
        Accelerometer = Accelerometer / np.linalg.norm(Accelerometer)
        v = [2*(q[1]*q[3] - q[0]*q[2]), 2*(q[0]*q[1] + q[2]*q[3]), q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2]
        e = np.cross(Accelerometer, v)
        Gyroscope = Gyroscope + self.Kp * e
        qDot = 0.5 * np.array(self.quaternProd(q, [0, Gyroscope[0], Gyroscope[1], Gyroscope[2]]))
        q = q + qDot * self.SamplePeriod
        self.Quaternion = q / np.linalg.norm(q)
        return self.Quaternion
        
    def quaternProd(self,a,b):
        ab=[0, 0, 0, 0]

        ab[0] = a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3]
        ab[1] = a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2]
        ab[2] = a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1]
        ab[3] = a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]
        return ab
        
    def quatern2rotMat(self,q):
        R = np.zeros((3, 3))
        R[0][0] = 2*q[0]**2-1+2*q[1]**2
        R[0][1] = 2*(q[1]*q[2]+q[0]*q[3])
        R[0][2] = 2*(q[1]*q[3]-q[0]*q[2])
        R[1][0] = 2*(q[1]*q[2]-q[0]*q[3])
        R[1][1] = 2*q[0]**2-1+2*q[2]**2
        R[1][2] = 2*(q[2]*q[3]+q[0]*q[1])
        R[2][0] = 2*(q[1]*q[3]+q[0]*q[2])
        R[2][1] = 2*(q[2]*q[3]-q[0]*q[1])
        R[2][2] = 2*q[0]**2-1+2.*q[3]**2