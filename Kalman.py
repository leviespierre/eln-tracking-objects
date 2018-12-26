from numpy import * 
from numpy.linalg import inv 
import matplotlib.pyplot as plt 
from math import *

class Kalman: # Definición de clase, class name_of_the_class
    def _init_(self):
        ####### X[k-1] ###########
        self.x = None 
        self.y = None 
        self.Vx = None 
        self.Vy = None 
        self.Ax = None
        self.Ay = None 
        ######### Initial Values #############
        self.Dt = 0.04 
        self.YawRate = None
        self.cosYawAngle = None 
        self.sinYawAngle = None
        self.FA = None 
        self.PNC = 2.25 
        self.InitValOfP = 1000
        self.VelStabilityFactor = None 
        self.SignedEgoSpeed = None
        self.Overground = None 
        self.MountingtoCenterY = None 
        self.MountingtoCenterX = None 
        self.SignedEgoAcel = None
        self.rangeRad= None 
        self.AZang= None
        ############# Kalman´s Parameters ##############
        self.X = None
        self.A = None 
        self.P = None 
        self.Q = None 
        self.H_Dist = None 
        self.R = None 
        self.I = None 
        ############# New X(k-1) ############
        self.Oldx = None
        self.Oldy = None
        self.OldVx = None
        self.OldVy = None 
        ###################### Relative Velocities ###########################
        self.VelRelVx = None
        self.VelRelVy = None
        self.RelVelVx = None
        self.RelVelVy = None  
        ###################### Aceleration - Framawork  ######################
        self.AB1 = None
        self.AB2 = None
        ################ Position prediction ####################
        self.XNew = None
        ############## Velocities prediction #####################
        ################ Aceleration prediction ####################
        #self. = array([[0],[0],[0],[0],[0],[0]]) 
        ############# New #######
        self.theta = None # Filtro
        self.T = None # Matriz de transformacion para cambiar de marco de referencia  
        self.C= None # vector de coordenadas de los clousters 
        self.Clost = None # Vector de las posiciones de los closter medidas desde el ego
        self.Newmarco = None # coordenadas de los closter medidias desde el objeto, cambio de marco de referencia
        self.thetaError = None 
        self.E = None # Matriz temporal, me sirve para sacar R 
        ######## Me sirve para sacar interpolación lineal ############
        self.maxerror = None 
        self.MinError = None
        self.MinRSP = None
        self.MaxRSP = None
        self.MaxAng = None
        #########################################################################################################
        self.J = None # Matriz jacobiana para sacar la actualización de E 
        self.Range = None 
        self.AgeFactor = None
        self.DGK = None #  Denominator Of Kalman´s Gain
        self.PS = None 
        self.I = None
        self.XNew = None
        self.X_measurement= None #[x 0 0 y 0 0 ] vector columna informacion del cluster

    def set_initialKalVal(self, posx, posy, velx, vely, dt, yawRate,  sinYangle, cosYangle, egoSpeed, mountingCenterY, mountingCenterX, egoAccel):
        self.x = posx 
        self.y = posy 
        self.Vx = velx 
        self.Vy = vely 
        self.Ax = 0.001
        self.Ay = 0.001
        self.Dt = dt 
        self.YawRate = yawRate 
        self.cosYawAngle = cosYangle 
        self.sinYawAngle = sinYangle
        self.SignedEgoSpeed = egoSpeed
        self.MountingtoCenterY = mountingCenterX
        self.MountingtoCenterX = mountingCenterY
        self.SignedEgoAcel = egoAccel
        self.PNC = 2.25 
        self.InitValOfP = 1000
        self.FA = 1-(0.3*0.04) 
        self.X = array([self.x,self.Vx,self.Ax,self.y,self.Vy,self.Ay]) 



    def Matrix_A_P_Q_H_R_I(self):
        # Transition Matrix 
        self.A = array( [ [1, self.Dt, 0, self.YawRate*self.Dt, 0, 0], [0, 1, self.Dt, 0, self.YawRate*self.Dt, 0], 
        [0, 0, self.cosYawAngle*self.FA, 0, 0, self.sinYawAngle*self.FA], [-self.YawRate*self.Dt, 0, 0, 1, self.Dt, 0], 
        [0, -self.YawRate*self.Dt, 0, 0, 1, self.Dt], [0, 0, -self.sinYawAngle*self.FA, 0, 0, self.cosYawAngle*self.FA] ] )
        # Covariance Matrix of The Process
        self.P = diag( ( self.InitValOfP ,self.InitValOfP ,self.InitValOfP ,self.InitValOfP ,self.InitValOfP ,self.InitValOfP ) )
        # Process Noise Matrix
        self.Q = diag( ( self.PNC*self.Dt*self.Dt , self.PNC*self.Dt*self.Dt, self.PNC*self.Dt*self.Dt, self.PNC*self.Dt*self.Dt,
        self.PNC*self.Dt*self.Dt, self.PNC*self.Dt*self.Dt ) )
        # Transformation Matrix 
        #self.H = diag( ( 1 ,1 ,1 ,1 ,1 ,1 ) )
        # Radar Sensitivity
        #self.R = diag( ( 25 ,25 ,6 ,6 ,1 ,1 ) )
        # Identity Matrix
        I = identity(6)

    def OldStateVector(self, cyclesLife):
        self.cyclesLife=cyclesLife
        self.Oldx = self.X[0]
        self.Oldy = self.X[3]
        self.Overground = sqrt((self.X[1]**2)+(self.X[4]**2))
        if self.Overground<5:
            self.VelStabilityFactor= interp(cyclesLife, [4, 7], [0.0 ,1.0])
        else:
            self.VelStabilityFactor=1
        self.OldVx = self.X[1]*self.VelStabilityFactor
        self.OldVy = self.X[4]*self.VelStabilityFactor

    def RelativeVelocities(self):
        self.VelRelVx =  self.OldVx - self.SignedEgoSpeed + self.YawRate*(self.Oldy + self.MountingtoCenterY)
        self.VelRelVy = self.OldVy -  self.YawRate*(self.Oldx + self.MountingtoCenterX)

    def AceleratioFramework(self):
        self.AB1 = self.X[2]*self.cosYawAngle + self.X[5]*self.sinYawAngle
        self.AB2 = -self.X[2]*self.sinYawAngle + self.X[5]*self.cosYawAngle

    def KalmanFilter_Predict(self):
        self.X[0] +=  self.VelRelVx*self.Dt
        self.X[3] +=  self.VelRelVy*self.Dt
        self.X[1] += (self.Ax - self.SignedEgoAcel + self.YawRate*self.OldVy)*self.Dt
        self.X[4] += (self.Ay - self.YawRate*self.OldVx)*self.Dt
        self.X[2] *= self.FA
        self.X[5] *= self.FA
        self.P = dot(self.A, dot(self.P, self.A.T))
    
    def CoordinateTransformation(self, dX, dY):#pasar como parametro x0 y x3
        # orientation
        self.theta = atan2(self.X[3], self.X[0])
        # Homogeneous Transformation Matrix
        self.T = array( [ [ cos(self.theta) , -sin(self.theta), self.X[0] ], [ sin(self.theta) , cos(self.theta), self.X[3] ],
        [ 0, 0, 1 ] ] )
        # X[0] and [3] are the centers of the objets
        self.Newmarco = array([dX,dY,1])
        # Newmarco es la medicion de los closter (x,y) desde el ego 
        self.Newmarco = dot(self.T,self.Newmarco)
        dist2obj=sqrt((self.Newmarco[0]**2)+(self.Newmarco[1]**2))
        return dist2obj
        # Newmarco ahora mide las coordenadas de los closter desde el marco de referencia del objeto 
    def KalmanFilter_Update(self, distX, distY, velX, velY, rangeRad, AZang, ClussinA, CluscosA):
        self.rangeRad=rangeRad
        self.AZang=AZang
        self.I = identity(6)
        self.X_measurement=[distX,0,0,distY,0,0]
        # Corrección de la posición
        # Initialization of matix H for correction of position 
        # Transformation Matrix 
        self.H_Dist = array( [ [1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0],  [0, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0],  [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0] ] ) 
        
        ############################ Calculo de theta ####################################
        self.maxerror = deg2rad(10.0)
        self.MinError = deg2rad(1.25)
        self.thetaError = interp(self.rangeRad,[0,12],[self.maxerror,self.MinError])
        self.MinRSP = deg2rad(70.0)
        self.MaxRSP = deg2rad(80.0)
        self.MaxAng = deg2rad(5.0)
        self.thetaError += interp(self.AZang,[self.MinRSP,self.MaxRSP],[0,self.MaxAng])
        ##################################################################################

        ######################## Calculo de la matriz temporal E #########################
        self.E = array( [ [ 0.75**2, 0 ], [ 0, self.thetaError**2 ] ] )
        self.Range = sqrt(distX**2 + distY**2) # Raiz del cuadrado de clust.Dist de (x) y (y)
        self.J = array([[CluscosA,-ClussinA*self.Range], [ClussinA, CluscosA*self.Range]])
        self.E = dot(self.J, dot(self.E, self.J.T)) 
        ##################################################################################

        ######################## Calculo de la matriz de error R #########################
        self.AgeFactor = interp(self.cyclesLife,[0,10.0],[0.05,1.0])
        self.R = diag( ( self.AgeFactor*self.E[0][0], 0 ,0 ,self.AgeFactor*self.E[1][1] ,0,0 ) )
        

        #self.R = array( [ [ self.AgeFactor*self.E[0][0], self.AgeFactor*self.E[0][1] ], 
        #[ self.AgeFactor*self.E[1][0], self.AgeFactor*self.E[1][1] ] ] )
        ##################################################################################

        ####################### Cálculo de la Gananacia de Kalman ################################
        self.DGK = self.R + dot(self.H_Dist, dot(self.P, self.H_Dist.T)) # Denominator Of Kalman´s Gain
        self.DGK[0][0] = 1/self.DGK[0][0]
        self.DGK[3][3] = 1/self.DGK[3][3]
        self.K = dot(self.P, dot(self.H_Dist.T, self.DGK)) # Kalman´s Gain
        
        self.PS = dot(self.H_Dist,self.X) # Predicted state  

        self.X = self.X + dot(self.K,(self.X_measurement-self.PS)) # Calculate the current State 
        
        self.P = dot((self.I -dot(self.K,self.H_Dist)),self.P)

















