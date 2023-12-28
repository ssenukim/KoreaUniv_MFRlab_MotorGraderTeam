import sys
# sys.path.append('c:\\users\\jisoo\\appdata\\local\\programs\\python\\python310\\lib\\site-packages')
from casadi import *
import math
from socket import *
from threading import *
from MG_Variable_231007 import *

import time
import struct
import tkinter as tk
from tkinter import ttk
import csv
from mti_receive import XdaCallback
import xsensdeviceapi as xda
from threading import Lock
from datetime import datetime
import numpy as np

# UI에 필요한 라이브러리


# NUVO -> UI
# HOST_S = '169.254.156.104'          # 모터 그레이더 supervisor IP
# HOST_S = '169.254.156.106'          # 테스트용 노트북 IP4
# HOST_S = '192.168.0.74'          # 테스트용 노트북 IP

# PORT_S = 9999                       # Supervisor - UI 포트
# PORT_S_camera = 9998                # 영상전송 용 포트    


# CRIO -> NUVO
CRIO_IP = '192.168.0.19'       # CRIO 고정 IP

# CRIO_IP = '192.168.0.176'         #  집컴

# CRIO_IP = '192.168.0.74'          #  로컬
# CRIO_IP = '163.152.126.208'       # 연구실 컴퓨터

# CRIO_IP = '127.0.0.1'             # 연구실 컴퓨터

# CRIO_IP = '192.168.0.14'          # 노트북

# CRIO_IP = '169.254.156.104'            # NUVO 고정 IP

# CRIO_IP = '169.254.156.106'            # 노트북 자체 IP
CRIO_PORT = 6004                    # 8월중으로 5003으로 변경 예정

    
class Server_Client():
    def __init__(self):

        self.thread_lock = Lock()

        self.crio_ip = CRIO_IP
        self.crio_port = CRIO_PORT
        
        self.MC_Mode = 0
        
        self.sock_list = []
        
        self.SVP_client_socket = socket(AF_INET, SOCK_STREAM)   
        
        self.SVP_client_socket.setblocking(True)
        
        self.logStatus = 0
        self.stopStatus = 0
        self.MPCStatus = 1
        # Steer값이나 속도값이 UI에 나오는지 확인 - velocity
        self.mainCylArea = np.pi*(0.08/np.pi)**2         # [m2]
        self.tiltCylArea = np.pi*(0.07/np.pi)**2         # [m2]
        
        self.vel        = 0
        self.HeadingAngle = 0
        self.steeringAngle = 0
        
        # MTi data value
        self.euler_X    = 0
        self.euler_Y    = 0
        self.euler_Z    = 0
        
        self.LeftCyl    = 0
        self.RightCyl   = 0
        self.SideCyl    = 0
        self.TiltCyl    = 0
        
        self.BladeRotation = 0
        
        self.LeftCylPres1 = 0
        self.LeftCylPres2 = 0
        self.LeftCylPres = 0
        self.RightCylPres1 = 0
        self.RightCylPres2 = 0
        self.RightCylPres = 0
        self.SideCylPres1 = 0
        self.SideCylPres2 = 0
        self.SideCylPres = 0
        self.TiltCylPres1 = 0
        self.TiltCylPres2 = 0
        self.TiltCylPres = 0
        
        self.optSteering = 0
        self.HeadingAngle =0
        self.IMU1_Roll  = 0
        self.IMU1_Pitch = 0
        self.IMU2_Roll  = 0
        self.IMU2_Pitch = 0
        
        self.latitude   = 0
        self.longitude  = 0
        self.altitude   = 0
        self.BldSideShift = 0
        self.current_date = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')

        # IMU 이동평균을 위한 코드
        self.windowSize = 10
        self.roll_Data = np.zeros(self.windowSize)
        self.pitch_Data = np.zeros(self.windowSize)
        self.rmsIndex = 0
        self.roll_total = 0.0
        self.pitch_total = 0.0
        self.roll_num_elements = 0
        self.pitch_num_elements = 0

        self.calibratedRoll = 0
        self.calibratedPitch = 0
        
        self.drivingStatus = 0

        self.CMDB = call_MG_CRIO_CMDB()
        self.CMDD = call_MG_CRIO_CMDD()

        
        self.DT         = 0.2    # discretization time between timesteps (s)
        self.L_F        = 1.6 		# distance from CoG to front axle (m)
        self.L_R        = 1.4 		# distance from CoG to rear axle (m)
        # V_MIN      = 1.0    # min/max velocity constraint (m/s)
        # V_MAX      = 70.0     
        self.A_MIN          = -15.0   # min/max acceleration constraint (m/s^2)
        self.A_MAX          =  10.0    
        self.DF_MIN         = -47.5/180*math.pi   # min/max front steer angle constraint (rad)
        self.DF_MAX         =  47.5/180*math.pi    
        self.A_DOT_MIN      = -10   # min/max jerk constraint (m/s^3)
        self.A_DOT_MAX      =  10
        self.DF_DOT_MIN     = -np.pi/2   # min/max front steer angle rate constraint (rad/s)
        self.DF_DOT_MAX     =  np.pi/2
        self.BETA_MIN       = -np.pi
        self.BETA_MAX       = np.pi 
        self.Q              = MX(np.diag([2,0,4,0])) # weights on ey, e_phi.
        # R = MX(1.)       # weights on control variables
        self.R              = MX(0.2)       # weights on control variables
        self.Nc             = 15
        self.Np             = 20
        # Nc : control horizon - prediction horizon의 10~20 %
        # Np : prediction horizon - 최적화 고려하는 state 단계

        self.m = 14093       # Mass of car [kg]
        self.Iz = 25080      # Moment of inertia about Z axis []
        self.lf = 4.347      # Distance between Center of Gravity and Front axle 
        self.lr = 1.568      # Distance between Center of Gravity and Rear axle
        self.Cf = 2748       # Cornering stiffness of the front tires (Np/rad)
        self.Cr = 3810       # Cornering stiffness of the rear tires (Np/rad)
        # self.Vx = 10./3.6    # 실시간으로 변하는 값으로 해야 함
        
        self.initialize_GUI()   
        print('GUI initialized')  
        
        self.TCP_connect()
       
        
    def TCP_connect(self):
        # Supervisor client -> CRIO(생기원) server에 접속 요청
        self.SVP_client_socket.connect((self.crio_ip,self.crio_port))           
        print('CRIO와 SupervisorPC가 연결되었습니다.')
        

        '''
        Thread 1 : CRIO에서 10ms마다 신호 받아오고 재처리하여 100ms 간격으로 자체 UI로 전달
        '''

        rcvFromCRIO = Thread(target=self.receive_data_from_CRIO_th1, args=())         # args : 실행되는 함수에서 써야 하는 인자 표시 - 현재는 필요 없음

        reprocessData = Thread(target=self.reprocess_data_th2, args=())

        mti_th = Thread(target=self.mti_data_th4, args=())
        MPC = Thread(target=self.MPC_driving, args=())

        # MPC.start()
        mti_th.start()
        rcvFromCRIO.start()
        reprocessData.start()
        # sendToCRIO.start()            # UI 명령 변환하여 전달
      
    def mti_data_th4(self):
        
        print("Creating XsControl object...")
        control = xda.XsControl_construct()
        assert(control is not 0)

        xdaVersion = xda.XsVersion()
        xda.xdaVersion(xdaVersion)
        print("Using XDA version %s" % xdaVersion.toXsString())

        try:
            print("Scanning for devices...")
            portInfoArray =  xda.XsScanner_scanPorts()

            # Find an MTi device
            mtPort = xda.XsPortInfo()
            for i in range(portInfoArray.size()):
                if portInfoArray[i].deviceId().isMti() or portInfoArray[i].deviceId().isMtig():
                    mtPort = portInfoArray[i]
                    break

            if mtPort.empty():
                raise RuntimeError("No MTi device found. Aborting.")

            did = mtPort.deviceId()
            print("Found a device with:")
            print(" Device ID: %s" % did.toXsString())
            print(" Port name: %s" % mtPort.portName())

            print("Opening port...")
            if not control.openPort(mtPort.portName(), mtPort.baudrate()):
                raise RuntimeError("Could not open port. Aborting.")

            # Get the device object
            self.device = control.device(did)
            assert(self.device is not 0)

            print("Device: %s, with ID: %s opened." % (self.device.productCode(), self.device.deviceId().toXsString()))

            # Create and attach callback handler to device
            callback = XdaCallback()
            self.device.addCallbackHandler(callback)

            # # Put the device into configuration mode before configuring the device
            # print("Putting device into configuration mode...")
            # if not device.gotoConfig():
            #     raise RuntimeError("Could not put device into configuration mode. Aborting.")

            # print("Configuring the device...")
            # configArray = xda.XsOutputConfigurationArray()
            # configArray.push_back(xda.XsOutputConfiguration(xda.XDI_PacketCounter, 0))
            # configArray.push_back(xda.XsOutputConfiguration(xda.XDI_SampleTimeFine, 0))


            # '''
            # 여기가 센서 설정 변경하는 곳으로 추정됨
            # : 
            # '''


            # if device.deviceId().isImu():
            #     configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Acceleration, 100))
            #     configArray.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, 100))
            #     configArray.push_back(xda.XsOutputConfiguration(xda.XDI_MagneticField, 100))
            #     print('isIMU')
            # elif device.deviceId().isVru() or device.deviceId().isAhrs():
            #     configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, 100))
            #     print('isAhres')
            # elif device.deviceId().isGnss():
            #     configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, 100))
            #     configArray.push_back(xda.XsOutputConfiguration(xda.XDI_LatLon, 100))
            #     configArray.push_back(xda.XsOutputConfiguration(xda.XDI_AltitudeEllipsoid, 100))
            #     configArray.push_back(xda.XsOutputConfiguration(xda.XDI_VelocityXYZ, 100))
            #     print('isGnss')
            # else:
            #     raise RuntimeError("Unknown device while configuring. Aborting.")

            # if not device.setOutputConfiguration(configArray):
            #     raise RuntimeError("Could not configure the device. Aborting.")

            print("Creating a log file...")
            logFileName = "mtlog.mtb"
            if self.device.createLogFile(logFileName) != xda.XRV_OK:
                raise RuntimeError("Failed to create a log file. Aborting.")
            else:
                print("Created a log file: %s" % logFileName)

            # logfileName = "mtlog.mtb"
            # if not control.openLogFile(logfileName):
            #     raise RuntimeError("Failed to open log file. Aborting.")
            # print("Opened log file: %s" % logfileName)


            print("Putting device into measurement mode...")
            if not self.device.gotoMeasurement():
                raise RuntimeError("Could not put device into measurement mode. Aborting.")

            print("Starting recording...")
            if not self.device.startRecording():
                raise RuntimeError("Failed to start recording. Aborting.")

            # print("Main loop. Recording data for 10 seconds.")

            startTime = xda.XsTimeStamp_nowMs()
            # with open('mti_data_logging_calibration.csv', 'a', newline='') as file:
            #     writer = csv.writer(file)

            while 1:
                if callback.packetAvailable():
                    # Retrieve a packet
                    mtiStart_tim = time.time()
                    packet = callback.getNextPacket()

                    s = ""

                    
                    # if packet.containsCalibratedData():
                    #     acc = packet.calibratedAcceleration()
                    #     s = "Acc X: %.2f" % acc[0] + ", Acc Y: %.2f" % acc[1] + ", Acc Z: %.2f" % acc[2]

                    #     gyr = packet.calibratedGyroscopeData()
                    #     s += " |Gyr X: %.2f" % gyr[0] + ", Gyr Y: %.2f" % gyr[1] + ", Gyr Z: %.2f" % gyr[2]

                    #     mag = packet.calibratedMagneticField()
                    #     s += " |Mag X: %.2f" % mag[0] + ", Mag Y: %.2f" % mag[1] + ", Mag Z: %.2f" % mag[2]

                    # if packet.containsOrientation():
                    #     quaternion = packet.orientationQuaternion()
                    #     s = "q0: %.2f" % quaternion[0] + ", q1: %.2f" % quaternion[1] + ", q2: %.2f" % quaternion[2] + ", q3: %.2f " % quaternion[3]

                    #     euler = packet.orientationEuler()
                    #     s += " |Roll: %.2f" % euler.x() + ", Pitch: %.2f" % euler.y() + ", Yaw: %.2f " % euler.z()

                    # if packet.containsLatitudeLongitude():
                    #     latlon = packet.latitudeLongitude()
                    #     s += " |Lat: %7.2f" % latlon[0] + ", Lon: %7.2f " % latlon[1]

                    # if packet.containsAltitude():
                    #     s += " |Alt: %7.2f " % packet.altitude()

                    # if packet.containsVelocity():
                    #     vel = packet.velocity(xda.XDI_CoordSysEnu)
                    #     s += " |E: %7.2f" % vel[0] + ", N: %7.2f" % vel[1] + ", U: %7.2f " % vel[2]
                    
                    # writer.writerow(euler.x())
                    # writer.writerow(euler.y())
                    # writer.writerow(euler.z())
                    
                    
                    # t = "%.2f " % euler.x()
                    # t += "%.2f " % euler.y()
                    # t += "%.2f " % euler.z()
                    
                    # print("%s\r" % t, end="", flush=True)
                    # print("%s\r" % s, end="", flush=True)
                    euler = packet.orientationEuler()
                    self.euler_X = euler.x()
                    self.euler_Y = euler.y()
                    self.euler_Z = euler.z()
     
            print("\nStopping recording...")
            if not self.device.stopRecording():
                raise RuntimeError("Failed to stop recording. Aborting.")

            print("Closing log file...")
            if not self.device.closeLogFile():
                raise RuntimeError("Failed to close log file. Aborting.")

            print("Removing callback handler...")
            self.device.removeCallbackHandler(callback)

            print("Closing port...")
            control.closePort(mtPort.portName())

            print("Closing XsControl object...")
            control.close()

        except RuntimeError as error:
            print(error)
            sys.exit(1)
        except:
            print("An unknown fatal error has occured. Aborting.")
            sys.exit(1)
        else:
            print("Successful exit.")               
                    # try:
                 
        # mti_logging_th.start()
        #sendCameraToUI_th.start()
    # Thread1 : receive data From CRIO and send MG_State to UI 
    
    def receive_data_from_CRIO_th1(self):
        # i = 0
        while True:
            try:
                self.bytearray_CRIO_Data = self.SVP_client_socket.recv(164)     # including CR/LF
                # print(len(self.bytearray_CRIO_Data))
                # self.MG_CRIO_Data = struct.unpack(
                #     '>ccccc'
                #     ,self.bytearray_CRIO_Data[0:5]
                # )
                if len(self.bytearray_CRIO_Data)==164:
                
                    self.MG_CRIO_Data = struct.unpack(
                        '>ccccchhhhhhhhhHHHHHHHHHHHHHHHHHHhhhhhhhhhhhhhhhhBBBBhhhhBBfBddddBBHHHHHHHHHcc'
                        ,self.bytearray_CRIO_Data
                    )
                    
                    # 전처리 구간
                    self.vel = self.MG_CRIO_Data[52]*0.001              # 예상 [m/s]  
                    
                    self.LeftCyl = self.MG_CRIO_Data[9] *0.1            # [mm]
                    self.RightCyl = self.MG_CRIO_Data[10]*0.1           # [mm]
                    self.SideCyl = self.MG_CRIO_Data[11]*0.1            # [mm]
                    self.BldSideShift = self.MG_CRIO_Data[12]*0.1       # [mm]
                    self.TiltCyl    = self.MG_CRIO_Data[13]*0.1         # [mm]
                                
                    self.IMU1_Roll = self.MG_CRIO_Data[39]*0.01 - 0.85        # [deg]         - 초기 IMU 설치 오차에 의한 offset 필요
                    self.IMU1_Pitch = self.MG_CRIO_Data[38]*0.01 - 0.5683       # [deg]         - 초기 IMU 설치 오차에 의한 offset 필요
                    
                    self.IMU2_Roll  = self.MG_CRIO_Data[47]*0.01     # [deg]
                    self.IMU2_Pitch = self.MG_CRIO_Data[46]*0.01     # [deg]

                    self.HeadingAngle = self.MG_CRIO_Data[63]*0.1           # 미지
                    
                    self.steeringAngle = self.MG_CRIO_Data[53]*0.1          # 미지
                    self.BladeRotation = self.MG_CRIO_Data[8]*0.1       # [deg]  -> 확인 필요
                    
                    # Waypoint 명령값을 2차원으로 변경하는 법 필요
                    self.latitude = self.MG_CRIO_Data[60]
                    self.longitude = self.MG_CRIO_Data[61]
                    self.altitude = self.MG_CRIO_Data[62]
                
                    self.LeftCylPres1 = self.MG_CRIO_Data[22]*0.01     # [MPa] 
                    self.LeftCylPres2 = self.MG_CRIO_Data[23]*0.01     # [MPa]
                    self.LeftCylPres = self.LeftCylPres2-self.LeftCylPres1
                    self.RightCylPres1 = self.MG_CRIO_Data[24]*0.01     # [MPa]
                    self.RightCylPres2 = self.MG_CRIO_Data[25]*0.01     # [MPa]
                    self.RightCylPres = self.RightCylPres2 - self.RightCylPres1
                    self.SideCylPres1 = self.MG_CRIO_Data[26]*0.01     # [MPa]
                    self.SideCylPres2 = self.MG_CRIO_Data[27]*0.01     # [MPa]
                    self.SideCylPres = self.SideCylPres2-self.SideCylPres1
                    self.TiltCylPres1 = self.MG_CRIO_Data[30]*0.01     # [MPa]
                    self.TiltCylPres2 = self.MG_CRIO_Data[31]*0.01     # [MPa]
                    self.TiltCylPres = self.TiltCylPres2 - self.TiltCylPres1
                    
                elif len(self.MG_CRIO_Data) == 9:   
                    # self.thread_lock.acquire()
                    self.MG_CRIO_Data_C = struct.unpack(
                    '>ccccccBcc'
                    ,self.bytearray_CRIO_Data
                    )
                    # 주행 상태를 판별하는 지표
                    self.drivingStatus = self.MG_CRIO_Data_C[6]
                    # self.thread_lock.release()

                else:
                    print(f"잘못된 길이의 프로토콜 전송 : {len(self.bytearray_CRIO_Data)}")
                    #self.KU_client_socket.close()    
                    #self.KU_server_socket.close()
         
            except: 
                print('Reprocess fail')
                continue
                
    def reprocess_data_th2(self):
        while 1:
            
            try:
                st_tim = time.time() 
                # self.start_reprocess_time = time.perf_counter()
                
                # if i%10==1:     # 로깅 시 7~9ms 소요, 100ms마다 UI에 반영함
                
                self.UI_vel.set(self.vel)
                self.UI_euler_X.set(self.drivingStatus)
                self.UI_euler_Y.set(self.euler_Y)
                self.UI_euler_Z.set(self.euler_Z)
                self.UI_LeftCyl.set(self.LeftCyl)
                self.UI_RightCyl.set(self.RightCyl)
                self.UI_SideCyl.set(self.SideCyl)
                self.UI_TiltCyl.set(self.TiltCyl)       
                self.UI_BladeRotation.set(self.BladeRotation)          
                self.UI_BldSideShift.set(self.BldSideShift)
                self.UI_LeftCylPres.set(self.LeftCylPres)      
                self.UI_RightCylPres.set(self.RightCylPres)  
                self.UI_SideCylPres.set(self.SideCylPres)  
                self.UI_TiltCylPres.set(self.TiltCylPres)  
                self.UI_IMU1_Roll.set(self.IMU1_Roll)
                self.UI_IMU1_Pitch.set(self.IMU1_Pitch)
                self.UI_IMU2_Roll.set(self.IMU2_Roll)
                self.UI_IMU2_Pitch.set(self.IMU2_Pitch)
                self.UI_latitude.set(self.latitude)
                self.UI_longitude.set(self.longitude)
                self.UI_altitude.set(self.altitude)
                
                # self.thread_lock.acquire()
                
                elp_tim = time.time() - st_tim
                time.sleep(0.5-elp_tim)

            except: 
                print('Reprocess th2 fail')
                continue
            
    def loadEstimation_th2(self):
        
        # thread_lock
        # 자세 계산 -> 자세 계산값을 그레이더 UI에 반영시켜야 하는지.
        # 자세 개선 완료 -> output으로 Mz와 Fy를 출력해야함
        
        
        return 0
                
    def MPC_driving(self):
        i = 0
        while True:
                 
            if self.MPCStatus == 0:
                
                try:
                    i += 1
                    self.thread_lock.acquire()
                    self.Vx = 1
                    if self.Vx == 0: continue
                    tim = time.time()        
                    self.x1 = MX.sym('x1')
                    self.x2 = MX.sym('x2')
                    self.x3 = MX.sym('x3')
                    self.x4 = MX.sym('x4')
                    self.x = vertcat(self.x1,self.x2,self.x3,self.x4)


                    # Reference path derivative, resistive force/torque as parameters - 일단 일정한 값 들어간다고 봄

                    self.phi_ref_diff = MX.sym('phi_ref_diff')           # 직선 주행
                    self.Ry = MX.sym('Ry')
                    self.Rt = MX.sym('Rt')

                    self.u = MX.sym('u')

                    # Model equation
                    self.xdot1 = self.x2
                    self.xdot2 = self.x2*(-2*self.Cf-4*self.Cr)/(self.m*self.Vx) + self.x3*(2*self.Cf+4*self.Cr)/self.m + self.x4*(-2*self.Cf*self.lf+4*self.Cr*self.lr)/(self.m*self.Vx)+self.u*(2*self.Cf)/self.m+(-self.Vx-(2*self.Cf*self.lf-4*self.Cr*self.lr)/(self.m*self.Vx))*self.phi_ref_diff-self.Ry/self.m
                    self.xdot3 = self.x4
                    self.xdot4 = self.x2*(-2*self.Cf*self.lf+4*self.Cr*self.lr)/(self.Iz*self.Vx) + self.x3*(2*self.Cf*self.lf-4*self.Cr*self.lr)/self.Iz + self.x4*(-2*self.Cf*self.lf**2-4*self.Cr*self.lr**2)/(self.Iz*self.Vx)+(2*self.Cf*self.lf)/self.Iz*self.u -(2*self.Cf*self.lf**2+4*self.Cr*self.lr**2)/(self.Iz*self.Vx)*self.phi_ref_diff-self.Rt/self.Iz

                    self.xdot = vertcat(self.xdot1,self.xdot2,self.xdot3,self.xdot4)

                    # Make system integration to function
                    self.intg_options = {'tf': self.DT, 'simplify': True, 'number_of_finite_elements': 4}
                    self.dae = dict(x = self.x, ode = self.xdot, p = vertcat(self.phi_ref_diff,self.Ry,self.Rt,self.u))
                    self.F = integrator('F','rk',self.dae,self.intg_options)
                    # F = integrator('F','rk',dae,opts)
                    # F = integrator('F','cvodes',dae,opts)

                    self.res = self.F(x0 = self.x, p = vertcat(self.phi_ref_diff,self.Ry,self.Rt,self.u))
                    self.x_next = self.res['xf']

                    # Integrator to function
                    self.f = Function('f',[self.x,self.u,self.phi_ref_diff,self.Ry,self.Rt],[self.x_next],['x','u','phi_ref_diff','Ry','Rt'],['x_next'])
                    # print(self.f)
                    # print('완')

                    self.opti=Opti()

                    self.X = self.opti.variable(4, self.Np+1)              # Decision variables for state trajectory
                    self.U = self.opti.variable(1, self.Nc+1)                
                    self.X0 = self.opti.parameter(4, 1)               # Initial state
                    self.U0 = self.opti.parameter(1, 1) 

                    self.Psi_ref = self.opti.parameter(self.Np,1)          # Trajectory information
                    self.RY = self.opti.parameter(1, 1)               # Calculated resistive lateral force
                    self.RT = self.opti.parameter(1, 1)               # Calculated resistive moment

                    self.opti.minimize(sumsqr(mtimes(self.Q,self.X))+sumsqr(mtimes(self.R,self.U)))

                    for k in range(self.Nc):
                        self.opti.subject_to(self.X[:,k+1]==self.f(self.X[:,k],self.U[:,k+1],self.Psi_ref[k,:],self.RY[0,0],self.RT[0,0]))

                    for k in range(self.Np-self.Nc+1):
                        self.opti.subject_to(self.X[:,k+self.Nc]==self.f(self.X[:,k+self.Nc-1],self.U[:,self.Nc-1],self.Psi_ref[k+self.Nc-1,:],self.RY[0,0],self.RT[0,0]))

                    for k in range(self.Nc):
                        self.opti.subject_to(self.U[:,k+1]-self.U[:,k]<=0.1)
                        self.opti.subject_to(self.U[:,k+1]-self.U[:,k]>=-0.1)   
                    self.opti.subject_to(self.U[:,0]==self.U0[0,0])
                    self.opti.subject_to(self.U<=self.DF_DOT_MAX)
                    self.opti.subject_to(self.U>=self.DF_DOT_MIN)

                    self.opti.subject_to(self.X[:,0]==self.X0)
                    self.tgrid = [self.DT*k for k in range(self.Np+1)]

                    '''
                    error value calculation code needed
                    '''
                    # opti.solver('sqpmethod', {'qpsol': 'qrqp'})
                    # opti.solver('ipopt')

                    self.p_opts = {'expand': True}
                    self.s_opts = {'max_cpu_time': 0.08, 'print_level': 0}
                    self.opti.solver('ipopt', self.p_opts, self.s_opts)

                    self.e1 = 1*np.sin(i*np.pi/20)
                    print(self.e1)
                    self.e1_dot = 0.
                    self.e2 = 0.
                    self.e2_dot = 0.
                    
                    self.opti.set_value(self.X0,[self.e1,self.e1_dot,self.e2,self.e2_dot])
                    self.opti.set_value(self.Psi_ref,0.*np.ones(self.Np))           # 변수로 받아야 함: 경로의 예상값          
                    self.opti.set_value(self.RY,[0.])                               # 변수로 받아야 함: 추정된 횡력      
                    self.opti.set_value(self.RT,[0.])                               # 변수로 받아야 함: 추정된 z축 모멘트
                    self.opti.set_value(self.U0,[0])                             # 변수로 받아야 함: 현재 steering value

                    self.sol = self.opti.solve()

                    self.optSteering = self.sol.value(self.U)[1]
                    self.prev_input = self.sol.value(self.U)[1]
                    
                    self.UI_opt_steer.set(self.optSteering)
                    
                    elp_tim = time.time()
                    self.thread_lock.release()
                    if i == 1000: i=0
                    print(f'소요 시간 : {elp_tim-tim}, \n예상되는 조향각 : {self.sol.value(self.U)}')
                    # time.sleep(0.2-elp_tim+tim)
                    
                except: 
                    print('MPC calculation failed')
                    pass 
            

    # 등속주행 제어를 위한 코드 : PI 제어를 사용할 예정, I gain 처리를 하는 것 검증 필요
    
    # def constant_velocity(self):
        
    #     # 속도 제어 식
    #     self.refVel = 10/3.6            # [m/s]
    #     self.velErr = self.refVel - self.vel
        
    #     return 0
    
    def mti_data_th4(self):
        
        print("Creating XsControl object...")
        control = xda.XsControl_construct()
        assert(control is not 0)

        xdaVersion = xda.XsVersion()
        xda.xdaVersion(xdaVersion)
        print("Using XDA version %s" % xdaVersion.toXsString())

        try:
            print("Scanning for devices...")
            portInfoArray =  xda.XsScanner_scanPorts()

            # Find an MTi device
            mtPort = xda.XsPortInfo()
            for i in range(portInfoArray.size()):
                if portInfoArray[i].deviceId().isMti() or portInfoArray[i].deviceId().isMtig():
                    mtPort = portInfoArray[i]
                    break

            if mtPort.empty():
                raise RuntimeError("No MTi device found. Aborting.")

            did = mtPort.deviceId()
            print("Found a device with:")
            print(" Device ID: %s" % did.toXsString())
            print(" Port name: %s" % mtPort.portName())

            print("Opening port...")
            if not control.openPort(mtPort.portName(), mtPort.baudrate()):
                raise RuntimeError("Could not open port. Aborting.")

            # Get the device object
            self.device = control.device(did)
            assert(self.device is not 0)

            print("Device: %s, with ID: %s opened." % (self.device.productCode(), self.device.deviceId().toXsString()))

            # Create and attach callback handler to device
            callback = XdaCallback()
            self.device.addCallbackHandler(callback)

            print("Creating a log file...")
            logFileName = "mtlog.mtb"
            if self.device.createLogFile(logFileName) != xda.XRV_OK:
                raise RuntimeError("Failed to create a log file. Aborting.")
            else:
                print("Created a log file: %s" % logFileName)

            print("Putting device into measurement mode...")
            if not self.device.gotoMeasurement():
                raise RuntimeError("Could not put device into measurement mode. Aborting.")

            print("Starting recording...")
            if not self.device.startRecording():
                raise RuntimeError("Failed to start recording. Aborting.")

            # print("Main loop. Recording data for 10 seconds.")

            startTime = xda.XsTimeStamp_nowMs()
            # with open('mti_data_logging_calibration.csv', 'a', newline='') as file:
            #     writer = csv.writer(file)

            while 1:
                if callback.packetAvailable():
                    # Retrieve a packet
                    mtiStart_tim = time.time()
                    packet = callback.getNextPacket()

                    s = ""
                    if packet.containsOrientation():
                        euler = packet.orientationEuler()
                        self.euler_X = euler.x()
                        self.euler_Y = euler.y()
                        self.euler_Z = euler.z()
                    
                    # try:
                    #     time.sleep(0.1 - time.time() + mtiStart_tim)
                    # except: pass

            print("\nStopping recording...")
            if not self.device.stopRecording():
                raise RuntimeError("Failed to stop recording. Aborting.")

            print("Closing log file...")
            if not self.device.closeLogFile():
                raise RuntimeError("Failed to close log file. Aborting.")

            print("Removing callback handler...")
            self.device.removeCallbackHandler(callback)

            print("Closing port...")
            control.closePort(mtPort.portName())

            print("Closing XsControl object...")
            control.close()

        except RuntimeError as error:
            print(error)
            sys.exit(1)
        except:
            print("An unknown fatal error has occured. Aborting.")
            sys.exit(1)
        else:
            print("Successful exit.")
        
    def initialize_GUI(self):

        # UI 생성
        self.SV_GUI = tk.Tk()
        self.SV_GUI.geometry('1300x900')
        self.SV_GUI.title("SupservisorPC_UI")
        self.modeSelect = tk.IntVar()
        self.logOnOff = tk.IntVar()
        self.modeSelect.set(1)
        self.logOnOff.set(1)

        
        self.UI_opt_steer = tk.DoubleVar()
        self.UI_opt_steer.set(self.optSteering)
        
        self.UI_stop = tk.DoubleVar()
        self.UI_stop.set(1)

        self.UI_vel = tk.DoubleVar()
        self.UI_vel.set(self.vel)

        self.UI_euler_X = tk.DoubleVar()
        self.UI_euler_X.set(self.drivingStatus)

        self.UI_euler_Y = tk.DoubleVar()
        self.UI_euler_Y.set(self.euler_Y)
        
        self.UI_euler_Z = tk.DoubleVar()
        self.UI_euler_Z.set(self.euler_Z)

        self.UI_LeftCyl = tk.DoubleVar()
        self.UI_LeftCyl.set(self.LeftCyl)

        self.UI_RightCyl = tk.DoubleVar()
        self.UI_RightCyl.set(self.RightCyl)

        self.UI_SideCyl = tk.DoubleVar()
        self.UI_SideCyl.set(self.SideCyl)

        self.UI_TiltCyl = tk.DoubleVar()
        self.UI_TiltCyl.set(self.TiltCyl)       

        self.UI_BladeRotation = tk.DoubleVar()
        self.UI_BladeRotation.set(self.BladeRotation)          

        self.UI_BldSideShift = tk.DoubleVar()
        self.UI_BldSideShift.set(self.BldSideShift)

        self.UI_LeftCylPres = tk.DoubleVar()
        self.UI_LeftCylPres.set(self.LeftCylPres)      
        
        self.UI_RightCylPres = tk.DoubleVar()
        self.UI_RightCylPres.set(self.RightCylPres)  

        self.UI_SideCylPres = tk.DoubleVar()
        self.UI_SideCylPres.set(self.SideCylPres)  

        self.UI_TiltCylPres = tk.DoubleVar()
        self.UI_TiltCylPres.set(self.TiltCylPres)  

        self.UI_IMU1_Roll = tk.DoubleVar()
        self.UI_IMU1_Roll.set(self.calibratedRoll)

        self.UI_IMU1_Pitch = tk.DoubleVar()
        self.UI_IMU1_Pitch.set(self.calibratedPitch)

        self.UI_HeadingAngle = tk.DoubleVar()
        self.UI_HeadingAngle.set(self.HeadingAngle)
        self.UI_steeringAngle = tk.DoubleVar()
        self.UI_steeringAngle.set(self.steeringAngle)

        self.UI_IMU2_Roll = tk.DoubleVar()
        self.UI_IMU2_Roll.set(self.IMU2_Roll)

        self.UI_IMU2_Pitch = tk.DoubleVar()
        self.UI_IMU2_Pitch.set(self.IMU2_Pitch)

        self.UI_latitude = tk.DoubleVar()
        self.UI_latitude.set(self.latitude)

        self.UI_longitude = tk.DoubleVar()
        self.UI_longitude.set(self.longitude)
        
        self.UI_altitude = tk.DoubleVar()
        self.UI_altitude.set(self.altitude)
        Fr = []

        # Frame 생성                                                                                                        
        Fr.append(ttk.LabelFrame(self.SV_GUI, width=600, height=1000, text='MG_Data_Blade',relief=tk.SOLID,labelanchor='n'))         # 데이터
        Fr[0].grid(column=0, row=0, padx=10, pady=10, sticky='w', rowspan=2)                                                                          # padx / pady 외부여백

        Fr.append(ttk.LabelFrame(self.SV_GUI,text='Joint Independent Control',relief=tk.SOLID,labelanchor='n'))
        Fr[1].grid(column=1, row=0, padx=10, pady=10, sticky='w')    

        Fr.append(ttk.LabelFrame(self.SV_GUI,text='Integrated Control',relief=tk.SOLID,labelanchor='n'))
        Fr[2].grid(column=1, row=1, padx=10, pady=0, sticky='w', columnspan = 2) 
        
        Fr.append(ttk.LabelFrame(self.SV_GUI,text='Mode Select',relief=tk.SOLID,labelanchor='n'))
        Fr[3].grid(column=1, row=2, padx=80, pady=0, sticky='w') 
        
        Fr.append(ttk.LabelFrame(self.SV_GUI, text='MG_Data_Drive',relief=tk.SOLID,labelanchor='n'))
        Fr[4].grid(column=0, row=2, padx=10, pady=10, sticky='w') 
        
        # 로깅할 때 같이 로깅되도록 설계
        Fr.append(ttk.LabelFrame(self.SV_GUI,text='MPC data',relief=tk.SOLID,labelanchor='n'))
        Fr[5].grid(column=2, row=0, padx=10, pady=10, sticky='w') 

        self.Rcv4 = ttk.Label(Fr[0],text = 'Left Cylinder',width = 30)
        self.Rcv4.grid(column=0, row=3, sticky='w', padx=10, pady=5)    
        self.RcvMsg4 = ttk.Label(Fr[0],textvariable = self.UI_LeftCyl, wraplength=500,width = 10)
        self.RcvMsg4.grid(column=1, row=3, sticky='e', padx=20, pady=5)

        self.Rcv5 = ttk.Label(Fr[0],text = 'Right Cylinder',width = 30)
        self.Rcv5.grid(column=0, row=4, sticky='w', padx=10, pady=5)    
        self.RcvMsg5 = ttk.Label(Fr[0],textvariable = self.UI_RightCyl, wraplength=500,width = 10)
        self.RcvMsg5.grid(column=1, row=4, sticky='e', padx=20, pady=5)

        self.Rcv6 = ttk.Label(Fr[0],text = 'Side Shift Cylinder',width = 30)
        self.Rcv6.grid(column=0, row=5, sticky='w', padx=10, pady=5)    
        self.RcvMsg6 = ttk.Label(Fr[0],textvariable = self.UI_SideCyl, wraplength=500,width = 10)
        self.RcvMsg6.grid(column=1, row=5, sticky='e', padx=20, pady=5)

        self.Rcv7 = ttk.Label(Fr[0],text = 'Tilt Cylinder',width = 30)
        self.Rcv7.grid(column=0, row=6, sticky='w', padx=10, pady=5)    
        self.RcvMsg7 = ttk.Label(Fr[0],textvariable = self.UI_TiltCyl, wraplength=500,width = 10)
        self.RcvMsg7.grid(column=1, row=6, sticky='e', padx=20, pady=5)

        self.Rcv8 = ttk.Label(Fr[0],text = 'Blade Rotation',width = 30)
        self.Rcv8.grid(column=0, row=7, sticky='w', padx=10, pady=5)    
        self.RcvMsg8 = ttk.Label(Fr[0],textvariable = self.UI_BladeRotation, wraplength=500,width = 10)
        self.RcvMsg8.grid(column=1, row=7, sticky='e', padx=20, pady=5)        

        self.Rcv9 = ttk.Label(Fr[0],text = 'Blade Shift Cylinder',width = 30)
        self.Rcv9.grid(column=0, row=8, sticky='w', padx=10, pady=5)    
        self.RcvMsg9 = ttk.Label(Fr[0],textvariable = self.UI_BldSideShift, wraplength=500,width = 10)
        self.RcvMsg9.grid(column=1, row=8, sticky='e', padx=20, pady=5)   

        self.Rcv10 = ttk.Label(Fr[0],text = 'Left Cylinder Pressure',width = 30)
        self.Rcv10.grid(column=0, row=9, sticky='w', padx=10, pady=5)    
        self.RcvMsg10 = ttk.Label(Fr[0],textvariable = self.UI_LeftCylPres, wraplength=500,width = 10)
        self.RcvMsg10.grid(column=1, row=9, sticky='e', padx=20, pady=5)  

        self.Rcv11 = ttk.Label(Fr[0],text = 'Right Cylinder Pressure',width = 30)
        self.Rcv11.grid(column=0, row=10, sticky='w', padx=10, pady=5)    
        self.RcvMsg11 = ttk.Label(Fr[0],textvariable = self.UI_RightCylPres, wraplength=500,width = 10)
        self.RcvMsg11.grid(column=1, row=10, sticky='e', padx=20, pady=5)   

        self.Rcv12 = ttk.Label(Fr[0],text = 'Side Shift Cylinder Pressure',width = 30)
        self.Rcv12.grid(column=0, row=11, sticky='w', padx=10, pady=5)    
        self.RcvMsg12 = ttk.Label(Fr[0],textvariable = self.UI_SideCylPres, wraplength=500,width = 10)
        self.RcvMsg12.grid(column=1, row=11, sticky='e', padx=20, pady=5)   

        self.Rcv13 = ttk.Label(Fr[0],text = 'Tilt Cylinder Pressure',width = 30)
        self.Rcv13.grid(column=0, row=12, sticky='w', padx=10, pady=5)    
        self.RcvMsg13 = ttk.Label(Fr[0],textvariable = self.UI_TiltCylPres, wraplength=500,width = 10)
        self.RcvMsg13.grid(column=1, row=12, sticky='e', padx=20, pady=5)  
        
        self.Rcv14 = ttk.Label(Fr[0],text = 'IMU1 Roll',width = 30)
        self.Rcv14.grid(column=0, row=13, sticky='w', padx=10, pady=5)    
        self.RcvMsg14 = ttk.Label(Fr[0],textvariable = self.UI_IMU1_Roll, wraplength=500,width = 10)
        self.RcvMsg14.grid(column=1, row=13, sticky='e', padx=20, pady=5)   

        self.Rcv15 = ttk.Label(Fr[0],text = 'IMU1 Pitch',width = 30)
        self.Rcv15.grid(column=0, row=14, sticky='w', padx=10, pady=5)    
        self.RcvMsg15 = ttk.Label(Fr[0],textvariable = self.UI_IMU1_Pitch, wraplength=500,width = 10)
        self.RcvMsg15.grid(column=1, row=14, sticky='e', padx=20, pady=5)       

        self.Rcv16 = ttk.Label(Fr[0],text = 'IMU2 Roll',width = 30)
        self.Rcv16.grid(column=0, row=15, sticky='w', padx=10, pady=5)    
        self.RcvMsg16 = ttk.Label(Fr[0],textvariable = self.UI_IMU2_Roll, wraplength=500,width = 10)
        self.RcvMsg16.grid(column=1, row=15, sticky='e', padx=20, pady=5)   

        self.Rcv17 = ttk.Label(Fr[0],text = 'IMU2 Pitch',width = 30)
        self.Rcv17.grid(column=0, row=16, sticky='w', padx=10, pady=5)    
        self.RcvMsg17 = ttk.Label(Fr[0],textvariable = self.UI_IMU2_Pitch, wraplength=500,width = 10)
        self.RcvMsg17.grid(column=1, row=16, sticky='e', padx=20, pady=5)    
        
        self.Rcv23 = ttk.Label(Fr[0],text = 'Driving Status',width = 30)
        self.Rcv23.grid(column=0, row=17, sticky='w', padx=10, pady=5)    
        self.RcvMsg23 = ttk.Label(Fr[0],textvariable = self.UI_euler_X, wraplength=500,width = 10)
        self.RcvMsg23.grid(column=1, row=17, sticky='e', padx=20, pady=5)       

        self.Rcv24 = ttk.Label(Fr[0],text = 'MTi Pitch',width = 30)
        self.Rcv24.grid(column=0, row=18, sticky='w', padx=10, pady=5)    
        self.RcvMsg24 = ttk.Label(Fr[0],textvariable = self.UI_euler_Y, wraplength=500,width = 10)
        self.RcvMsg24.grid(column=1, row=18, sticky='e', padx=20, pady=5)   

        self.Rcv25 = ttk.Label(Fr[0],text = 'MTi Yaw',width = 30)
        self.Rcv25.grid(column=0, row=19, sticky='w', padx=10, pady=5)    
        self.RcvMsg25 = ttk.Label(Fr[0],textvariable = self.UI_euler_Z, wraplength=500,width = 10)
        self.RcvMsg25.grid(column=1, row=19, sticky='e', padx=20, pady=5)    
        
        self.Rcv1 = ttk.Label(Fr[4],text = 'Velocity',width = 30)
        self.Rcv1.grid(column=0, row=0, sticky='w', padx=10, pady=5)    
        self.RcvMsg1 = ttk.Label(Fr[4], textvariable = self.UI_vel, wraplength=500,width = 10)
        self.RcvMsg1.grid(column=1, row=0, sticky='e', padx=20, pady=5)

        self.Rcv21 = ttk.Label(Fr[4],text = 'Heading Angle',width = 30)
        self.Rcv21.grid(column=0, row=4, sticky='w', padx=10, pady=5)    
        self.RcvMsg21 = ttk.Label(Fr[4], textvariable = self.UI_HeadingAngle, wraplength=500,width = 10)
        self.RcvMsg21.grid(column=1, row=4, sticky='e', padx=20, pady=5)
        
        self.Rcv22 = ttk.Label(Fr[4],text = 'Steering Angle',width = 30)
        self.Rcv22.grid(column=0, row=5, sticky='w', padx=10, pady=5)    
        self.RcvMsg22 = ttk.Label(Fr[4], textvariable = self.UI_steeringAngle, wraplength=500,width = 10)
        self.RcvMsg22.grid(column=1, row=5, sticky='e', padx=20, pady=5)

        self.Rcv18 = ttk.Label(Fr[4],text = 'Latitude',width = 30)
        self.Rcv18.grid(column=0, row=1, sticky='w', padx=10, pady=5)    
        self.RcvMsg18 = ttk.Label(Fr[4],textvariable = self.UI_latitude, wraplength=500,width = 10)
        self.RcvMsg18.grid(column=1, row=1, sticky='e', padx=20, pady=5)  
                
        self.Rcv19 = ttk.Label(Fr[4],text = 'Longitude',width = 30)
        self.Rcv19.grid(column=0, row=2, sticky='w', padx=10, pady=5)    
        self.RcvMsg19 = ttk.Label(Fr[4],textvariable = self.UI_longitude, wraplength=500,width = 10)
        self.RcvMsg19.grid(column=1, row=2, sticky='e', padx=20, pady=5)   

        
        self.Rcv20 = ttk.Label(Fr[4],text = 'Altitude',width = 30)
        self.Rcv20.grid(column=0, row=3, sticky='w', padx=10, pady=5)    
        self.RcvMsg20 = ttk.Label(Fr[4],textvariable = self.UI_altitude, wraplength=500,width = 10)
        self.RcvMsg20.grid(column=1, row=3, sticky='e', padx=20, pady=5)   


        self.Snd1 = ttk.Label(Fr[1],text = 'Left Cylinder')
        self.Snd1.grid(column=0, row=0, sticky='w', padx=10, pady=5)    
        self.SndMsg1 = ttk.Entry(Fr[1])
        self.SndMsg1.grid(column=1, row=0, sticky='e', padx=20, pady=5)   
        self.SndBtn1 = ttk.Button(Fr[1],text = "입력",command=self.leftCylControl)
        self.SndBtn1.grid(column=2, row=0, sticky='ne', padx=20, pady=5)   

        self.Snd2 = ttk.Label(Fr[1],text = 'Right Cylinder')
        self.Snd2.grid(column=0, row=1, sticky='w', padx=10, pady=5)    
        self.SndMsg2 = ttk.Entry(Fr[1])
        self.SndMsg2.grid(column=1, row=1, sticky='e', padx=20, pady=5)   
        self.SndBtn2 = ttk.Button(Fr[1],text = "입력",command=self.rightCylControl)
        self.SndBtn2.grid(column=2, row=1, sticky='ne', padx=20, pady=5)  

        self.Snd3 = ttk.Label(Fr[1],text = 'Circle Side Shift Cylinder')
        self.Snd3.grid(column=0, row=2, sticky='w', padx=10, pady=5)    
        self.SndMsg3 = ttk.Entry(Fr[1])
        self.SndMsg3.grid(column=1, row=2, sticky='e', padx=20, pady=5)   
        self.SndBtn3 = ttk.Button(Fr[1],text = "입력",command=self.sideCylControl)
        self.SndBtn3.grid(column=2, row=2, sticky='ne', padx=20, pady=5)  

        self.Snd4 = ttk.Label(Fr[1],text = 'Tilt Cylinder')
        self.Snd4.grid(column=0, row=3, sticky='w', padx=10, pady=5)    
        
        self.SndMsg4 = ttk.Entry(Fr[1])
        self.SndMsg4.grid(column=1, row=3, sticky='e', padx=20, pady=5)   
        self.SndBtn4 = ttk.Button(Fr[1],text = "입력",command=self.tiltCylControl)
        self.SndBtn4.grid(column=2, row=3, sticky='ne', padx=20, pady=5)  
        
        self.Snd5 = ttk.Label(Fr[1],text = 'Blade Rotation')
        self.Snd5.grid(column=0, row=4, sticky='w', padx=10, pady=5)    
        
        self.SndMsg5 = ttk.Entry(Fr[1])
        self.SndMsg5.grid(column=1, row=4, sticky='e', padx=20, pady=5)   
        self.SndBtn5 = ttk.Button(Fr[1],text = "입력",command=self.bladeRotControl)
        self.SndBtn5.grid(column=2, row=4, sticky='ne', padx=20, pady=5)  
   
        self.Snd6 = ttk.Label(Fr[1],text = 'Blade Side Shift Cylinder')
        self.Snd6.grid(column=0, row=5, sticky='w', padx=10, pady=5)    
        
        self.SndMsg6 = ttk.Entry(Fr[1])
        self.SndMsg6.grid(column=1, row=5, sticky='e', padx=20, pady=5)   
        self.SndBtn6 = ttk.Button(Fr[1],text = "입력",command=self.bladeSideshiftControl)
        self.SndBtn6.grid(column=2, row=5, sticky='ne', padx=20, pady=5)  
   
   
        self.Snd7 = ttk.Label(Fr[2],text = 'Blade height')
        self.Snd7.grid(column=0, row=0, sticky='w', padx=10, pady=5)    
        self.SndMsg7 = ttk.Entry(Fr[2])
        self.SndMsg7.grid(column=1, row=0, sticky='e', padx=20, pady=5)   
        
        self.Snd8 = ttk.Label(Fr[2],text = 'Blade Cross Section Angle')
        self.Snd8.grid(column=0, row=1, sticky='w', padx=10, pady=5)    
        self.SndMsg8 = ttk.Entry(Fr[2])
        self.SndMsg8.grid(column=1, row=1, sticky='e', padx=20, pady=5)   
        
        self.Snd9 = ttk.Label(Fr[2],text = 'Blade Rotation Angle')
        self.Snd9.grid(column=0, row=2, sticky='w', padx=10, pady=5)    
        self.SndMsg9 = ttk.Entry(Fr[2])
        self.SndMsg9.grid(column=1, row=2, sticky='e', padx=20, pady=5)  
        
        self.Snd10 = ttk.Label(Fr[2],text = 'Blade Tilt Angle')
        self.Snd10.grid(column=0, row=3, sticky='w', padx=10, pady=5)    
        self.SndMsg10 = ttk.Entry(Fr[2])
        self.SndMsg10.grid(column=1, row=3, sticky='e', padx=20, pady=5)  
        
        self.Snd14 = ttk.Label(Fr[2],text = 'Blade Side shift')
        self.Snd14.grid(column=0, row=4, sticky='w', padx=10, pady=5)    
        self.SndMsg14 = ttk.Entry(Fr[2])
        self.SndMsg14.grid(column=1, row=4, sticky='e', padx=20, pady=5)         
        
        
        self.SndBtn10 = ttk.Button(Fr[2],text = "차량 좌표계 기준 명령",command=self.highLevelControl)
        self.SndBtn10.grid(column=2, row=3, sticky='ne', padx=20, pady=5)  
   
        self.SndBtn10 = ttk.Button(Fr[2],text = "전역 좌표계 기준 입력",command=self.highLevelControl_Global)
        self.SndBtn10.grid(column=2, row=4, sticky='ne', padx=20, pady=5)
        
        self.SndBtnUp = ttk.Button(Fr[2],text = "Up",command=self.heightUp)
        self.SndBtnUp.grid(column=2, row=1, sticky='ne', padx=20, pady=5)
        
        self.SndBtnDown = ttk.Button(Fr[2],text = "Down",command=self.heightDown)
        self.SndBtnDown.grid(column=2, row=2, sticky='ne', padx=20, pady=5)
        
   
        self.Snd11 = ttk.Label(Fr[2],text = 'Roll : -7~7')
        self.Snd11.grid(column=3, row=1, sticky='w', padx=10, pady=5)    
        self.SndMsg11 = ttk.Entry(Fr[2])
        self.SndMsg11.grid(column=4, row=1, sticky='e', padx=20, pady=5)   
        
        self.Snd12 = ttk.Label(Fr[2],text = 'Pitch : -1~4')
        self.Snd12.grid(column=3, row=2, sticky='w', padx=10, pady=5)    
        self.SndMsg12 = ttk.Entry(Fr[2])
        self.SndMsg12.grid(column=4, row=2, sticky='e', padx=20, pady=5)   
        
        self.Snd13 = ttk.Label(Fr[2],text = 'Yaw : 0~7')
        self.Snd13.grid(column=3, row=3, sticky='w', padx=10, pady=5)    
        self.SndMsg13 = ttk.Entry(Fr[2])
        self.SndMsg13.grid(column=4, row=3, sticky='e', padx=20, pady=5)  

        self.SndBtn13 = ttk.Button(Fr[2],text = "입력",command=self.RPYcontrol)
        self.SndBtn13.grid(column=5, row=3, sticky='ne', padx=20, pady=5)  
        
        # self.Snd15 = ttk.Label(Fr[2],text = 'Goal steering value')
        # self.Snd15.grid(column=0, row=0, sticky='w', padx=10, pady=5)  
   

   
        self.rad1 = ttk.Radiobutton(Fr[3],text = 'UI mode', variable = self.modeSelect, value = 1, command=self.modeFunction)
        self.rad1.grid(column=0, row=0, sticky='w', padx=10, pady=5)   
        
        self.rad2 = ttk.Radiobutton(Fr[3],text = 'Manual', variable = self.modeSelect, value = 0, command=self.modeFunction)
        self.rad2.grid(column=0, row=1, sticky='w', padx=10, pady=5)
           
        self.btnStop = ttk.Button(Fr[3],text = '초기 상태 복귀',command=self.getToInitial)
        self.btnStop.grid(column=2, row=0, sticky='w')
        
        self.btnStop = ttk.Button(Fr[3],text = '비상 정지',command=self.stopUI)
        self.btnStop.grid(column=2, row=1, sticky='w')
        
        self.btnLogging1 = ttk.Radiobutton(Fr[3],text = 'logging on', value = 0, variable = self.logOnOff, command=self.loggingFn)
        self.btnLogging1.grid(column=1, row=0, sticky='w', padx=10, pady=5) 
           
        self.btnLogging2 = ttk.Radiobutton(Fr[3],text = 'logging off', value = 1, variable = self.logOnOff, command=self.loggingFn)
        self.btnLogging2.grid(column=1, row=1, sticky='w', padx=10, pady=5)  


        self.btnResetIMUYaw = ttk.Button(Fr[3],text = 'Yaw value 0', command=self.setIMUYaw0)
        self.btnResetIMUYaw.grid(column=3, row=0, sticky='w', padx=10, pady=5) 
        
        self.btnGetInitialRef = ttk.Button(Fr[3],text = '기준 위치로 이동', command=self.getInitialRef)
        self.btnGetInitialRef.grid(column=3, row=1, sticky='w', padx=10, pady=5) 
        
        
        self.btnSeq1 = ttk.Button(Fr[3],text = 'Ramp input', command=self.seqRamp)
        self.btnSeq1.grid(column=4, row=0, sticky='w', padx=10, pady=5)
        
        self.btnSeq2 = ttk.Button(Fr[3],text = 'Sinusoidal input', command=self.seqSin)
        self.btnSeq2.grid(column=4, row=1, sticky='w', padx=10, pady=5)
        
        
        self.MPCOnOff = ttk.Button(Fr[5],text = 'MPC 실행', command=self.MPCswitch)
        self.MPCOnOff.grid(column=0, row=1, sticky='w', padx=10, pady=5)
        
        self.RcvOpt = ttk.Label(Fr[5],text = 'Optimal steering',width = 30)
        self.RcvOpt.grid(column=1, row=0, sticky='w', padx=10, pady=5)    
        self.RcvMsgOpt = ttk.Label(Fr[5],textvariable = self.UI_opt_steer, wraplength=500,width = 10)
        self.RcvMsgOpt.grid(column=1, row=1, sticky='e', padx=20, pady=5)  
        
        self.SteerLabel = ttk.Label(Fr[5],text = 'Steer input')
        self.SteerLabel.grid(column=0, row=2, sticky='w', padx=10, pady=5)    
        self.SteerMsg = ttk.Entry(Fr[5])
        self.SteerMsg.grid(column=1, row=2, sticky='e', padx=20, pady=5)   
        self.SteerBtn = ttk.Button(Fr[5],text = "입력",command=self.steerInput)
        self.SteerBtn.grid(column=2, row=2, sticky='ne', padx=20, pady=5)    
 
        

    def loggingFn(self):
        if self.logOnOff.get() == 0:       # logging on
            # self.CMDB[5] = 1
            # self.SVP_client_socket.sendall(self.CMDB)            
            self.current_date = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
            # self.logtime = time.time()
            self.logtime = time.perf_counter()
            self.csv_file_name = f'logfile/data_logging_{self.current_date}.csv'
            print(self.csv_file_name)
            self.file = open(self.csv_file_name,'w',newline='')
            self.writer = csv.writer(self.file,delimiter=',')  
            self.rows = []
            self.rows.append(['time','LeftCyl','RightCyl','SideCyl','Blade rotation','BldSideShift','TiltCyl','euler_X','euler_Y','euler_Z','IMU1_Roll','IMU1_Pitch','IMU2_Roll','IMU2_Pitch', 'LeftCylPres1', 'RightCylPres1', 'SideCylPres1', 'TiltCylPres1', 'LeftCylPres2', 'RightCylPres2', 'SideCylPres2', 'TiltCylPres2','vel', 'latitude','longitude','altitude','heading','Steering Angle'])
            self.writer.writerows(self.rows)
            print('Logging set')
            
            self.logStatus = 1
            
        else:
            self.file.close()
            self.logStatus = 0
            # self.CMDB[5] = 0
            # self.SVP_client_socket.sendall(self.CMDB)

    def leftCylControl(self):
        leftCylOrder = self.SndMsg1.get()       # [mm] command
        leftCylOrder = int(float(leftCylOrder)*10)
        temp = struct.pack('>h',leftCylOrder)
        self.CMDB[10] = temp[0]
        self.CMDB[11] = temp[1]
        # self.SVP_client_socket.sendall(self.CMDD)
        self.SVP_client_socket.sendall(self.CMDB)
        print(leftCylOrder)
        
    def rightCylControl(self):
        rightCylOrder = self.SndMsg2.get()
        rightCylOrder = int(float(rightCylOrder)*10)
        temp = struct.pack('>h',rightCylOrder)
        self.CMDB[12] = temp[0]
        self.CMDB[13] = temp[1]
        # self.SVP_client_socket.sendall(self.CMDD)
        self.SVP_client_socket.sendall(self.CMDB)
        print(rightCylOrder)
        
    def sideCylControl(self):
        sideCylOrder = self.SndMsg3.get()
        sideCylOrder = int(float(sideCylOrder)*10)
        temp = struct.pack('>h',sideCylOrder)
        self.CMDB[14] = temp[0]
        self.CMDB[15] = temp[1]
        # self.SVP_client_socket.sendall(self.CMDD)
        self.SVP_client_socket.sendall(self.CMDB)
        print(sideCylOrder)
        
    def tiltCylControl(self):
        tiltCylOrder = self.SndMsg4.get()
        tiltCylOrder = int(float(tiltCylOrder)*10)
        print(tiltCylOrder)
        temp = struct.pack('>h',tiltCylOrder)
        self.CMDB[18] = temp[0]
        self.CMDB[19] = temp[1]
        # self.SVP_client_socket.sendall(self.CMDD)
        self.SVP_client_socket.sendall(self.CMDB)  
        # print(self.CMDB)   
        
    def bladeRotControl(self):
        bladeRotOrder = self.SndMsg5.get()
        bladeRotOrder = int(-float(bladeRotOrder)*10)
        temp = struct.pack('>h',bladeRotOrder)
        self.CMDB[8] = temp[0]
        self.CMDB[9] = temp[1]
        # self.SVP_client_socket.sendall(self.CMDD)
        self.SVP_client_socket.sendall(self.CMDB)   
        print(bladeRotOrder)
        
    def bladeSideshiftControl(self):
        
        bladeSideshiftOrder = self.SndMsg6.get()
        bladeSideshiftOrder = int(float(bladeSideshiftOrder)*10)
        temp = struct.pack('>h',bladeSideshiftOrder)
        self.CMDB[16] = temp[0]
        self.CMDB[17] = temp[1]
        # self.SVP_client_socket.sendall(self.CMDD)
        self.SVP_client_socket.sendall(self.CMDB)   
        print(bladeSideshiftOrder)
        
    def seqRamp(self):
        
        for i in range(100):
            tim = time.time()
            self.thread_lock.acquire()
            self.Height = -1 + 0.002*i             # 지면 기준
            print(self.Height)
            self.CrossSectionAngle = 0   # 정면 각도 [deg]
            self.RotationAngle = 0
            self.Tiltangle = 60      # 60~90 : -30~0
            
            # Initial guess for pitch and roll (can be any reasonable values)
            initial_guess = (0.0, 0.0)

            # Use fsolve to find the solutions for pitch and roll
            if self.Tiltangle > 90: self.Tiltangle = 90
            elif self.Tiltangle<60: self.Tiltangle = 60
            
            tiltCylOrder = int(-1800/30*(self.Tiltangle-60)+200)
            self.Tiltangle = (self.Tiltangle-116)*np.pi/180     
            
            solutions = fsolve(func=lambda x: [crossAngle(x[0],x[1],self.CrossSectionAngle,self.RotationAngle), desiredHeight(x[0],x[1],self.Height,self.RotationAngle,self.Tiltangle)], x0=initial_guess)
            pitch_solution, roll_solution = solutions
            print(pitch_solution)
            roll = roll_solution *180/np.pi
            pitch = pitch_solution *180/np.pi
            print(f'desired roll : {roll}, desired pitch : {pitch}')
            yaw_solution = yaw
            
            Reul = rotz(yaw_solution)@roty(pitch_solution)@rotx(roll_solution)
            
            # l_ab = 0.100
            # l_c = 0.100     # 실린더 길이 (변위를 받아봐야 함)
            # l_b = 0.100

            tb1 = Reul@b1
            tb2 = Reul@b2
            tb3 = Reul@b3

            # Inverse kinematics
            d1 = np.sqrt(a1.T@a1 + tb1.T@tb1 - 2*a1.T@tb1)
            d2 = np.sqrt(a2.T@a2 + tb2.T@tb2 - 2*a2.T@tb2)
            d3 = np.sqrt(a3.T@a3 + tb3.T@tb3 - 2*a3.T@tb3)
            print(f'd1 : {d1}, d2 : {d2}, d3 : {d3},')
            
            # 실린더 변위값
            leftCylOrder = -(d1-d1_0)*10000
            rightCylOrder = -(d2-d2_0)*10000
            sideCylOrder = -(d3-d3_0)*10000
            # 임시 : 매핑한 값
            if leftCylOrder<-3300 or rightCylOrder<-3300:
                return
            
            if leftCylOrder>10 or rightCylOrder>10:
                return
            
            # Mapping하는 식은 잘 모르겠음

            bladeRotOrder = -self.RotationAngle*10/np.pi*180
            bladeSideshiftOrder = 0
            print(leftCylOrder,rightCylOrder,sideCylOrder,tiltCylOrder,bladeRotOrder,bladeSideshiftOrder)
            
            # leftCylOrder
            temp = struct.pack('>h',int(leftCylOrder))
            self.CMDB[10] = temp[0]
            self.CMDB[11] = temp[1]
            # rightCylOrder
            temp = struct.pack('>h',int(rightCylOrder))
            self.CMDB[12] = temp[0]
            self.CMDB[13] = temp[1]    
            # sideCylOrder
            temp = struct.pack('>h',int(sideCylOrder))
            self.CMDB[14] = temp[0]
            self.CMDB[15] = temp[1]
            # tiltCylOrder
            temp = struct.pack('>h',int(tiltCylOrder))
            self.CMDB[18] = temp[0]
            self.CMDB[19] = temp[1]
            # bladeRotOrder
            temp = struct.pack('>h',int(bladeRotOrder))
            self.CMDB[8] = temp[0]
            self.CMDB[9] = temp[1]
            # bladeSideshiftOrder
            temp = struct.pack('>h',int(bladeSideshiftOrder))
            self.CMDB[16] = temp[0]
            self.CMDB[17] = temp[1]
            
            # print(leftCylOrder,rightCylOrder,sideCylOrder)
            # print(self.CMDB)
            # self.SVP_client_socket.sendall(self.CMDD)
            self.SVP_client_socket.sendall(self.CMDB)
            self.thread_lock.release()
            elp_tim = time.time()
            time.sleep(0.2-elp_tim+tim)
        
    def seqSin(self):
        print('1')
        for i in range(100):
            tim = time.time()
            self.thread_lock.acquire()
            self.Height = -0.9 + 0.1*np.sin(2*np.pi*i/30)             # 지면 기준
            print(self.Height)
            self.CrossSectionAngle = 0   # 정면 각도 [deg]
            self.RotationAngle = 0
            self.Tiltangle = 60      # 60~90 : -30~0
            
            # Initial guess for pitch and roll (can be any reasonable values)
            initial_guess = (0.0, 0.0)

            # Use fsolve to find the solutions for pitch and roll
            if self.Tiltangle > 90: self.Tiltangle = 90
            elif self.Tiltangle<60: self.Tiltangle = 60
            
            tiltCylOrder = int(-1800/30*(self.Tiltangle-60)+200)
            self.Tiltangle = (self.Tiltangle-116)*np.pi/180     
            
            solutions = fsolve(func=lambda x: [crossAngle(x[0],x[1],self.CrossSectionAngle,self.RotationAngle), desiredHeight(x[0],x[1],self.Height,self.RotationAngle,self.Tiltangle)], x0=initial_guess)
            pitch_solution, roll_solution = solutions
            print(pitch_solution)
            roll = roll_solution *180/np.pi
            pitch = pitch_solution *180/np.pi
            print(f'desired roll : {roll}, desired pitch : {pitch}')
            yaw_solution = yaw
            
            Reul = rotz(yaw_solution)@roty(pitch_solution)@rotx(roll_solution)
            
            # l_ab = 0.100
            # l_c = 0.100     # 실린더 길이 (변위를 받아봐야 함)
            # l_b = 0.100

            tb1 = Reul@b1
            tb2 = Reul@b2
            tb3 = Reul@b3

            # Inverse kinematics
            d1 = np.sqrt(a1.T@a1 + tb1.T@tb1 - 2*a1.T@tb1)
            d2 = np.sqrt(a2.T@a2 + tb2.T@tb2 - 2*a2.T@tb2)
            d3 = np.sqrt(a3.T@a3 + tb3.T@tb3 - 2*a3.T@tb3)
            print(f'd1 : {d1}, d2 : {d2}, d3 : {d3},')
            
            # 실린더 변위값
            leftCylOrder = -(d1-d1_0)*10000
            rightCylOrder = -(d2-d2_0)*10000
            sideCylOrder = -(d3-d3_0)*10000
            # 임시 : 매핑한 값
            if leftCylOrder<-3300 or rightCylOrder<-3300:
                return
            
            if leftCylOrder>10 or rightCylOrder>10:
                return
            
            
            
            # Mapping하는 식은 잘 모르겠음

            bladeRotOrder = -self.RotationAngle*10/np.pi*180
            bladeSideshiftOrder = 0
            print(leftCylOrder,rightCylOrder,sideCylOrder,tiltCylOrder,bladeRotOrder,bladeSideshiftOrder)
            
            # leftCylOrder
            temp = struct.pack('>h',int(leftCylOrder))
            self.CMDB[10] = temp[0]
            self.CMDB[11] = temp[1]
            # rightCylOrder
            temp = struct.pack('>h',int(rightCylOrder))
            self.CMDB[12] = temp[0]
            self.CMDB[13] = temp[1]    
            # sideCylOrder
            temp = struct.pack('>h',int(sideCylOrder))
            self.CMDB[14] = temp[0]
            self.CMDB[15] = temp[1]
            # tiltCylOrder
            temp = struct.pack('>h',int(tiltCylOrder))
            self.CMDB[18] = temp[0]
            self.CMDB[19] = temp[1]
            # bladeRotOrder
            temp = struct.pack('>h',int(bladeRotOrder))
            self.CMDB[8] = temp[0]
            self.CMDB[9] = temp[1]
            # bladeSideshiftOrder
            temp = struct.pack('>h',int(bladeSideshiftOrder))
            self.CMDB[16] = temp[0]
            self.CMDB[17] = temp[1]
            
            # print(leftCylOrder,rightCylOrder,sideCylOrder)
            # print(self.CMDB)
            # self.SVP_client_socket.sendall(self.CMDD)
            self.SVP_client_socket.sendall(self.CMDB)
            self.thread_lock.release()
            elp_tim = time.time()
            time.sleep(0.1)

    def heightDown(self):
        
        self.Height = self.Height-0.05
        initial_guess = (0.0, 0.0)

        # Use fsolve to find the solutions for pitch and roll
        if self.Tiltangle > 90: self.Tiltangle = 90
        elif self.Tiltangle<60: self.Tiltangle = 60
        
        tiltCylOrder = int(-1800/30*(self.Tiltangle-60)+200)
        self.Tiltangle = (self.Tiltangle-116)*np.pi/180     
        
        solutions = fsolve(func=lambda x: [crossAngle(x[0],x[1],self.CrossSectionAngle,self.RotationAngle), desiredHeight(x[0],x[1],self.Height,self.RotationAngle,self.Tiltangle)], x0=initial_guess)
        pitch_solution, roll_solution = solutions
        print(pitch_solution)
        roll = roll_solution *180/np.pi
        pitch = pitch_solution *180/np.pi
        print(f'desired roll : {roll}, desired pitch : {pitch}')
        yaw_solution = yaw
        
        Reul = rotz(yaw_solution)@roty(pitch_solution)@rotx(roll_solution)
        
        # l_ab = 0.100
        # l_c = 0.100     # 실린더 길이 (변위를 받아봐야 함)
        # l_b = 0.100

        tb1 = Reul@b1
        tb2 = Reul@b2
        tb3 = Reul@b3

        # Inverse kinematics
        d1 = np.sqrt(a1.T@a1 + tb1.T@tb1 - 2*a1.T@tb1)
        d2 = np.sqrt(a2.T@a2 + tb2.T@tb2 - 2*a2.T@tb2)
        d3 = np.sqrt(a3.T@a3 + tb3.T@tb3 - 2*a3.T@tb3)
        print(f'd1 : {d1}, d2 : {d2}, d3 : {d3},')
        
        # 실린더 변위값
        leftCylOrder = -(d1-d1_0)*10000
        rightCylOrder = -(d2-d2_0)*10000
        sideCylOrder = -(d3-d3_0)*10000
        # 임시 : 매핑한 값
        if leftCylOrder<-3300 or rightCylOrder<-3300:
            return
        
        if leftCylOrder>10 or rightCylOrder>10:
            return        
        
        
        
        # Mapping하는 식은 잘 모르겠음

        bladeRotOrder = -self.RotationAngle*10/np.pi*180
        bladeSideshiftOrder = 0
        print(leftCylOrder,rightCylOrder,sideCylOrder,tiltCylOrder,bladeRotOrder,bladeSideshiftOrder)
        
        # leftCylOrder
        temp = struct.pack('>h',int(leftCylOrder))
        self.CMDB[10] = temp[0]
        self.CMDB[11] = temp[1]
        # rightCylOrder
        temp = struct.pack('>h',int(rightCylOrder))
        self.CMDB[12] = temp[0]
        self.CMDB[13] = temp[1]    
        # sideCylOrder
        temp = struct.pack('>h',int(sideCylOrder))
        self.CMDB[14] = temp[0]
        self.CMDB[15] = temp[1]
        # tiltCylOrder
        temp = struct.pack('>h',int(tiltCylOrder))
        self.CMDB[18] = temp[0]
        self.CMDB[19] = temp[1]
        # bladeRotOrder
        temp = struct.pack('>h',int(bladeRotOrder))
        self.CMDB[8] = temp[0]
        self.CMDB[9] = temp[1]
        # bladeSideshiftOrder
        temp = struct.pack('>h',int(bladeSideshiftOrder))
        self.CMDB[16] = temp[0]
        self.CMDB[17] = temp[1]
        
        # print(leftCylOrder,rightCylOrder,sideCylOrder)
        # print(self.CMDB)
        # self.SVP_client_socket.sendall(self.CMDD)
        self.SVP_client_socket.sendall(self.CMDB)
        self.thread_lock.release()
        
    
    def heightUp(self):
        
        self.Height = self.Height+0.05
        initial_guess = (0.0, 0.0)

        # Use fsolve to find the solutions for pitch and roll
        if self.Tiltangle > 90: self.Tiltangle = 90
        elif self.Tiltangle<60: self.Tiltangle = 60
        
        tiltCylOrder = int(-1800/30*(self.Tiltangle-60)+200)
        self.Tiltangle = (self.Tiltangle-116)*np.pi/180     
        
        solutions = fsolve(func=lambda x: [crossAngle(x[0],x[1],self.CrossSectionAngle,self.RotationAngle), desiredHeight(x[0],x[1],self.Height,self.RotationAngle,self.Tiltangle)], x0=initial_guess)
        pitch_solution, roll_solution = solutions
        print(pitch_solution)
        roll = roll_solution *180/np.pi
        pitch = pitch_solution *180/np.pi
        print(f'desired roll : {roll}, desired pitch : {pitch}')
        yaw_solution = yaw
        
        Reul = rotz(yaw_solution)@roty(pitch_solution)@rotx(roll_solution)
        
        # l_ab = 0.100
        # l_c = 0.100     # 실린더 길이 (변위를 받아봐야 함)
        # l_b = 0.100

        tb1 = Reul@b1
        tb2 = Reul@b2
        tb3 = Reul@b3

        # Inverse kinematics
        d1 = np.sqrt(a1.T@a1 + tb1.T@tb1 - 2*a1.T@tb1)
        d2 = np.sqrt(a2.T@a2 + tb2.T@tb2 - 2*a2.T@tb2)
        d3 = np.sqrt(a3.T@a3 + tb3.T@tb3 - 2*a3.T@tb3)
        print(f'd1 : {d1}, d2 : {d2}, d3 : {d3},')
        
        # 실린더 변위값
        leftCylOrder = -(d1-d1_0)*10000
        rightCylOrder = -(d2-d2_0)*10000
        sideCylOrder = -(d3-d3_0)*10000
        # 임시 : 매핑한 값
        if leftCylOrder<-3300 or rightCylOrder<-3300:
            return
        
        if leftCylOrder>10 or rightCylOrder>10:
            return          
        
        
        
        # Mapping하는 식은 잘 모르겠음

        bladeRotOrder = -self.RotationAngle*10/np.pi*180
        bladeSideshiftOrder = 0
        print(leftCylOrder,rightCylOrder,sideCylOrder,tiltCylOrder,bladeRotOrder,bladeSideshiftOrder)
        
        # leftCylOrder
        temp = struct.pack('>h',int(leftCylOrder))
        self.CMDB[10] = temp[0]
        self.CMDB[11] = temp[1]
        # rightCylOrder
        temp = struct.pack('>h',int(rightCylOrder))
        self.CMDB[12] = temp[0]
        self.CMDB[13] = temp[1]    
        # sideCylOrder
        temp = struct.pack('>h',int(sideCylOrder))
        self.CMDB[14] = temp[0]
        self.CMDB[15] = temp[1]
        # tiltCylOrder
        temp = struct.pack('>h',int(tiltCylOrder))
        self.CMDB[18] = temp[0]
        self.CMDB[19] = temp[1]
        # bladeRotOrder
        temp = struct.pack('>h',int(bladeRotOrder))
        self.CMDB[8] = temp[0]
        self.CMDB[9] = temp[1]
        # bladeSideshiftOrder
        temp = struct.pack('>h',int(bladeSideshiftOrder))
        self.CMDB[16] = temp[0]
        self.CMDB[17] = temp[1]
        
        # print(leftCylOrder,rightCylOrder,sideCylOrder)
        # print(self.CMDB)
        # self.SVP_client_socket.sendall(self.CMDD)
        self.SVP_client_socket.sendall(self.CMDB)
        self.thread_lock.release()
    
    
    def highLevelControl_Global(self):
        
        IMU1_roll  = np.round(self.calibratedRoll,1)/180*np.pi
        IMU1_pitch = np.round(self.calibratedPitch,1)/180*np.pi
        
        offset_roll = math.atan((np.cos(IMU1_pitch)*np.sin(IMU1_roll))/np.cos(IMU1_roll))
        print(f'offset : {offset_roll}')
  

        # 나중에 단위 변환시켜줘야 함
        self.thread_lock.acquire()
        self.Height = float(self.SndMsg7.get())              # 지면 기준
        self.CrossSectionAngle = float(self.SndMsg8.get())*np.pi/180 - offset_roll  # 정면 각도 [deg]
        self.RotationAngle = float(self.SndMsg9.get())*np.pi/180
        self.Tiltangle = float(self.SndMsg10.get())      # 60~90 : -30~0
        self.Bladeside = float(self.SndMsg14.get())     # mm 단위
        
        
        # Initial guess for pitch and roll (can be any reasonable values)
        initial_guess = (0.0, 0.0)

        # Use fsolve to find the solutions for pitch and roll
        if self.Tiltangle > 90: self.Tiltangle = 90
        elif self.Tiltangle<60: self.Tiltangle = 60
        
        tiltCylOrder = int(-1800/30*(self.Tiltangle-60)+200)
        self.Tiltangle = (self.Tiltangle-116)*np.pi/180     
        
        
        
        solutions = fsolve(func=lambda x: [crossAngle(x[0],x[1],self.CrossSectionAngle,self.RotationAngle), desiredHeight(x[0],x[1],self.Height,self.RotationAngle,self.Tiltangle)], x0=initial_guess)
        pitch_solution, roll_solution = solutions
        print(pitch_solution)
        roll = roll_solution *180/np.pi
        pitch = pitch_solution *180/np.pi
        print(f'desired roll : {roll}, desired pitch : {pitch}')
        yaw_solution = yaw
        
        Reul = rotz(yaw_solution)@roty(pitch_solution)@rotx(roll_solution)
        
        # l_ab = 0.100
        # l_c = 0.100     # 실린더 길이 (변위를 받아봐야 함)
        # l_b = 0.100

        tb1 = Reul@b1
        tb2 = Reul@b2
        tb3 = Reul@b3

        # Inverse kinematics
        d1 = np.sqrt(a1.T@a1 + tb1.T@tb1 - 2*a1.T@tb1)
        d2 = np.sqrt(a2.T@a2 + tb2.T@tb2 - 2*a2.T@tb2)
        d3 = np.sqrt(a3.T@a3 + tb3.T@tb3 - 2*a3.T@tb3)
        print(f'd1 : {d1}, d2 : {d2}, d3 : {d3},')
        
        # 실린더 변위값
        leftCylOrder = -(d1-d1_0)*10000
        rightCylOrder = -(d2-d2_0)*10000
        sideCylOrder = -(d3-d3_0)*10000
        # 임시 : 매핑한 값
        if leftCylOrder<-3300 or rightCylOrder<-3300:
            print('out of range')
            return
        
        if leftCylOrder>10 or rightCylOrder>10:
            print('out of range')
            return        
        
        # Mapping하는 식은 잘 모르겠음

        bladeRotOrder = -self.RotationAngle*10/np.pi*180
        if bladeRotOrder<-400 or bladeRotOrder>400:
            print('Too big blade rotation order')
            return
        
        bladeSideshiftOrder = 0
        print(leftCylOrder,rightCylOrder,sideCylOrder,tiltCylOrder,bladeRotOrder,bladeSideshiftOrder)
        
        # leftCylOrder
        temp = struct.pack('>h',int(leftCylOrder))
        self.CMDB[10] = temp[0]
        self.CMDB[11] = temp[1]
        # rightCylOrder
        temp = struct.pack('>h',int(rightCylOrder))
        self.CMDB[12] = temp[0]
        self.CMDB[13] = temp[1]    
        # sideCylOrder
        temp = struct.pack('>h',int(sideCylOrder))
        self.CMDB[14] = temp[0]
        self.CMDB[15] = temp[1]
        # tiltCylOrder
        temp = struct.pack('>h',int(tiltCylOrder))
        self.CMDB[18] = temp[0]
        self.CMDB[19] = temp[1]
        # bladeRotOrder
        temp = struct.pack('>h',int(bladeRotOrder))
        self.CMDB[8] = temp[0]
        self.CMDB[9] = temp[1]
        # bladeSideshiftOrder
        temp = struct.pack('>h',int(self.Bladeside*10))
        self.CMDB[16] = temp[0]
        self.CMDB[17] = temp[1]
        
        
        # print(leftCylOrder,rightCylOrder,sideCylOrder)
        # print(self.CMDB)
        
        # self.SVP_client_socket.sendall(self.CMDD)
        self.SVP_client_socket.sendall(self.CMDB)
        
        self.thread_lock.release()
        
        
    def highLevelControl(self):
        
        '''
        
        실린더의 명령으로 다 변환해야 함
        
        '''       
        
        # 나중에 단위 변환시켜줘야 함
        self.thread_lock.acquire()
        self.Height = float(self.SndMsg7.get())              # 지면 기준
        self.CrossSectionAngle = float(self.SndMsg8.get())*np.pi/180   # 정면 각도 [deg]
        self.RotationAngle = float(self.SndMsg9.get())*np.pi/180
        self.Tiltangle = float(self.SndMsg10.get())      # 60~90 : -30~0
        
        # Initial guess for pitch and roll (can be any reasonable values)
        initial_guess = (0.0, 0.0)

        # Use fsolve to find the solutions for pitch and roll
        if self.Tiltangle > 90: self.Tiltangle = 90
        elif self.Tiltangle<60: self.Tiltangle = 60
        
        tiltCylOrder = int(-1800/30*(self.Tiltangle-60)+200)
        self.Tiltangle = (self.Tiltangle-116)*np.pi/180     
        
        solutions = fsolve(func=lambda x: [crossAngle(x[0],x[1],self.CrossSectionAngle,self.RotationAngle), desiredHeight(x[0],x[1],self.Height,self.RotationAngle,self.Tiltangle)], x0=initial_guess)
        pitch_solution, roll_solution = solutions
        print(pitch_solution)
        roll = roll_solution *180/np.pi
        pitch = pitch_solution *180/np.pi
        print(f'desired roll : {roll}, desired pitch : {pitch}')
        yaw_solution = yaw
        
        Reul = rotz(yaw_solution)@roty(pitch_solution)@rotx(roll_solution)
        
        # l_ab = 0.100
        # l_c = 0.100     # 실린더 길이 (변위를 받아봐야 함)
        # l_b = 0.100

        tb1 = Reul@b1
        tb2 = Reul@b2
        tb3 = Reul@b3

        # Inverse kinematics
        d1 = np.sqrt(a1.T@a1 + tb1.T@tb1 - 2*a1.T@tb1)
        d2 = np.sqrt(a2.T@a2 + tb2.T@tb2 - 2*a2.T@tb2)
        d3 = np.sqrt(a3.T@a3 + tb3.T@tb3 - 2*a3.T@tb3)
        print(f'd1 : {d1}, d2 : {d2}, d3 : {d3},')
        
        # 실린더 변위값
        leftCylOrder = -(d1-d1_0)*10000
        rightCylOrder = -(d2-d2_0)*10000
        sideCylOrder = -(d3-d3_0)*10000
        # 임시 : 매핑한 값
        
        # Mapping하는 식은 잘 모르겠음

        bladeRotOrder = -self.RotationAngle*10/np.pi*180
        
        if bladeRotOrder<-400 or bladeRotOrder<-400:
            return
        bladeSideshiftOrder = 0
        print(leftCylOrder,rightCylOrder,sideCylOrder,tiltCylOrder,bladeRotOrder,bladeSideshiftOrder)
        
        # leftCylOrder
        temp = struct.pack('>h',int(leftCylOrder))
        self.CMDB[10] = temp[0]
        self.CMDB[11] = temp[1]
        # rightCylOrder
        temp = struct.pack('>h',int(rightCylOrder))
        self.CMDB[12] = temp[0]
        self.CMDB[13] = temp[1]    
        # sideCylOrder
        temp = struct.pack('>h',int(sideCylOrder))
        self.CMDB[14] = temp[0]
        self.CMDB[15] = temp[1]
        # tiltCylOrder
        temp = struct.pack('>h',int(tiltCylOrder))
        self.CMDB[18] = temp[0]
        self.CMDB[19] = temp[1]
        # bladeRotOrder
        temp = struct.pack('>h',int(bladeRotOrder))
        self.CMDB[8] = temp[0]
        self.CMDB[9] = temp[1]
        # bladeSideshiftOrder
        temp = struct.pack('>h',int(bladeSideshiftOrder))
        self.CMDB[16] = temp[0]
        self.CMDB[17] = temp[1]
        
        
        # print(leftCylOrder,rightCylOrder,sideCylOrder)
        # print(self.CMDB)
        
        # self.SVP_client_socket.sendall(self.CMDD)
        self.SVP_client_socket.sendall(self.CMDB)
        self.thread_lock.release()
        
    def RPYcontrol(self):
        
        '''s
        
        실린더의 명령으로 다 변환해야 함
        
        '''       

        # if self.device.resetOrientation(xda.XRM_Heading):
        #     time.sleep(2)
        #     print('ok')
        try:
            self.device.resetOrientation(xda.XRM_Heading)
            time.sleep(0.5)
        #     
        # time.sleep(5)
        
        except: pass
        


        # self.device.resetOrientation(xda.XRM_Heading)


        # 나중에 단위 변환시켜줘야 함
        # self.thread_lock.acquire()
        
        roll = float(self.SndMsg11.get())*np.pi/180   # 정면 각도 [deg]
        pitch = float(self.SndMsg12.get())*np.pi/180
        yaw = float(self.SndMsg13.get())*np.pi/180      # 60~90 : -30~0
        
        # Initial guess for pitch and roll (can be any reasonable values)
        initial_guess = (0.0, 0.0)

        # Use fsolve to find the solutions for pitch and roll
        if roll >= 8*np.pi/180: roll = 8*np.pi/180
        elif roll<=-8*np.pi/180: roll = -8*np.pi/180
        
        if pitch >= 4*np.pi/180: pitch = 4*np.pi/180
        elif pitch <= -1*np.pi/180: pitch =-1*np.pi/180

        if yaw >= 7: yaw = 7
        elif yaw <= 0 : yaw = 0
        Reul = rotz(yaw)@roty(pitch)@rotx(roll)
        
        # l_ab = 0.100
        # l_c = 0.100     # 실린더 길이 (변위를 받아봐야 함)
        # l_b = 0.100

        tb1 = Reul@b1
        tb2 = Reul@b2
        tb3 = Reul@b3

        # Inverse kinematics
        d1 = np.sqrt(a1.T@a1 + tb1.T@tb1 - 2*a1.T@tb1)
        d2 = np.sqrt(a2.T@a2 + tb2.T@tb2 - 2*a2.T@tb2)
        d3 = np.sqrt(a3.T@a3 + tb3.T@tb3 - 2*a3.T@tb3)
        print(f'd1 : {d1}, d2 : {d2}, d3 : {d3},')
        
        # 실린더 변위값
        leftCylOrder = -(d1-d1_0)*10000
        rightCylOrder = -(d2-d2_0)*10000
        sideCylOrder = -(d3-d3_0)*10000 
        # 임시 : 매핑한 값
        
        print(f'Roll = {roll}, Pitch = {pitch}, Yaw = {yaw}')
        
    
        # Mapping하는 식은 잘 모르겠음

        bladeRotOrder = 0
        bladeSideshiftOrder = 0
        tiltCylOrder = 200
        print(leftCylOrder,rightCylOrder,sideCylOrder,tiltCylOrder,bladeRotOrder,bladeSideshiftOrder)
        
        # leftCylOrder
        temp = struct.pack('>h',int(leftCylOrder))
        self.CMDB[10] = temp[0]
        self.CMDB[11] = temp[1]
        # rightCylOrder
        temp = struct.pack('>h',int(rightCylOrder))
        self.CMDB[12] = temp[0]
        self.CMDB[13] = temp[1]    
        # sideCylOrder
        temp = struct.pack('>h',int(sideCylOrder))
        self.CMDB[14] = temp[0]
        self.CMDB[15] = temp[1]
        # tiltCylOrder
        temp = struct.pack('>h',int(tiltCylOrder))
        self.CMDB[18] = temp[0]
        self.CMDB[19] = temp[1]
        # bladeRotOrder
        temp = struct.pack('>h',int(bladeRotOrder))
        self.CMDB[8] = temp[0]
        self.CMDB[9] = temp[1]
        # bladeSideshiftOrder
        temp = struct.pack('>h',int(bladeSideshiftOrder))
        self.CMDB[16] = temp[0]
        self.CMDB[17] = temp[1]
        
        
        # print(leftCylOrder,rightCylOrder,sideCylOrder)
        # print(self.CMDB)
        
        # self.SVP_client_socket.sendall(self.CMDD)
   
        self.SVP_client_socket.sendall(self.CMDB)
        
        # self.thread_lock.release()
        
    def setIMUYaw0(self):
        # IMU의 Yaw값을 0으로 초기화시켜주기 위한 코드
        self.device.resetOrientation(xda.XRM_Heading)
        
    # 비상 정지 누를 시 기본 상태로 돌아감
    def getToInitial(self):
        self.CMDB = call_MG_CRIO_CMDB()
        self.SVP_client_socket.sendall(self.CMDB)
                
        return 0
        
    def MPCswitch(self):
        
        if self.MPCStatus==0: self.MPCStatus = 1
        
        else: self.MPCStatus = 0
        
    def stopUI(self):
        
        self.file.close()
        self.stopStatus = 1
        self.SVP_client_socket.close()
        
    def modeFunction(self):
        if self.modeSelect.get() == 0:
            self.SndBtn1['state'] = tk.DISABLED
            self.SndBtn2['state'] = tk.DISABLED
            self.SndBtn3['state'] = tk.DISABLED
            self.SndBtn4['state'] = tk.DISABLED
            self.SndBtn5['state'] = tk.DISABLED
            self.SndBtn6['state'] = tk.DISABLED
            self.SndBtn10['state'] = tk.DISABLED
        if self.modeSelect.get() == 1:
            self.SndBtn1['state'] = tk.NORMAL
            self.SndBtn2['state'] = tk.NORMAL
            self.SndBtn3['state'] = tk.NORMAL
            self.SndBtn4['state'] = tk.NORMAL
            self.SndBtn5['state'] = tk.NORMAL
            self.SndBtn6['state'] = tk.NORMAL
            self.SndBtn10['state'] = tk.NORMAL
            
    def steerInput(self):
        self.steer = float(self.SteerMsg.get())*10
        cmdd = call_MG_CRIO_CMDD()
        
        temp = struct.pack('>h',int(self.steer))
        cmdd[6] = temp[0]
        cmdd[7] = temp[1]
        print(f'Steering 명령 시행됨 : {self.steer}')
        
        self.SVP_client_socket.sendall(cmdd)
        
            
    def getInitialRef(self):
        
        leftCylOrder = -1935
        temp = struct.pack('>h',leftCylOrder)
        self.CMDB[10] = temp[0]
        self.CMDB[11] = temp[1]

        rightCylOrder = -2015
        temp = struct.pack('>h',rightCylOrder)
        self.CMDB[12] = temp[0]
        self.CMDB[13] = temp[1]
        
        sideCylOrder = -262
        temp = struct.pack('>h',sideCylOrder)
        self.CMDB[14] = temp[0]
        self.CMDB[15] = temp[1]
        
        tiltCylOrder = 20
        temp = struct.pack('>h',tiltCylOrder)
        self.CMDB[18] = temp[0]
        self.CMDB[19] = temp[1]
  

        bladeRotOrder = 0
        temp = struct.pack('>h',bladeRotOrder)
        self.CMDB[8] = temp[0]
        self.CMDB[9] = temp[1]
        # self.SVP_client_socket.sendall(self.CMDD)
        self.SVP_client_socket.sendall(self.CMDB)  
        


if __name__ == "__main__":
    
    Server_Client()
    tk.mainloop()
    
