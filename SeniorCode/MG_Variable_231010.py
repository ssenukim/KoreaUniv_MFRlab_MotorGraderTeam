import sys
# sys.path.append('c:\\users\\jisoo\\appdata\\local\\programs\\python\\python310\\lib\\site-packages')
import numpy as np
import struct
import datetime
from datetime import datetime
from scipy.optimize import fsolve, minimize, minimize_scalar
from scipy.optimize import broyden1, broyden2
import math

# Theta_4와 tilt_angle 사이의 function이 있어야 함

# Parameter values
l_blade = 3.655/2          # 블레이드의 폭 길이 [m]
l_3x = 2.12
l_4x = 0.5
l_4z = 0.77
l_x = 0
l_z = 0.320
# l_z = 0.350

# nominal value
d1_0 = 0.7739
d2_0 = 0.7649
d3_0 = 1.2234
# 명령값 - tkinter로 제작

yaw = 0                 # 3RRPS-S yaw - 0으로 고정
# theta_4 = -30/180*np.pi             # Tilt mechanism pitch 회전값
# theta_3 = 0             # Blade rotation 회전값
# height = 0.0456651        # 블레이드 목표 높이값
# height = -0.05
# cross_angle = 0         # [deg]

l_ab = 0.44
# l_c = 0.100     # 실린더 길이 (변위를 받아봐야 함)
l_b = 0.547
l_c_0 = 0.7

# a1 = np.array([-2.0272,0.7901,0.9972]).T
# a2 = np.array([-2.0272,-0.7901,0.9972]).T
# a3 = np.array([-2.2772,0.4043,0.3254]).T

# b1 = np.array([-2.1153,0.7013,0.0384]).T
# b2 = np.array([-2.1153,-0.7013,0.0384]).T
# b3 = np.array([-2.3664,-0.812,0.0384]).T

a1 = np.array([-2.0275,0.7926,0.9987]).T
a2 = np.array([-2.0275,-0.7926,0.9987]).T
a3 = np.array([-2.2775,0.4013,0.3332]).T

b1 = np.array([-2.1249,0.694,0.0413]).T
b2 = np.array([-2.1249,-0.694,0.0413]).T
b3 = np.array([-2.3644,-0.8143,0.0413]).T

# Input : angle(rad)
def rotx(r):
    rotx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0, np.sin(r),np.cos(r)]])
    return rotx
def roty(r):
    roty = np.array([[np.cos(r),0,np.sin(r)],[0,1,0],[-np.sin(r),0,np.cos(r)]])
    return roty
def rotz(r):
    rotz = np.array([[np.cos(r),-np.sin(r),0],[np.sin(r),np.cos(r),0],[0,0,1]])
    return rotz


MG_State = {"grd_protocol_id":"A",
            "grd_type":1,
            "grd_id":"ABCD-1243-QWER-5678",
            "TimeStamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'),
            "grd_bdy_lttd":0,
            "grd_bdy_lgtd":0,
            "grd_bdy_altd":0,
            "grd_bdy_x_pose":0.0,#roll
            "grd_bdy_y_pose":0.0,#pitch
            "grd_bdy_z_pose":0.0,#yaw
            "grd_bdy_velocity":0.0,#vel
            "grd_bld_x_pose":0.0,#roll
            "grd_bld_y_pose":10.0,#pitch
            "grd_bld_z_pose":20.0,#yaw
            "grd_bld_z_offset":0.0,
            "grd_running_state":0,
            "grd_running_mode":0,
            "grd_ctrlr_comm_state":1,
            "grd_gnss_comm_state":1,
            "grd_reverse_drv":False}


MG_SetRequest = {
                "ProtocolType": "B",
                "grd_cad_file_name": "",
                "grd_mc_ctrl_on_rqst": False,
                "grd_set_request":{
                                "bld_vtcal_offset":0.0,
                                "bld_hrztal_offset":0.0,
                                "bld_roll":0.0,
                                "bld_pitch":0.0,
                                "bld_yaw":0.0,
                                "bld_left_dist":0,
                                "bld_right_dist":0,
                                "bld_center_dist":0,
                                "bld_elevation":0,
                                "bld_east":0.0,
                                "bld_north":0.0
                },
                "grd_blade_manager":{
                                "focus":"Left",
                                "vertical":"Center"
                }
                }


MG_Response = {"grd_protocol_id":"R",
               "grd_req_msg_type":"B",
               "grd_response":"OK"}

def init_MG_State(): # MG_State 초기화(UI에 보내줄 때)
    #global MG_State
    
    MG_State = {"grd_protocol_id":"A",
            "grd_type":1,
            "grd_id":"ABCD-1243-QWER-5678",
            "TimeStamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'),
            "grd_bdy_lttd":36.48,
            "grd_bdy_lgtd":127.30,
            "grd_bdy_altd":67.60,
            "grd_bdy_x_pose":0.0,#roll
            "grd_bdy_y_pose":0.0,#pitch
            "grd_bdy_z_pose":0.0,#yaw
            "grd_bdy_velocity":0.0,#vel
            "grd_bld_x_pose":0.0,#roll
            "grd_bld_y_pose":10.0,#pitch
            "grd_bld_z_pose":20.0,#yaw
            "grd_bld_z_offset":0.0,
            "grd_running_state":0,
            "grd_running_mode":0,
            "grd_ctrlr_comm_state":1,
            "grd_gnss_comm_state":1,
            "grd_reverse_drv":False}
    
    return MG_State


def call_MG_CRIO_CMDB():

    # 실행되면 제어 목표를 보냄
    MG_CRIO_CMDB = bytearray(107)      
        
    temp = struct.pack('>cccccB',b'$',b'C',b'M',b'D',b'B',1)            # 끝에는 mode 전환
    MG_CRIO_CMDB[0] = temp[0]
    MG_CRIO_CMDB[1] = temp[1]
    MG_CRIO_CMDB[2] = temp[2]
    MG_CRIO_CMDB[3] = temp[3]

    MG_CRIO_CMDB[4] = temp[4]
    MG_CRIO_CMDB[5] = temp[5]           # Blade control mode On/off
    # On: 1, Off: 0

    '''
    이후에 mode를 full auto, half blade control로 전환 예정
    '''

    temp = struct.pack('>hhhhhhh',0,0,0,0,0,0,0)        # PID control desired value
    MG_CRIO_CMDB[6] = temp[0]
    MG_CRIO_CMDB[7] = temp[1]

    MG_CRIO_CMDB[8] = temp[2]
    MG_CRIO_CMDB[9] = temp[3]

    MG_CRIO_CMDB[10] = temp[4]
    MG_CRIO_CMDB[11] = temp[5]

    MG_CRIO_CMDB[12] = temp[6]
    MG_CRIO_CMDB[13] = temp[7]

    MG_CRIO_CMDB[14] = temp[8]
    MG_CRIO_CMDB[15] = temp[9]

    MG_CRIO_CMDB[16] = temp[10]
    MG_CRIO_CMDB[17] = temp[11]

    MG_CRIO_CMDB[18] = temp[12]
    MG_CRIO_CMDB[19] = temp[13]

    temp = struct.pack('>ffffffffffffff',1,2,1,1,1,1,1,0,0,0,0,0,0,0)       # Kp gain, Ki gain
    MG_CRIO_CMDB[20] = temp[0]
    MG_CRIO_CMDB[21] = temp[1]
    MG_CRIO_CMDB[22] = temp[2]
    MG_CRIO_CMDB[23] = temp[3]
    
    MG_CRIO_CMDB[24] = temp[4]
    MG_CRIO_CMDB[25] = temp[5]
    MG_CRIO_CMDB[26] = temp[6]
    MG_CRIO_CMDB[27] = temp[7]

    MG_CRIO_CMDB[28] = temp[8]
    MG_CRIO_CMDB[29] = temp[9]
    MG_CRIO_CMDB[30] = temp[10]
    MG_CRIO_CMDB[31] = temp[11]
    
    MG_CRIO_CMDB[32] = temp[12]
    MG_CRIO_CMDB[33] = temp[13]
    MG_CRIO_CMDB[34] = temp[14]
    MG_CRIO_CMDB[35] = temp[15]
    
    MG_CRIO_CMDB[36] = temp[16]
    MG_CRIO_CMDB[37] = temp[17]
    MG_CRIO_CMDB[38] = temp[18]
    MG_CRIO_CMDB[39] = temp[19]
    
    MG_CRIO_CMDB[40] = temp[20]
    MG_CRIO_CMDB[41] = temp[21]
    MG_CRIO_CMDB[42] = temp[22]
    MG_CRIO_CMDB[43] = temp[23]
    
    MG_CRIO_CMDB[44] = temp[24]
    MG_CRIO_CMDB[45] = temp[25]
    MG_CRIO_CMDB[46] = temp[26]
    MG_CRIO_CMDB[47] = temp[27]

    MG_CRIO_CMDB[48] = temp[28]
    MG_CRIO_CMDB[49] = temp[29]
    MG_CRIO_CMDB[50] = temp[30]
    MG_CRIO_CMDB[51] = temp[31]
    
    MG_CRIO_CMDB[52] = temp[32]
    MG_CRIO_CMDB[53] = temp[33]
    MG_CRIO_CMDB[54] = temp[34]
    MG_CRIO_CMDB[55] = temp[35]
    
    MG_CRIO_CMDB[56] = temp[36]
    MG_CRIO_CMDB[57] = temp[37]
    MG_CRIO_CMDB[58] = temp[38]
    MG_CRIO_CMDB[59] = temp[39]
    
    MG_CRIO_CMDB[60] = temp[40]
    MG_CRIO_CMDB[61] = temp[41]
    MG_CRIO_CMDB[62] = temp[42]
    MG_CRIO_CMDB[63] = temp[43]
    
    MG_CRIO_CMDB[64] = temp[44]
    MG_CRIO_CMDB[65] = temp[45]
    MG_CRIO_CMDB[66] = temp[46]
    MG_CRIO_CMDB[67] = temp[47]
    
    MG_CRIO_CMDB[68] = temp[48]
    MG_CRIO_CMDB[69] = temp[49]
    MG_CRIO_CMDB[70] = temp[50]
    MG_CRIO_CMDB[71] = temp[51]
    
    MG_CRIO_CMDB[72] = temp[52]
    MG_CRIO_CMDB[73] = temp[53]
    MG_CRIO_CMDB[74] = temp[54]
    MG_CRIO_CMDB[75] = temp[55]

    temp = struct.pack('>HHHHHHHHHHHHHH',500,510,510,510,500,500,510,400,500,410,410,400,400,400)

    MG_CRIO_CMDB[76] = temp[0]
    MG_CRIO_CMDB[77] = temp[1]

    MG_CRIO_CMDB[78] = temp[2]
    MG_CRIO_CMDB[79] = temp[3]

    MG_CRIO_CMDB[80] = temp[4]
    MG_CRIO_CMDB[81] = temp[5]

    MG_CRIO_CMDB[82] = temp[6]
    MG_CRIO_CMDB[83] = temp[7]

    MG_CRIO_CMDB[84] = temp[8]
    MG_CRIO_CMDB[85] = temp[9]

    MG_CRIO_CMDB[86] = temp[10]
    MG_CRIO_CMDB[87] = temp[11]

    MG_CRIO_CMDB[88] = temp[12]
    MG_CRIO_CMDB[89] = temp[13]
    
    MG_CRIO_CMDB[90] = temp[14]
    MG_CRIO_CMDB[91] = temp[15]

    MG_CRIO_CMDB[92] = temp[16]
    MG_CRIO_CMDB[93] = temp[17]

    MG_CRIO_CMDB[94] = temp[18]
    MG_CRIO_CMDB[95] = temp[19]

    MG_CRIO_CMDB[96] = temp[20]
    MG_CRIO_CMDB[97] = temp[21]

    MG_CRIO_CMDB[98] = temp[22]
    MG_CRIO_CMDB[99] = temp[23]

    MG_CRIO_CMDB[100] = temp[24]
    MG_CRIO_CMDB[101] = temp[25]

    MG_CRIO_CMDB[102] = temp[26]
    MG_CRIO_CMDB[103] = temp[27]

    temp = struct.pack('>B',1)

    MG_CRIO_CMDB[104] = temp[0]

    MG_CRIO_CMDB[105] = 13
    MG_CRIO_CMDB[106] = 10
    
    return MG_CRIO_CMDB

def call_MG_CRIO_CMDC1():

    # 실행되면 제어 목표를 보냄
    MG_CRIO_CMDC1 = bytearray(9)      
        
    temp = struct.pack('>ccccccB',b'$',b'C',b'M',b'D',b'B',b'1',0)
    MG_CRIO_CMDC1[0] = temp[0]
    MG_CRIO_CMDC1[1] = temp[1]
    MG_CRIO_CMDC1[2] = temp[2]
    MG_CRIO_CMDC1[3] = temp[3]
    MG_CRIO_CMDC1[4] = temp[4]
    MG_CRIO_CMDC1[5] = temp[5]
    MG_CRIO_CMDC1[6] = temp[6]
    MG_CRIO_CMDC1[7] = 13
    MG_CRIO_CMDC1[8] = 10
    
    return MG_CRIO_CMDC1
    
def call_MG_CRIO_CMDC2():
    
    # 형식 맞춰서
    MG_CRIOC2 = {"la":[35.9369518527778,35.9366952222222],              # n개
                 "lo":[128.814011163889,128.813957002778],              # n개
                 "vel":[2],                                             # (n-1)개
                 "dir":[1],                                             # (n-1)개
                 "shift":[1]}                                           # (n-1)개
    
    return MG_CRIOC2
    
    
def call_MG_CRIO_CMDD():

    # 실행되면 제어 목표를 보냄
    MG_CRIO_CMDD = bytearray(16)      
        
    temp = struct.pack('>ccccc',b'$',b'C',b'M',b'D',b'D')
    MG_CRIO_CMDD[0] = temp[0]
    MG_CRIO_CMDD[1] = temp[1]
    MG_CRIO_CMDD[2] = temp[2]
    MG_CRIO_CMDD[3] = temp[3]
    MG_CRIO_CMDD[4] = temp[4]

    temp = struct.pack('>BhBBBBBB',1,0,0,100,100,2,1,0)
    # Mode : 0 : OFF / 1 : ON
    MG_CRIO_CMDD[5] = temp[0]
    # Wheel direction [0.1 deg]
    MG_CRIO_CMDD[6] = temp[1]
    MG_CRIO_CMDD[7] = temp[2]
    # Accel [%]
    MG_CRIO_CMDD[8] = temp[3]
    # Brake [%]
    MG_CRIO_CMDD[9] = temp[4]
    # Clutch [%]
    MG_CRIO_CMDD[10] = temp[5]
    # Gear (1 : Reverse / 2 : Netural / 3 : Forward) 
    MG_CRIO_CMDD[11] = temp[6]
    # Transmission
    MG_CRIO_CMDD[12] = temp[7]
    # Logging : 0 : OFF / 1 : ON
    MG_CRIO_CMDD[13] = temp[8]
    
    MG_CRIO_CMDD[14] = 13
    MG_CRIO_CMDD[15] = 10
    
    return MG_CRIO_CMDD
   
def driving_setup():
    
    cmd = call_MG_CRIO_CMDD()
    # mode enable
    cmd[5] = 1
    cmd[11] = 3
    
    return cmd

def driving_start(cmd):
    cmd[9] = 0
    return cmd
    
def driving_start_init(cmd):
    
    cmd[5] = 0
    temp = struct.pack('>h',0)
    cmd[6] = temp[0]
    cmd[7] = temp[1]
    cmd[8] = 0
    cmd[9] = 40
    cmd[10] = 100
    return cmd

def driving_start_fin(cmd):
    
    cmd[5] = 0
    temp = struct.pack('>h',0)
    cmd[6] = temp[0]
    cmd[7] = temp[1]
    cmd[8] = 0
    cmd[9] = 100
    cmd[10] = 100
    return cmd
    
def crossAngle(pitch, roll,cross_angle,theta_3):
    return np.tan(cross_angle) - (l_blade*(np.sin(pitch)*np.sin(theta_3) + np.cos(pitch)*np.cos(theta_3)*np.sin(roll)))/(l_blade*np.cos(roll)*np.cos(theta_3)) 


def desiredHeight(pitch, roll,height,theta_3,theta_4):
    return 2*l_4x*(np.cos(theta_3)*np.sin(pitch) - np.cos(pitch)*np.sin(roll)*np.sin(theta_3)) + 2*l_3x*np.sin(pitch) - 2*l_x*(np.cos(theta_4)*(np.cos(theta_3)*np.sin(pitch) - np.cos(pitch)*np.sin(roll)*np.sin(theta_3)) + np.cos(pitch)*np.cos(roll)*np.sin(theta_4)) + 2*l_z*(np.sin(theta_4)*(np.cos(theta_3)*np.sin(pitch) - np.cos(pitch)*np.sin(roll)*np.sin(theta_3)) - np.cos(pitch)*np.cos(roll)*np.cos(theta_4)) - 2*l_4z*np.cos(pitch)*np.cos(roll) - 2*height
     
def crossAngle_Global(pitch, roll,cross_angle,theta_3,IMU1_roll,IMU1_pitch):
    # 함수 목표 : 차량의 기울어진 정도를 반영하여 목표 rp 수정
    offset_roll = math.atan((np.cos(IMU1_pitch)*np.sin(IMU1_roll))/np.cos(IMU1_roll))
    print(f'offset : {offset_roll}')
    roll = roll + offset_roll/180*np.pi
    
    return np.tan(cross_angle) - (l_blade*(np.sin(pitch)*np.sin(theta_3) + np.cos(pitch)*np.cos(theta_3)*np.sin(roll)))/(l_blade*np.cos(roll)*np.cos(theta_3)) 


def desiredHeight_Global(pitch, roll,height,theta_3,theta_4,IMU1_roll,IMU1_pitch):
    
    offset_roll = math.atan((np.cos(IMU1_pitch)*np.sin(IMU1_roll))/np.cos(IMU1_roll))
    roll = roll + offset_roll/180*np.pi
    
    return 2*l_4x*(np.cos(theta_3)*np.sin(pitch) - np.cos(pitch)*np.sin(roll)*np.sin(theta_3)) + 2*l_3x*np.sin(pitch) - 2*l_x*(np.cos(theta_4)*(np.cos(theta_3)*np.sin(pitch) - np.cos(pitch)*np.sin(roll)*np.sin(theta_3)) + np.cos(pitch)*np.cos(roll)*np.sin(theta_4)) + 2*l_z*(np.sin(theta_4)*(np.cos(theta_3)*np.sin(pitch) - np.cos(pitch)*np.sin(roll)*np.sin(theta_3)) - np.cos(pitch)*np.cos(roll)*np.cos(theta_4)) - 2*l_4z*np.cos(pitch)*np.cos(roll) - 2*height
     

# def crossAngle_Global(pitch, roll,cross_angle,theta_3,IMU1_roll,IMU1_pitch):
#     # 함수 목표 : 차량의 기울어진 정도를 반영하여 목표 rp 수정
#     offset_roll = math.atan((np.cos(IMU1_pitch)*np.sin(roll))/np.cos(IMU1_roll))
#     roll = roll - offset_roll/180*np.pi
    
#     return (np.sin(theta_3)*(np.sin(IMU1_pitch)*np.cos(pitch) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.sin(pitch))+ np.cos(theta_3)*(np.cos(IMU1_pitch)*np.sin(IMU1_roll)*np.cos(roll) - np.sin(IMU1_pitch)*np.sin(pitch)*np.sin(roll) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.cos(pitch)*np.sin(roll)))/(np.cos(theta_3)*(np.cos(IMU1_roll)*np.cos(roll) - np.sin(IMU1_roll)*np.cos(pitch)*np.sin(roll)) - np.sin(IMU1_roll)*np.sin(pitch)*np.sin(theta_3)) - np.tan(cross_angle)


# def desiredHeight_Global(pitch, roll,height,theta_3,theta_4,IMU1_roll,IMU1_pitch):
    
#     offset_roll = math.atan((np.cos(IMU1_pitch)*np.sin(roll))/np.cos(IMU1_roll))
#     roll = roll - offset_roll/180*np.pi
    
#     return l_z*(np.sin(theta_4)*(np.cos(theta_3)*(np.sin(IMU1_pitch)*np.cos(pitch) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.sin(pitch)) - np.sin(theta_3)*(np.cos(IMU1_pitch)*np.sin(IMU1_roll)*np.cos(roll) - np.sin(IMU1_pitch)*np.sin(pitch)*np.sin(roll) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.cos(pitch)*np.sin(roll))) + np.cos(theta_4)*(np.cos(IMU1_pitch)*np.sin(IMU1_roll)*np.sin(roll) + np.sin(IMU1_pitch)*np.cos(roll)*np.sin(pitch) - np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.cos(pitch)*np.cos(roll))) + l_4z*(np.cos(IMU1_pitch)*np.sin(IMU1_roll)*np.sin(roll) + np.sin(IMU1_pitch)*np.cos(roll)*np.sin(pitch) - np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.cos(pitch)*np.cos(roll)) + l_3x*(np.sin(IMU1_pitch)*np.cos(pitch) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.sin(pitch)) + l_4x*(np.cos(theta_3)*(np.sin(IMU1_pitch)*np.cos(pitch) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.sin(pitch)) - np.sin(theta_3)*(np.cos(IMU1_pitch)*np.sin(IMU1_roll)*np.cos(roll) - np.sin(IMU1_pitch)*np.sin(pitch)*np.sin(roll) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.cos(pitch)*np.sin(roll)))-height
     
     
     
def UItoCRIO_CMDB(dict_UI_CMD): # UI CMD(MG_Request)를 CRIO CMD(MG_CRIO_CMD)로 변환
    
    '''
    변경 사항 있음
    '''
    
    # MG_CRIO_CMD = init_MG_CRIO_CMD()
    MG_CRIO_CMD = call_MG_CRIO_CMDB()
    # 각도를 길이로로 변환해야함
    '''
    AutoMode = dict_UI_CMD['grd_mc_ctrl_on_rqst']                       # 0 or 1
    # 블레이드 사이드 시프트 조절
    bld_vtcal_offset = dict_UI_CMD['grd_set_request']['bld_vtcal_offset']             # degree float -> 변환 문의
    # 리프트 실린더 조절
    bld_hrztal_offset = dict_UI_CMD['grd_set_request']['bld_hrztal_offset']       # bld_elevation + roll (degree) ->  cylinder length(mm) : 변환
    # 기준점 반영하여 반대 실린더 조절 - focus data를 활용
    bld_roll = dict_UI_CMD['grd_set_request']['bld_roll']
    # tilt cylinder 제어
    bld_pitch = dict_UI_CMD['grd_set_request']['bld_pitch']
    # 블레이드 로테이션 제어
    bld_yaw = dict_UI_CMD['grd_set_request']['bld_yaw']
    # ?? 어떤 거리를 말하는지 모르겠음
    bld_left_dist = dict_UI_CMD['grd_set_request']['bld_left_dist']
    bld_right_dist = dict_UI_CMD['grd_set_request']['bld_right_dist']
    bld_center_dist = dict_UI_CMD['grd_set_request']['bld_center_dist']
    
    # 리프트 실린더 조절 - hrztal_offset과 다른점을 모르겠음
    bld_elevation = dict_UI_CMD['grd_set_request']['bld_elevation']
    focus = dict_UI_CMD['grd_set_request']['focus']
    
    
    # bld_pitch (degree) -> cylinder length(mm) : 변환
    '''


    '''
    # 필요한 것 - 실린더 단위 제어로 변환
    '''
    
    temp = struct.pack('Bhhhh',0,0,0,0,0) # fixed
    # temp = struct.pack('Bhhhh',int(AutoMode),int(circleDrive),int(leftBldLift),int(rightBldLift),int(bldPitch)) # fixed
    MG_CRIO_CMD[5] = temp[0]
    MG_CRIO_CMD[33] = temp[1]                                           # bld circle drive (문의)
    MG_CRIO_CMD[34] = temp[2]                                           # bld circle drive (문의)
    
    MG_CRIO_CMD[35] = temp[3]                                           # left bld lift (mm)
    MG_CRIO_CMD[36] = temp[4]                                           # left bld lift (mm)
    
    MG_CRIO_CMD[37] = temp[5]                                           # right bld lift (mm)
    MG_CRIO_CMD[38] = temp[7]                                           # right bld lift (mm)
    
    MG_CRIO_CMD[43] = temp[6]                                           # bld pitch (mm)
    MG_CRIO_CMD[44] = temp[8]                                           # bld pitch (mm)

    return MG_CRIO_CMD

# def desiredHeight_Global(pitch, roll,height,theta_3,theta_4,IMU1_roll,IMU1_pitch,side):

#     return l_z*(np.sin(theta_4)*(np.cos(theta_3)*(np.sin(IMU1_pitch)*np.cos(pitch) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.sin(pitch)) - np.sin(theta_3)*(np.cos(IMU1_pitch)*np.sin(IMU1_roll)*np.cos(roll) - np.sin(IMU1_pitch)*np.sin(pitch)*np.sin(roll) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.cos(pitch)*np.sin(roll))) + np.cos(theta_4)*(np.cos(IMU1_pitch)*np.sin(IMU1_roll)*np.sin(roll) + np.sin(IMU1_pitch)*np.cos(roll)*np.sin(pitch) - np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.cos(pitch)*np.cos(roll))) - l_x*(np.cos(theta_4)*(np.cos(theta_3)*(np.sin(IMU1_pitch)*np.cos(pitch) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.sin(pitch)) - np.sin(theta_3)*(np.cos(IMU1_pitch)*np.sin(IMU1_roll)*np.cos(roll) - np.sin(IMU1_pitch)*np.sin(pitch)*np.sin(roll) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.cos(pitch)*np.sin(roll))) - np.sin(theta_4)*(np.cos(IMU1_pitch)*np.sin(IMU1_roll)*np.sin(roll) + np.sin(IMU1_pitch)*np.cos(roll)*np.sin(pitch) - np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.cos(pitch)*np.cos(roll))) + l_4z*(np.cos(IMU1_pitch)*np.sin(IMU1_roll)*np.sin(roll) + np.sin(IMU1_pitch)*np.cos(roll)*np.sin(pitch) - np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.cos(pitch)*np.cos(roll)) + l_3x*(np.sin(IMU1_pitch)*np.cos(pitch) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.sin(pitch)) + l_4x*(np.cos(theta_3)*(np.sin(IMU1_pitch)*np.cos(pitch) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.sin(pitch)) - np.sin(theta_3)*(np.cos(IMU1_pitch)*np.sin(IMU1_roll)*np.cos(roll) - np.sin(IMU1_pitch)*np.sin(pitch)*np.sin(roll) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.cos(pitch)*np.sin(roll))) + side*(np.sin(theta_3)*(np.sin(IMU1_pitch)*np.cos(pitch) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.sin(pitch)) + np.cos(theta_3)*(np.cos(IMU1_pitch)*np.sin(IMU1_roll)*np.cos(roll) - np.sin(IMU1_pitch)*np.sin(pitch)*np.sin(roll) + np.cos(IMU1_roll)*np.cos(IMU1_pitch)*np.cos(pitch)*np.sin(roll)))-height
 
# def crossAngle_Global(pitch, roll,cross_angle,theta_3,IMU1_roll,IMU1_pitch):

#     return (np.sin(theta_3)*(np.sin(IMU1_pitch)*np.sin(pitch) + np.sin(IMU1_roll)*np.sin(IMU1_pitch)*np.sin(pitch)) + np.sin(theta_3)*(np.sin(IMU1_pitch)*np.sin(IMU1_roll)*np.sin(roll) - np.sin(IMU1_pitch)*np.sin(pitch)*np.sin(roll) + np.sin(IMU1_roll)*np.sin(IMU1_pitch)*np.sin(pitch)*np.sin(roll)))/(np.sin(theta_3)*(np.sin(IMU1_roll)*np.sin(roll) - np.sin(IMU1_roll)*np.sin(pitch)*np.sin(roll)) - np.sin(IMU1_roll)*np.sin(pitch)*np.sin(theta_3))-np.tan(cross_angle)

