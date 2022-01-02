from __future__ import print_function
import time

from dronekit import connect, VehicleMode, mavutil, LocationGlobalRelative
import math
import pickle
import requests

def arm_and_takeoff( aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(2)
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def hiz(speed):
    vehicle.groundspeed= speed
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    for x in range(0, duration):
        vehicle.send_mavlink(msg)


vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
a_e=320
a_n=480
time.sleep(2)
arm_and_takeoff(8)
time.sleep(2)
counter=0
def goto_position_target_relative_ned(x, y, down):

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111111000,
        x, y, down,
        0, 0, 0,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)

def current_milli_time():
    return round(time.time() * 1000)
kord = []
data = {}
while True:

    try:
        with open("pay1.pkl", "rb") as f:
            xd = pickle.load(f)
            time.sleep(3)
    except (EOFError):
        pass
        print("okuma hatasi")
    a_e = xd[0]
    a_n = xd[1]
    dist_x = math.fabs(a_e - 320)
    dist_y = math.fabs(a_n - 480)
    disx=math.pow(dist_x,2)
    disy=math.pow(dist_y,2)
    distance=disx+disy
    dis=math.sqrt(distance)
    aci=math.acos(dist_y/dis)
    aci_degree=(aci*180)/math.pi
    print(aci_degree)
    counter=counter+1
    if (a_e > 300 and a_e < 360 and a_n > 200 and a_n < 280):
        send_ned_velocity(0,0,0,1)
        print("Position Hold")


    elif a_e > 320:
        if a_n<240:

            v_n = math.cos(aci)
            v_e = math.sin(aci)
            v_N=v_n*5
            v_E=v_e*5
            send_ned_velocity(v_N,v_E,0,3)
            print (" kuzey dogu ")
            print (v_N)
        elif a_n>240:
            v_n = math.cos(aci)
            v_e = math.sin(aci)
            v_N=-v_n*5
            v_E=v_e*5
            send_ned_velocity(v_N,v_E,0,3)
            print (" guney dogu ")
            print (-v_N)
    elif  a_e < 320:

        if a_n<240:

            v_n = math.cos(aci)
            v_e = math.sin(aci)
            v_N=v_n*5
            v_E=-v_e*5
            send_ned_velocity(v_N,v_E,0,3)
            print (" kuzey bati ")
            print (v_N)
        elif a_n>240:

            v_n = math.cos(aci)
            v_e = math.sin(aci)
            v_N=-v_n*5
            v_E=-v_e*5
            send_ned_velocity(v_N,v_E,0,3)
            print (" guney bati ")
            print (-v_N)
            #print (radian_3)
    else:
        send_ned_velocity(0,0,0,3)
        print("stabil")
        if vehicle.mode.name == 'PosHold':
            requests.get(' http://127.0.0.25:5000 /api/cikis')
            print("oturum kapanıyor")
    if counter>=20:
        print("breaking counter excedeed 40")
        break
    if vehicle.mode.name!="GUIDED" :
        break
    a_e=320
    a_n=480
    print (counter)


vehicle.mode=VehicleMode("RTL")
print ("RTL modu")
time.sleep(3)
vehicle.close()








