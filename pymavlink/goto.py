import time
import math
import numpy as np
from pymavlink import mavutil
import time
import math

point = LocationGlobalRelative(48.581586, 7.764111, 2)

time.sleep(1)

#~ chemin_drone = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0'
chemin_drone = '/dev/ttyACM0'
#~ drone = connect(chemin_drone, wait_ready=True, baud=57600, heartbeat_timeout=2)

#chemin_drone = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0'

# Connect to the Vehicle (in this case a UDP endpoint)
print ("Connexion au vehicule ...")
vehicle = connect(chemin_drone, wait_ready=False, baud=57600, heartbeat_timeout=2)
print ("Connexion établie !")

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print ("Verifications basiques de pre-armement")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" En attente de l'initialisation du vehicule...")
        time.sleep(1)

    print ("Armement des moteurs")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" En attente d'armement...")
        time.sleep(1)

    print ("Décollage!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Altitude souhaitee atteinte")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# Attente du mode "STABIIZE"
while vehicle.mode != VehicleMode("STABILIZE"):
    print("En attente du mode STABILIZE")
    time.sleep(1)

        # Attente du mode "AUTO"
while vehicle.mode != VehicleMode("AUTO"):
    print("En attente du mode AUTO")
    time.sleep(1)

        # Passage en mode "GUIDED"
vehicle.mode = VehicleMode("GUIDED_NOGPS")
print ("Véhicule en mode guidé !")

print ("Début de la procédure d'armement et de décollage ...")
arm_and_takeoff(2)
print ("Attente de 3 secondes")
time.sleep(3)

print("Reglage de la vitesse aerienne à 3")
vehicle.airspeed = 3

print ("Début du vol vers le point")
vehicle.simple_goto(point)

pos = vehicle.location.global_relative_frame
dist = get_distance_metres(pos, point)

while(dist > 3):
    time.sleep(0.3)
    pos = vehicle.location.global_relative_frame
    dist = get_distance_metres(pos, point)
    print ("Distance au waypoint: {} m".format(dist))
print("Waypoint atteint ! ")

time.sleep(1)

print("Mise en mode atterrissage")
vehicle.mode = VehicleMode("LAND")

print ("Deconnexion du vehicule")
vehicle.close()

print ("Arret du script")
