from dronekit import connect, VehicleMode
import time



time.sleep(1)
print("Connecting...")
#~ chemin_drone = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0'
chemin_drone = '/dev/ttyACM0'
#~ drone = connect(chemin_drone, wait_ready=True, baud=57600, heartbeat_timeout=2)

#chemin_drone = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0'
vehicle = connect(chemin_drone, wait_ready=False, baud=57600, heartbeat_timeout=2)
#drone = connect('127.0.0.1:14550', wait_ready=True)


# Connect to the Vehicle (in this case a UDP endpoint)
#vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print ("Basic pre-arm checks")
    # Don't try to arm until autwinopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

# Attente du mode "STABIIZE"
while vehicle.mode != VehicleMode("STABILIZE"):
    print("En attente du mode STABILIZE")
    time.sleep(1)

        # Attente du mode "AUTO"
while vehicle.mode != VehicleMode("AUTO"):    
    print("En attente du mode AUTO")
    time.sleep(1)

        # Passage en mode "GUIDED"    
vehicle.mode = VehicleMode("GUIDED")
print ("Véhicule en mode guidé !")

arm_and_takeoff(2)

time.sleep(2)


print("Mise en mode atterrissage")
vehicle.mode = VehicleMode("LAND")

print ("Deconnexion du vehicule")
vehicle.close()

print ("Arret du script")
