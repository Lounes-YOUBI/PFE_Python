import time
import math
import numpy as np
from pymavlink import mavutil
import time
import math
from collections import namedtuple


# Classe Position
Position = namedtuple('Position', ['lat_deg', 'lon_deg', 'relative_alt_m'])

point = Position(lat_deg=48.581962, lon_deg = 7.763348, relative_alt_m = 1)

time.sleep(1)

#~ chemin_drone = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0'
chemin_drone = '/dev/ttyACM0'
#~ drone = connect(chemin_drone, wait_ready=True, baud=57600, heartbeat_timeout=2)

#chemin_drone = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0'

# Connect to the Vehicle (in this case a UDP endpoint)
print ("Connexion au vehicule ...")
vehicle = mavutil.mavlink_connection('/dev/ttyACM0')
# Attente du premier message HEARTBEAT (confirmation de connexion)
vehicle.wait_heartbeat()
print("Drone connecté")

vehicle.mav.request_data_stream_send(vehicle.target_system, vehicle.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

def is_armable():
	""" Vérifie si le drone est armable """
	msg = vehicle.recv_match(type='SYS_STATUS', blocking=True)
	prearm_check = msg.onboard_control_sensors_health  # Bitmask du statut
	return prearm_check > 0  # Si tout va bien, la valeur est non nulle

# Fonction pour changer de mode
def set_mode(mode_name):
	""" Change le mode de vol du drone """
	mode_id = vehicle.mode_mapping().get(mode_name)
	if mode_id is None:
		print(f"Mode {mode_name} inconnu !")
		return

	# Envoi du message MAVLink pour changer de mode
	vehicle.mav.set_mode_send(
		vehicle.target_system,
		mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
		mode_id
	)
	#print(f"Demande de passage en mode {mode_name} envoyée.")

	# Attendre la confirmation
	while True:
		msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
		if msg.custom_mode == mode_id:
			#print(f"Mode {mode_name} activé !")
			break

def get_mode():
	""" Récupère et affiche le mode de vol actuel """
	msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
	if not msg:
		print("Impossible de récupérer le mode.")
		return None

	mode_id = msg.custom_mode  # ID du mode actuel
	mode_name = None

	# Mapper l'ID avec un nom de mode
	for name, mode in vehicle.mode_mapping().items():
		if mode == mode_id:
			mode_name = name
			break

	if mode_name:
		print(f"Mode actuel : {mode_name}")
	else:
		print(f"Mode inconnu (ID: {mode_id})")

	return mode_name


# Fonction de decollage du drone du GitHub de dronekit
def arm_and_takeoff(aTargetAltitude):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""

	print("Basic pre-arm checks")
	# Don't try to arm until autopilot is ready
	#while not is_armable2():
		#print(" Waiting for vehicle to initialise...")

	time.sleep(1)
	print("Arming motors")

	# Copter should arm in GUIDED mode
	set_mode("GUIDED")
	vehicle.arducopter_arm()  # On arme le drone

	# Confirm vehicle armed before attempting to take off
	vehicle.motors_armed_wait()
	print("Drone armé")

	print("Taking off!")
	# Décollage
	vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
								 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
								 0, 0, 0, 0, 0, 0, aTargetAltitude
								 )
	print(f"Décollage à {aTargetAltitude}m")

	# Attente de l'altitude cible
	while True:
		msg = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=True)
		alt = msg.current_distance / 100.0  # Convertir en mètres
		print(f"Altitude : {alt:.1f}m")
		if alt >= aTargetAltitude * 0.95:  # Seuil de 95%
			print("Altitude atteinte !")
			break



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

# Fonction pour aller à un waypoint en attendant d'être arrivé
def goto(latitude, longitude, altitude):
	""" Envoie le drone vers une position GPS en mode GUIDED """

	# Vérifier si le drone est bien en mode GUIDED
	set_mode("GUIDED")

	# Envoyer la commande MAVLink NAV_WAYPOINT
	vehicle.mav.mission_item_send(
		vehicle.target_system, vehicle.target_component,
		0,  # Séquence du waypoint
		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Référence relative au sol
		mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Commande de navigation
		2, 0,  # Confirmation requise
		0, 0, 0, 0,  # Paramètres inutilisés
		latitude, longitude, altitude  # Coordonnées
	)
	print(f"En route vers {latitude}, {longitude}, {altitude}m")

	# Attendre d'atteindre le waypoint
	while True:
		msg = vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
		msg_alt = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=True)
		if msg and msg_alt:
			current_lat = msg.lat / 1e7
			current_lon = msg.lon / 1e7
			current_alt = msg_alt.current_distance / 100.0  # Convertir en mètres

			print(f"Position actuelle : {current_lat}, {current_lon}, {current_alt}m")

			# Vérification si on est proche du waypoint
			if abs(current_lat - latitude) < 0.00005 and abs(current_lon - longitude) < 0.00005 and abs(
					current_alt - altitude) < 0.5:
				print("Waypoint atteint !")
				break

# Attente du mode "STABIIZE"
while get_mode() != "STABILIZE":
    print("En attente du mode STABILIZE")
    time.sleep(1)

        # Attente du mode "AUTO"
while get_mode() != "AUTO":
    print("En attente du mode AUTO")
    time.sleep(1)

        # Passage en mode "GUIDED"
set_mode("GUIDED")
print ("Véhicule en mode guidé !")

print ("Début de la procédure d'armement et de décollage ...")
arm_and_takeoff(2)
print ("Attente de 3 secondes")
time.sleep(3)

print("Reglage de la vitesse aerienne à 3")
vehicle.airspeed = 3

print ("Début du vol vers le point")

goto(point.lat_deg, point.lon_deg, point.relative_alt_m)

time.sleep(1)

print("Mise en mode atterrissage")
set_mode("LAND")

print("Delai de 10 secondes")

time.sleep(10)
#vehicle.close()

print ("Arret du script")
