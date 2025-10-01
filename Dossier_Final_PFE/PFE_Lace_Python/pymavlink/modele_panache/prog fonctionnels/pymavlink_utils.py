#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
from pymavlink import mavutil
from collections import namedtuple
import pymap3d as pm # Bibliothèque pour la conversion GPS / ENU

# Classe Position
Position = namedtuple('Position', ['lat_deg', 'lon_deg', 'relative_alt_m'])

################################# Fonctions #######################################


def enu_to_gps(x, y, z, ref_lat, ref_lon, ref_alt):
    """obotique, Télédéte
    Convertit les coordonnées ENU en GPS par rapport à une origine.
    """
    lat, lon, alt = pm.enu2geodetic(x, y, z, ref_lat, ref_lon, ref_alt)
    return lat, lon, alt

def is_armable(master):
	""" Vérifie si le drone est armable """
	msg = master.recv_match(type='SYS_STATUS', blocking=True)
	prearm_check = msg.onboard_control_sensors_health  # Bitmask du statut
	return prearm_check > 0  # Si tout va bien, la valeur est non nulle


# Fonction pour changer de mode
def set_mode(master, mode_name):
	""" Change le mode de vol du drone """
	mode_id = master.mode_mapping().get(mode_name)
	if mode_id is None:
		print(f"Mode {mode_name} inconnu !")
		return

	# Envoi du message MAVLink pour changer de mode
	master.mav.set_mode_send(
		master.target_system,
		mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
		mode_id
	)
	#print(f"Demande de passage en mode {mode_name} envoyée.")

	# Attendre la confirmation
	while True:
		msg = master.recv_match(type='HEARTBEAT', blocking=True)
		if msg.custom_mode == mode_id:
			#print(f"Mode {mode_name} activé !")
			break


def get_mode(master):
    """ Récupère et affiche le mode de vol actuel """
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if not msg:
        print("Impossible de récupérer le mode.")
        return None
    while msg.type != 2: # MAV_TYPE_QUADROTOR
        msg = master.recv_match(type='HEARTBEAT', blocking=True)

    mode_id = msg.custom_mode
    mode_name = None

    for name, mode in master.mode_mapping().items():
        if mode == mode_id:
            mode_name = name
            break

    if mode_name:
        print(f"Mode actuel : {mode_name}")
    else:
        print(f"Mode inconnu (ID: {mode_id})")

    return mode_name


def get_position(master):
	""" Récupère la position actuelle du drone et retourne un objet Position """
	msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
	if not msg:
		print("Erreur : Impossible de récupérer la position.")
		return None

	# Conversion des données reçues
	lat = msg.lat / 1e7  # De micro degrés en degrés
	lon = msg.lon / 1e7
	alt = msg.relative_alt / 1000.0  # De millimètres en mètres

	# Création de l'objet Position
	return Position(lat_deg=lat, lon_deg=lon, relative_alt_m=alt)


def set_param(master, param_id, value):
	""" Change un paramètre du drone via MAVLink """
	master.mav.param_set_send(
		master.target_system, master.target_component,
		param_id.encode(),  # Nom du paramètre
		float(value),  # Valeur du paramètre
		mavutil.mavlink.MAV_PARAM_TYPE_REAL32  # Type de paramètre
	)
	print(f"Paramètre {param_id} réglé à {value}")


# Fonction de decollage du drone du GitHub de dronekit
def arm_and_takeoff(master, aTargetAltitude):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""

	print("Basic pre-arm checks")
	# Don't try to arm until autopilot is ready
	while not is_armable(master):
		print(" Waiting for vehicle to initialise...")

	time.sleep(1)
	print("Arming motors")

	# Copter should arm in GUIDED mode
	set_mode(master,"GUIDED")
	master.arducopter_arm()  # On arme le drone

	# Confirm vehicle armed before attempting to take off
	master.motors_armed_wait()
	print("Drone armé")

	print("Taking off!")
	# Décollage
	master.mav.command_long_send(master.target_system, master.target_component,
								 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
								 0, 0, 0, 0, 0, 0, aTargetAltitude
								 )
	print(f"Décollage à {aTargetAltitude}m")

	# Attente de l'altitude cible
	while True:
		msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
		alt = msg.relative_alt / 1000.0  # Convertir en mètres
		print(f"Altitude : {alt:.1f}m")
		if alt >= aTargetAltitude * 0.95:  # Seuil de 95%
			print("Altitude atteinte !")
			break
		time.sleep(0.2)

# Fonction pour récupérer l'état de la batterie
def get_battery_cap(master):
	""" Récupère et affiche les informations de batterie """
	msg = master.recv_match(type='BATTERY_STATUS', blocking=True)
	if not msg:
		print("Impossible de lire l'état de la batterie.")
		return

	#voltage = msg.voltage_battery / 1000  # Convertir en volts
	#current = msg.current_battery / 100   # Convertir en ampères
	remaining = msg.battery_remaining  # En pourcentage

	#print(f"Tension : {voltage:.2f}V")
	#print(f"Courant : {current:.2f}A")
	#print(f"Charge restante : {remaining}%")

	return remaining

# Fonction pour aller à un waypoint sans attendre l'arrivée du drone
def simple_goto(master, latitude, longitude, altitude):
    """ Envoie le drone vers une position GPS en mode GUIDED """

    # Vérifier si le drone est bien en mode GUIDED
    set_mode(master, "GUIDED")

    # Envoyer une commande directe de déplacement
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Position relative au sol
        int(0b110111111000),  # Masque pour contrôler X, Y, Z sans vitesse ni accélération
        int(latitude * 1e7), int(longitude * 1e7), altitude,  # Coordonnées
        0, 0, 0,  # Vitesses X, Y, Z (non utilisées ici)
        0, 0, 0,  # Accélérations (non utilisées)
        0, 0  # Yaw, yaw rate (non utilisés)
    )

    #print(f"[SIMPLE GOTO] En route vers {latitude}, {longitude}, {altitude}m")


# Fonction pour aller à un waypoint en attendant d'être arrivé
def goto(master, latitude, longitude, altitude):
	""" Envoie le drone vers une position GPS en mode GUIDED """

	# Vérifier si le drone est bien en mode GUIDED
	set_mode(master,"GUIDED")

	# Envoyer la commande MAVLink NAV_WAYPOINT
	master.mav.mission_item_send(
		master.target_system, master.target_component,
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
		msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
		if msg:
			current_lat = msg.lat / 1e7
			current_lon = msg.lon / 1e7
			current_alt = msg.relative_alt / 1000.0

			print(f"Position actuelle : {current_lat}, {current_lon}, {current_alt}m")

			# Vérification si on est proche du waypoint
			if abs(current_lat - latitude) < 0.00005 and abs(current_lon - longitude) < 0.00005 and abs(
					current_alt - altitude) < 0.5:
				print("Waypoint atteint !")
				break
