#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math
import numpy as np
from pymavlink import mavutil
from collections import namedtuple
import argparse
import folium
from folium import plugins
import matplotlib.cm as cm
import matplotlib.colors as colors
from branca.colormap import linear

import pymap3d as pm # Bibliothèque pour la conversion GPS / ENU

##################### PARAMETRES ###########################################


#### Donnees en entrees du programme à rentrer ou une valeur par défaut est donnee ######

parser = argparse.ArgumentParser()
arg1 = parser.add_argument('--NbPts', type=int, default=15)
arg2 = parser.add_argument('--Rayon', type=float, default=9)
arg3 = parser.add_argument('--it', type=int, default=4)
arg4 = parser.add_argument('--fact_dist', type=float, default=2)
arg5 = parser.add_argument('--VitesseCercle', type=float, default=1.4)
arg6 = parser.add_argument('--GAxe', type=float, default=1.4)
arg7 = parser.add_argument('--PAxe', type=float, default=0.7)
arg8 = parser.add_argument('--Vent', type=float, default=90)
arg9 = parser.add_argument('--altitude', type=float, default=10)
ar10 = parser.add_argument('--angle', type=float, default=85)
ar11 = parser.add_argument('--Min_battery', type=float, default=-10)
args = vars(parser.parse_args())

print("NbPts: %s" % arg1)
print("Rayon: %s" % arg2)
print("it: %s" % arg3)
print("fact_dist: %s" % arg4)
print("VitCercle: %s" % arg5)
print("GAxe: %s" % arg6)
print("PAxe: %s" % arg7)
print("Vent: %s" % arg8)
print("Altitude: %s" % arg9)
print("Angle: %s" % ar10)
print("Batt_min: %s" % ar11)

#### Drone #########
VitesseDrone = 2
try:
	max_iterations = args["it"]  # Nb d'iterations
except:
	max_iterations = arg3.default
try:
	batt_min = args["Min_battery"]
except:
	batt_min = ar11.default

####### Conversion ########
Klat = 1.109462521e5
Klon = Klat * 2 / 3

########## Parametres ellipse/cercle ##########
try:
	VitesseCercle = args["VitesseCercle"]  # Vitesse Cercle
except:
	VitesseCercle = arg5.default

try:
	RayonCercle = args["Rayon"]  # Rayon du cercle en m
except:
	RayonCercle = arg2.default

try:
	NbPoints = args["NbPts"]  # Points GPS pour un cercle
except:
	NbPoints = arg1.default

try:
	fact_dist = args["fact_dist"]
except:
	fact_dist = arg4.default

fact_temps = 0.8

try:
	ang_ellipse = args["angle"]
except:
	ang_ellipse = ar10.default

try:
	b = args["GAxe"]
except:
	b = arg6.default

try:
	a = args["PAxe"]
except:
	a = arg7.default

try:
	theta = args["Vent"]
except:
	theta = arg8.default
theta = (theta + 90) % 360

##Parametres drone##
try:
	alt = args["altitude"]
except:
	alt = arg9.default
# vehicle.parameters['RTL_ALT'] = alt


##Parametres nuage##
Seuil_Critique = 100  # Seuil critique qui declenche la procedure de fin de cartographie du nuage
#Seuil_entree_debut = 18
#Seuil_entree = 22
#Seuil_sortie = 22
#larg = 100 * 1.5
#longu = 100 * 0.75

Seuil_entree_debut = 15
Seuil_entree = 15
Seuil_sortie = 15
#larg = 100 * 1
#longu = 100 * 1
largeur_nuage = 200
longueur_nuage = 200

sample = 150

#################################################################################################################

# Classe Position
Position = namedtuple('Position', ['lat_deg', 'lon_deg', 'relative_alt_m'])

# --- CENTRE GPS DU NUAGE ---
centre = Position(lat_deg=48.629687, lon_deg = 7.787335, relative_alt_m = alt)

########## Création de la carte ##########
m = folium.Map(
    location=[centre.lat_deg, centre.lon_deg],
    zoom_start=17,
    tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attr='Esri'
)

####### Conversion ########
Klat = 1.109462521e5
Klon = Klat * 2 / 3

def gps_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    """
    Convertit les coordonnées GPS en coordonnées ENU par rapport à une origine.
    """
    x, y, z = pm.geodetic2enu(lat, lon, alt, ref_lat, ref_lon, ref_alt)
    return x, y, z

def enu_to_gps(x, y, z, ref_lat, ref_lon, ref_alt):
    """
    Convertit les coordonnées ENU en GPS par rapport à une origine.
    """
    lat, lon, alt = pm.enu2geodetic(x, y, z, ref_lat, ref_lon, ref_alt)
    return lat, lon, alt


################################ Connexion a la simulation ########################
print('Connecting to vehicle')

# Connexion en UDP (simu ou radio)
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Attente du premier message HEARTBEAT (confirmation de connexion)
master.wait_heartbeat()
print("Drone connecté")

# === Paramètres modele ===
fichier_concentration = "concentration.npy"   # Nom du fichier .npy contenant le tableau C
fichier_altitude = "altitudes.npy"            # Tableau z[] contenant les altitudes
zslice = 7                                    # Altitude à extraire (en mètres)

################################# Fonctions #######################################

def is_armable():
	""" Vérifie si le drone est armable """
	msg = master.recv_match(type='SYS_STATUS', blocking=True)
	prearm_check = msg.onboard_control_sensors_health  # Bitmask du statut
	return prearm_check > 0  # Si tout va bien, la valeur est non nulle


# Fonction pour changer de mode
def set_mode(mode_name):
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


def get_mode():
	""" Récupère et affiche le mode de vol actuel """
	msg = master.recv_match(type='HEARTBEAT', blocking=True)
	if not msg:
		print("Impossible de récupérer le mode.")
		return None

	mode_id = msg.custom_mode  # ID du mode actuel
	mode_name = None

	# Mapper l'ID avec un nom de mode
	for name, mode in master.mode_mapping().items():
		if mode == mode_id:
			mode_name = name
			break

	if mode_name:
		print(f"Mode actuel : {mode_name}")
	else:
		print(f"Mode inconnu (ID: {mode_id})")

	return mode_name


def get_position():
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


def set_param(param_id, value):
	""" Change un paramètre du drone via MAVLink """
	master.mav.param_set_send(
		master.target_system, master.target_component,
		param_id.encode(),  # Nom du paramètre
		float(value),  # Valeur du paramètre
		mavutil.mavlink.MAV_PARAM_TYPE_REAL32  # Type de paramètre
	)
	print(f"Paramètre {param_id} réglé à {value}")


def set_throttle_override(throttle_value):
	""" Envoie une commande manuelle de poussée (Throttle) via RC override """

	# Création d'un tableau pour les 8 premiers canaux RC
	rc_values = [65535] * 8  # 65535 signifie "pas d'override" pour un canal

	# Canal 3 = Throttle (normalement utilisé pour la poussée verticale)
	rc_values[2] = throttle_value  # Les indices sont 0-based, donc 2 = canal 3

	# Envoi de la commande MAVLink
	master.mav.rc_channels_override_send(
		master.target_system, master.target_component,
		*rc_values
	)


# Fonction de decollage du drone du GitHub de dronekit
def arm_and_takeoff(aTargetAltitude):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""

	print("Basic pre-arm checks")
	# Don't try to arm until autopilot is ready
	while not is_armable():
		print(" Waiting for vehicle to initialise...")

	time.sleep(1)
	print("Arming motors")

	# Copter should arm in GUIDED mode
	set_mode("GUIDED")
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
def get_battery_cap():
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
	print(f"Charge restante : {remaining}%")

	return remaining


# Fonction pour aller à un waypoint en attendant d'être arrivé
def goto(latitude, longitude, altitude):
	""" Envoie le drone vers une position GPS en mode GUIDED """

	# Vérifier si le drone est bien en mode GUIDED
	set_mode("GUIDED")

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

# Fonction pour aller à un waypoint sans attendre l'arrivée du drone
def simple_goto(latitude, longitude, altitude):
    """ Envoie le drone vers une position GPS en mode GUIDED """

    # Vérifier si le drone est bien en mode GUIDED
    set_mode("GUIDED")

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

    print(f"[SIMPLE GOTO] En route vers {latitude}, {longitude}, {altitude}m")


# Fonction de calcul de distance entre 2 points GPS
def get_distance_metres(aLocation1, aLocation2):
	dlat = aLocation2.lat_deg - aLocation1.lat_deg
	dlong = aLocation2.lon_deg - aLocation1.lon_deg
	return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


# Fonction qui calcule les coordonnees des points GPS du cercle/ellipse a realiser
# rayon: Rayon du cercle
# direction : Direction cardinale dans laquelle l'ellipse est direigee par rapport au drone
# pointsCercle : Nombre de points generes sur le cercle/ellipse
# a,b: Facteur d'allongement du petit axe et grand axe de l'ellipse
# theta : Angle de rotation de l'ellipse

def Cercle(rayon, direction, pointsCercle, a, b, theta):
	Klat = 1.109462521e5  # Facteur de conversion de metres en coordonnees en latitude
	Klon = Klat * 2 / 3  # Facteur de conversion de metres en coordonnees en longitude
	rayon = rayon
	theta = theta * (math.pi / 180)
	direction = (math.pi / 180) * direction
	position = get_position()
	Lat_act = position.lat_deg + rayon * a * math.cos(direction) / Klat
	Lon_act = position.lon_deg + rayon * b * math.sin(direction) / Klon
	DroneLat = Lat_act
	DroneLon = Lon_act
	coords = []
	for i in range(0, pointsCercle):  # Pour tout les points du cercle
		degrees = (i / pointsCercle) * 360
		radians = (math.pi / 180) * degrees
		x = rayon * a * math.cos(radians)  # Position en metres absolue du point
		y = rayon * b * math.sin(radians)  # Position en metres absolue du point
		x_r = x * math.cos(theta) - y * math.sin(theta)  # Matrice de rotation d'un angle theta
		y_r = x * math.sin(theta) + y * math.cos(theta)  # Matrice de rotation d'un angle theta
		x_rC = x_r / Klat  # Position absolue en latitude (conversion)
		y_rC = y_r / Klon  # Position absolue en longitude (conversion)
		coords.append((Lat_act + x_rC, Lon_act + y_rC))
	# On ajoute a la liste les coordonnees GPS du point par rapport au centre du cercle
	return coords  # On retourne la liste qui contient toutes les coordonnees GPS des points generes


# Fonction qui retourne la distance et la position dans le tableau "coords" du point GPS le plus proche d'une localisation
# coords : Liste avec les coordonnees GPS (latitude,longitude)
# aLoc : position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
def NearestCoord(coords, aLoc):
	alt = 10
	dist = 100000
	for i in range(0, len(coords)):
		# LocPoint=LocationGlobalRelative(coords[i][0],coords[i][1],alt)
		LocPoint = Position(lat_deg=coords[i][0], lon_deg=coords[i][1], relative_alt_m=alt)
		if get_distance_metres(LocPoint, aLoc) < dist:
			dist = get_distance_metres(LocPoint, aLoc)
			rang = i
	return dist, rang


# retourne la distance entre la position du drone et le point GPS le plus proche,
# retourne l'indice dans le tableau du point GPS le plus proche


# Fonction qui tri le tableau des points GPS pour avoir dans le bon ordre les WayPoints
# coords : Liste avec les coordonnees GPS (latitude,longitude)
# i : indice du point GPS le plus proche
# sens : sens de parcours de l'ellipse
def Tri(coords, i, sens):
	coordsF = []
	for j in range(i, len(coords)):
		coordsF.append(coords[j])
	for k in range(0, i):
		coordsF.append(coords[k])
	if sens == -1:  # le drone va dans le sens anti-horaire(sens trigo)
		coordsF.reverse()
	coordsF.append(coordsF[0])
	return coordsF  # Retourne une liste dans l'ordre des points GPS a parcourir dans l'ellipse


# Fonction de calcul du diametre du cercle
# coords : Liste avec les coordonnees GPS (latitude,longitude)	
def Diametre(coords):
	alt = 10
	dist = 0
	# aLoc=LocationGlobalRelative(coords[0][0],coords[0][1],alt)
	aLoc = Position(lat_deg=coords[0][0], lon_deg=coords[0][1], relative_alt_m=alt)
	for i in range(0, len(coords)):
		# LocPoint=LocationGlobalRelative(coords[i][0],coords[i][1],alt)
		LocPoint = Position(lat_deg=coords[i][0], lon_deg=coords[i][1], relative_alt_m=alt)
		if get_distance_metres(LocPoint, aLoc) > dist:
			dist = get_distance_metres(LocPoint, aLoc)
	return dist


# Fonction pour determiner le cap entre deux pointqs GPS (bearing)
# aLocation1 : position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
# aLocation2 : position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
def get_bearing(aLocation1, aLocation2):
	off_x = aLocation2.lon_deg - aLocation1.lon_deg
	off_y = aLocation2.lat_deg - aLocation1.lat_deg
	bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
	if bearing < 0:
		bearing += 360.00
	return bearing  # retourne le cap entre deux points GPS


# Fonction logarithme utilisee pour genere le nuage
def f(x):
	if 1.0*math.log(x)<=0:
		y=0
	else:
		y=1.0*math.log(x)
	return y


def GenereNuage(npy_3d_path, npy_alt_path, altitude_coupe, ptGPS, longueur_zone, largeur_zone):
	"""
    Extrait une coupe à altitude fixe d'un champ 3D de concentration,
    convertit en µg/m³, et géoréférence le résultat autour du point GPS
    correspondant au maximum de concentration.

    Args:
        npy_3d_path (str): Chemin vers le fichier .npy contenant le champ 3D (X, Y, Z).
        npy_alt_path (str): Chemin vers le fichier .npy contenant les altitudes (m) des couches du champ 3D.
        altitude_coupe (float): Altitude à laquelle extraire la coupe horizontale.
        ptGPS (dict): Dictionnaire {"lat_deg": float, "lon_deg": float} du centre GPS.
        longueur_zone (float): Longueur de la zone (axe nord-sud), en mètres.
        largeur_zone (float): Largeur de la zone (axe est-ouest), en mètres.

    Returns:
        GPSLat (2D np.array), GPSLon (2D np.array), SensorValue (2D np.array, µg/m³)
    """

	# Constantes de conversion approximatives (à ajuster selon votre latitude)
	# 1 degré de latitude ≈ 111 km
	# 1 degré de longitude ≈ 111 km * cos(latitude)
	Klat = 111000  # mètres par degré de latitude
	Klon = 111000 * np.cos(np.radians(ptGPS.lat_deg))  # mètres par degré de longitude

	CONVERSION_KG_M3_TO_UG_M3 = 1e9

	# Charger le tableau 3D
	data_3d = np.load(npy_3d_path)  # Dimensions (X, Y, Z) = (500, 200, 200)

	# Charger le tableau des altitudes
	altitudes = np.load(npy_alt_path)

	# Trouver l'index de l'altitude la plus proche
	altitudes_array = np.array(altitudes)
	idx_alt = np.argmin(np.abs(altitudes_array - altitude_coupe))

	# CORRECTION: Extraire la coupe 2D à cette altitude
	# Votre tableau C a les dimensions (X, Y, Z), donc pour extraire Z fixe :
	slice_2d_kg_m3 = data_3d[:, :, idx_alt]  # Dimensions (X, Y)

	# Convertir en µg/m³
	#SensorValue = slice_2d_kg_m3 * CONVERSION_KG_M3_TO_UG_M3
	SensorValue = slice_2d_kg_m3
	# Obtenir les dimensions
	rows, cols = SensorValue.shape

	# Trouver le centre de concentration maximale
	ligne_centre, col_centre = np.unravel_index(np.argmax(SensorValue), SensorValue.shape)

	# Création des grilles GPS
	GPSLat = np.zeros_like(SensorValue)
	GPSLon = np.zeros_like(SensorValue)

	for i in range(rows):
		for j in range(cols):
			# Calcul des décalages en mètres par rapport au centre de concentration max
			delta_lat = ((i - ligne_centre) * (longueur_zone / rows)) / Klat
			delta_lon = ((j - col_centre) * (largeur_zone / cols)) / Klon

			GPSLat[i, j] = ptGPS.lat_deg + delta_lat
			GPSLon[i, j] = ptGPS.lon_deg + delta_lon

	return GPSLat, GPSLon, SensorValue



# On retourne une liste pour les latitudes, longitude et les valeurs de capteurs

# Fonction qui normalise les donnees capteurs du nuage
# SensorValue: Liste contenant les valeurs de capteurs
# sample: echantillonage du nuage genere
# Max : Valeur max du capteur 
def SensorVal(SensorValue, sample, Max):
	for l in range(0, sample):
		for i in range(0, sample):
			SensorValue[l][i] = 1 - (SensorValue[l][i] / np.max(SensorValue))
	SensorValueAff = np.ones((sample, sample))
	for j in range(0, sample):
		for k in range(0, sample):
			SensorValue[j][k] = SensorValue[j][k] * Max
			SensorValueAff[j][k] = round(SensorValue[j][k], 0)
	return SensorValue


# Fonction qui va lire et selectionner la valeur de capteur la plus proche 
# et retourner True lorque le drone est a proximite de cette derniere et que le seuil est depasse
def ToutDroitStop(GPSLat, GPSLon, SensorValue, Seuil_entree_debut, alt):

	i = 0
	Dist_min = 100000
	position = get_position()

	# On calcul a chaque instant le point GPS du nuage le plus proche du drone
	while i < len(GPSLat):
		j = 0
		while j < len(GPSLon):
			Dist_act = get_distance_metres(position,Position(lat_deg=GPSLat[i][j], lon_deg=GPSLon[i][j], relative_alt_m=alt))
			if Dist_act < Dist_min:
				Dist_min = Dist_act  # Distance avec le pt GPS du nuage le plus proche
				Indice = (i, j)
			# Indice du point du nuage le plus proche du drone a l'instant T dans la base de donnees
			j = j + 1
		i = i + 1
	if SensorValue[Indice[0]][Indice[1]] > Seuil_entree_debut and get_distance_metres(position, Position(lat_deg=GPSLat[Indice[0]][Indice[1]],lon_deg=GPSLon[Indice[0]][Indice[1]], relative_alt_m=alt)) < 5:
		return True  # Seuil depasse et le drone est a proximite du point associe a la valeur de capteur
	# Le drone est dans le nuage


# Fonction qui realise la trajectoir en ellipse du drone
# VitesseCercle: Vitesse max en m/s lors de la trajectoire en ellipse
# Seuil_PM1_entree: Seuil d'entree dans le nuage des PM1 
# Seuil_PM1_sortie: Seuil de sortie du nuage des PM1 
# Seuil_PM_2_5_entree: Seuil d'entree dans le nuage des PM2_5
# Seuil_PM_2_5_sortie: Seuil de sortie du nuage des PM2_5 
# centre : Points GPS du coeur du nuage
# angle : Angle de direction cardinal des ellipses
# RayonCercle : taille du rayon en m du cercle/ellipse
# NbPoints : nombre de points GPS generes pour realiser la trajectoire en cercle/ellipse
# a,b: Facteur d'allongement du petit axe et grand axe de l'ellipse
# fact_dist : facteur de distance, lorsque le drone arrive a une certaine distance du point GPS voulu 
#	alors il passe au point suivant de l'ellipse 
# fact_temps : facteur de temps, au bout d'un certain temps si le drone ne parvient pas dans la zone du point GPS voulu
# 	alors il se dirige vers le suivant
# theta : Angle de rotation de l'ellipse
# alt : altitude de vol du drone
# Seuil_Critique: Seuil critique au dessus duquel le capteur risque d'etre endommage

def Traj(VitesseCercle, Seuil_entree, Seuil_sortie, SensorValue, centre, angle, RayonCercle, NbPoints, a, b,
		 fact_dist, fact_temps, theta, alt, Seuil_Critique):
	coords = Cercle(RayonCercle, angle, NbPoints, a, b, theta)  # Calcul des points de l'ellipse a parcourir
	print("Traj - ")
	position = get_position()
	dist, rang = NearestCoord(coords, position)
	# Detection du point de l'ellipse le plus proche du drone
	coordsF = Tri(coords, rang, -1)  # Tri des points GPS a parcourir dans l'ordre
	points = []
	for lat, lon in coordsF:
		# points.append(LocationGlobalRelative(lat,lon,alt))
		points.append(Position(lat_deg=lat, lon_deg=lon, relative_alt_m=alt))
	# Ajout des points GPS a parcourir dans une liste de "LocationGlobalRelative"
	ok_j = 1
	pt = 0
	ok = 0

	while True:  # Boucle while
		if get_battery_cap() < batt_min:
			# Si le niveau de batterie est trop faible on sors de la fonction "Traj()"
			# et on arrete de parcourir les points GPS de l'ellipse
			return False, True
		# Return False,True (seuil critique pas depasse et arret de la traj en ellipse)
		if pt == len(points) - 1:
			# Si le point a parcourir est le dernier du tableau de point
			# alors le drone a parcouru tout les points de l'ellipse sans sortir du nuage (perte de la frontiere)
			pt = 0
			return False, False
		# Return False,False (seuil critique pas depasse et modif des parametres de l'ellipse)
		if ok == 0:
			# Si le point precedent a ete atteint alors le drone se deplace en ligne droite vers le point GPS suivant
			# vehicle.simple_goto(points[pt], groundspeed=VitesseCercle)
			#print("Déplacement vers le prochain point de la trajectoire")
			simple_goto(points[pt].lat_deg, points[pt].lon_deg, points[pt].relative_alt_m)
			start = time.time()
			ok = 1
		nxt = pt - 1
		if nxt == -1:
			nxt = 1
		D = get_distance_metres(points[nxt], points[pt])
		# print("D",D)
		if get_distance_metres(get_position(), points[pt]) < fact_dist * D or time.time() - start > D / fact_temps:
			pt = pt + 1
			ok = 0
		i = 0
		Dist_min = 100000
		print("--- Début While recherche point + proche ---")
		position = get_position()
		while i < len(GPSLat):
			j = 0
			while j < len(GPSLon):
				Dist_act = get_distance_metres(position,Position(lat_deg=GPSLat[i][j], lon_deg=GPSLon[i][j], relative_alt_m=alt))
				if Dist_act < Dist_min:
					Dist_min = Dist_act
					Indice = (i, j)  # Point GPS du nuage le plus proche du drone a l'instant t
				j = j + 1
			i = i + 1
		position = get_position()
		print("FIN WHILE")
		if SensorValue[Indice[0]][Indice[1]] > Seuil_Critique and get_distance_metres(position, Position(
				lat_deg=GPSLat[Indice[0]][Indice[1]], lon_deg=GPSLon[Indice[0]][Indice[1]], relative_alt_m=alt)) < 15:
			print("Seuil critique dépassé !")
			return True, True  # Return True,True (seuil critique depasse et arret de la traj en ellipse)
		if SensorValue[Indice[0]][Indice[1]] > Seuil_entree and get_distance_metres(position, Position(
				lat_deg=GPSLat[Indice[0]][Indice[1]], lon_deg=GPSLon[Indice[0]][Indice[1]],
				relative_alt_m=alt)) < 15 and ok_j == 2:
			print("Deuxième sortie")
			ok_j = 1
			return False, True  # Return False,True (seuil critique pas depasse et arret de la traj en ellipse)
		elif SensorValue[Indice[0]][Indice[1]] < Seuil_sortie and ok_j == 1 and get_distance_metres(position,Position(lat_deg=GPSLat[Indice[0]][Indice[1]],lon_deg=GPSLon[Indice[0]][Indice[1]],relative_alt_m=alt)) < 15:
			ok_j = 2


# Affichage de la carte

def afficher_nuage_sur_carte(GPSLat, GPSLon, SensorValue, save_path='nuage_map.html'):
    global m

    vmin = 0
    vmax = SensorValue.max()

    # Colormap plasma linéaire
    colormap = linear.plasma.scale(vmin, vmax)
    colormap.caption = 'Concentration en particules fines'

    for i in range(GPSLat.shape[0]):
        for j in range(GPSLat.shape[1]):
            val = SensorValue[i, j]
            folium.CircleMarker(
                location=[GPSLat[i, j], GPSLon[i, j]],
                radius=3,
                fill=True,
                fill_color=colormap(val),
                color=None,
                fill_opacity=0.4
            ).add_to(m)

    colormap.add_to(m)
    m.save(save_path)
    print(f"Carte sauvegardée : {save_path}")


def colormap(value, vmin=0, vmax=1):
    norm = colors.Normalize(vmin=vmin, vmax=vmax)
    cmap = cm.get_cmap('plasma')  # ou 'viridis', 'inferno', etc.
    rgb = cmap(norm(value))[:3]
    return f'#{int(rgb[0] * 255):02x}{int(rgb[1] * 255):02x}{int(rgb[2] * 255):02x}'

#Réccupération de l'origine du drone pour le repère ENU local

origine = get_position()

def get_enu_position(pos):
    """Convertit une position GPS en coordonnées ENU 2D (x, y)."""
    global origine
    return np.array(gps_to_enu(pos.lat_deg, pos.lon_deg, pos.relative_alt_m, origine.lat_deg, origine.lon_deg, origine.relative_alt_m)[:2])


# Points cardinaux defini pour la procedure de fin de suivi de frontiere
# est=LocationGlobalRelative(centre.lat,centre.lon+larg/Klon,alt)#Point a l'Est
# ouest=LocationGlobalRelative(centre.lat,centre.lon-larg/Klon,alt)#Point a l'Ouest
# nord=LocationGlobalRelative(centre.lat+larg/Klat,centre.lon,alt)#Point au Nord
# sud=LocationGlobalRelative(centre.lat-larg/Klat,centre.lon,alt)#Point au Sud

est = Position(lat_deg=centre.lat_deg, lon_deg=centre.lon_deg + largeur_nuage / Klon, relative_alt_m=alt)
ouest = Position(lat_deg=centre.lat_deg, lon_deg=centre.lon_deg - largeur_nuage / Klon, relative_alt_m=alt)
nord = Position(lat_deg=centre.lat_deg + largeur_nuage / Klat, lon_deg=centre.lon_deg, relative_alt_m=alt)
sud = Position(lat_deg=centre.lat_deg - largeur_nuage / Klat, lon_deg=centre.lon_deg, relative_alt_m=alt)

# Centre du nuage genere
Max_capteur = 100  # Valeur maximale du capteur pour le nuage

ok_i = 1
ok_j = 1
# Generation nuage
GPSLat, GPSLon, SensorValue = GenereNuage(fichier_concentration, fichier_altitude, zslice, centre, longueur_nuage, largeur_nuage)

SensorValue = SensorVal(SensorValue, sample, Max_capteur)
print("Min SensorValue:", np.min(SensorValue))
print("Max SensorValue:", np.max(SensorValue))
print(f"Min: {SensorValue.min():.2f}, Max: {SensorValue.max():.2f}, Mean: {SensorValue.mean():.2f}, Std: {SensorValue.std():.2f}")
afficher_nuage_sur_carte(GPSLat, GPSLon, SensorValue)

#set_param('RTL_ALT', alt * 100)

########## Procedure de decollage  du drone ################
set_mode("STABILIZE")
# Le drone decolle
while True:
	print("en attente de auto")
	print("mode: %s" % get_mode())
	if (get_mode() == "AUTO"):  # Attente du passage du mode "AUTO" manuellement sur QGC
		arm_and_takeoff(alt)  # Decollage
		break
	time.sleep(0.25)
	print("Lancement de la mission")
	print("Decollage")
	# vehicle.parameters['WP_YAW_BEHAVIOR'] = 0 # Angle de lacet (yaw) constant lors vol
	set_param('WP_YAW_BEHAVIOR',
			  0)  # On définit WP_YAW_BEHAVIOR à 0 pour avoir un Angle de lacet (yaw) constant lors vol

# Le drone se deplace en direction de point GPS dans le nuage
# vehicle.simple_goto(centre, groundspeed=VitesseDrone)
simple_goto(centre.lat_deg, centre.lon_deg, centre.relative_alt_m)

while True:
	if ToutDroitStop(GPSLat, GPSLon, SensorValue, Seuil_entree_debut, alt) == True:
		break
print("Nuage détecté !")
# vehicle.channels.overrides['3'] = 1500 #Poussee vertical du drone
set_throttle_override(1500)
set_mode("LOITER")  # Passage du mode "LOITER" le drone garde son altitude et sa position
time.sleep(2)

while True:
	if get_battery_cap() < batt_min:
		# Si le niveau de batterie est trop faible on lance la procedure de fin
		break
	set_mode("GUIDED")
	# Passage en mode "GUIDED" pour parcourir les points GPS de l'ellipse
	time.sleep(1)
	tour = False
	a_inc = 0
	b_inc = 0
	ang_inc = 0
	while True:  # Parcours des points GPS de l'ellipse
		bearing = get_bearing(get_position(), centre)
		angle = (bearing + ang_ellipse + ang_inc) % 360  # Calcul de l'angle d'orientation de l'ellipse
		# vehicle.parameters['WP_YAW_BEHAVIOR'] = 0 # Angle de lacet (yaw) constant lors vol
		set_param('WP_YAW_BEHAVIOR',
				  0)  # On définit WP_YAW_BEHAVIOR à 0 pour avoir un Angle de lacet (yaw) constant lors vol
		# fonction de parcours des points GPS
		seuil_crit, tour = Traj(VitesseCercle, Seuil_entree, Seuil_sortie, SensorValue, centre, angle,
								RayonCercle, NbPoints, a + a_inc, b + b_inc, fact_dist, fact_temps, theta, alt,
								Seuil_Critique)
		if tour == True or seuil_crit == True:
			# Si le seuil du capteur est depasse ou que le seuil critique est depasse on stoppe le parcours des points de l'ellipse
			print("--- Tour terminé ou seuil dépassé ! ---")
			break
		if get_battery_cap() < batt_min:
			# Si le niveau de batterie est trop faible on stoppe le parcours des points de l'ellipse
			print("--- Batterie faible, veuillez le recharger ---")
			break
		### Ajustements parametres ellipse lorsque le drone est perdu ##############
		print("--- Ajustement des paramètres de l'ellipse ---")
		a_inc = a_inc + (a + a_inc) * 0.3  # Ajustement du facteur du petit axe
		b_inc = b_inc + (b + b_inc) * 0.3  # Ajustement du facteur du grand axe
		ang_inc = ang_inc + 40  # Ajustement de l'angle d'orientation de l'ellipse
	if seuil_crit == True:
		# Si le seuil critique est depasse on passe a la procedure de fin de suivi de frontiere
		print("--- Seuil critique dépassé !!! ---")
		break
	set_mode("LOITER")  # Mode "LOITER" lorsque le seuil du capteur est depasse
	time.sleep(0.3)

################# Procedure de fin de suivi de frontiere ######################
# On calcule le cap du drone par rapport au point au coeur du nuage
bear = get_bearing(centre, get_position())
print("bearing :", bear)
set_mode("GUIDED")
time.sleep(1)

# Le drone se dirige vers le point cardinal defini le plus proche
if bear > 315 or bear <= 45:
	print("Nord")
	# vehicle.simple_goto(nord, groundspeed=VitesseDrone)
	goto(nord.lat_deg, nord.lon_deg, nord.relative_alt_m)
	set_mode("LAND")
elif bear > 45 and bear <= 135:
	print("Est")
	goto(est.lat_deg, est.lon_deg, est.relative_alt_m)
	set_mode("LAND")
elif bear > 135 and bear <= 225:
	print("Sud")
	goto(sud.lat_deg, sud.lon_deg, sud.relative_alt_m)
	set_mode("LAND")
else:
	print("Ouest")
	goto(ouest.lat_deg, ouest.lon_deg, ouest.relative_alt_m)
	set_mode("LAND")
