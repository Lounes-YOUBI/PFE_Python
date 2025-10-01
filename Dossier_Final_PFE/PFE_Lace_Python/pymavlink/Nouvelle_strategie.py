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

from pyproj import Transformer  # Bibliothèque pour la conversion GPS / ENU

# Classe Position
Position = namedtuple('Position', ['lat_deg', 'lon_deg', 'relative_alt_m'])
trajectory_points = []  # Liste des tuples (lat, lon)

gamma = 20
taille_segment = 20

##Parametres nuage##
Seuil_Critique = 100  # Seuil critique qui declenche la procedure de fin de cartographie du nuage

Seuil_entree_debut = 15
Seuil_entree = 12
Seuil_sortie = 60

seuil_crit = False

larg = 120
longu = 300

sample = 150


#### Donnees en entrees du programme à rentrer ou une valeur par défaut est donnee ######

parser = argparse.ArgumentParser()
arg1 = parser.add_argument('--NbPts', type=int, default=35)
arg2 = parser.add_argument('--Rayon', type=float, default=9)
arg3 = parser.add_argument('--it', type=int, default=4)
arg4 = parser.add_argument('--fact_dist', type=float, default=2)
arg5 = parser.add_argument('--VitesseCercle', type=float, default=1.4)
arg6 = parser.add_argument('--GAxe', type=float, default=1.2)
arg7 = parser.add_argument('--PAxe', type=float, default=0.8)
arg8 = parser.add_argument('--Vent', type=float, default=0)
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

def gps_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    """
    Convertit des coordonnées GPS (lat, lon, alt) vers ENU (x, y, z) en mètres,
    par rapport à un point de référence GPS.
    """
    transformer = Transformer.from_crs(
        "epsg:4979",        # WGS84 3D (lat, lon, alt)
        f"+proj=enu +lat_0={ref_lat} +lon_0={ref_lon} +h_0={ref_alt} +units=m +datum=WGS84",
        always_xy=True
    )
    x, y, z = transformer.transform(lon, lat, alt)
    return x, y, z

def enu_to_gps(x, y, z, ref_lat, ref_lon, ref_alt):
    """
    Convertit des coordonnées ENU (x, y, z) vers GPS (lat, lon, alt),
    par rapport à un point de référence GPS.
    """
    transformer = Transformer.from_crs(
        f"+proj=enu +lat_0={ref_lat} +lon_0={ref_lon} +h_0={ref_alt} +units=m +datum=WGS84",
        "epsg:4979",        # WGS84 3D
        always_xy=True
    )
    lon, lat, alt = transformer.transform(x, y, z)
    return lat, lon, alt

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


# Point GPS du coeur du nuage de fumee
# centre=LocationGlobalRelative(48.858114, 7.205582,alt)
#centre = Position(lat_deg=48.858114, lon_deg = 7.205582, relative_alt_m = alt)
centre = Position(lat_deg=48.629687, lon_deg = 7.787335, relative_alt_m = alt)

# Création de la carte

m = folium.Map(
		location=[centre.lat_deg, centre.lon_deg],
		zoom_start=17,
		tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
		attr='Esri'
	)

################################ Connexion a la simulation ########################
print('Connecting to vehicle')

# Connexion en UDP (simu ou radio)
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Attente du premier message HEARTBEAT (confirmation de connexion)
master.wait_heartbeat()
print("Drone connecté")


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

def get_heading():
	""" Récupère le heading actuel du drone le retourne """
	msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
	if not msg:
		print("Erreur : Impossible de récupérer la position.")
		return None

	# Conversion des données reçues
	heading = msg.hdg / 100   # Conversion en centidegrés vers degrés

	return heading

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

	global gamma
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

			# Vérification de la concentration en pm
			concentration_pm = get_closest_sensor_value(GPSLat, GPSLon, SensorValue, alt)
			print("Concentration en PM2.5 : ", concentration_pm)

			if concentration_pm >= Seuil_sortie:
				print("Seuil haut dépassé")
				gamma = -gamma
				break
			elif concentration_pm < Seuil_entree:
				print("Seuil bas dépassé")
				gamma = -gamma
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

def segment_angle_dist(gamma_deg, distance_m):
    """
    Calcule un point GPS situé à une distance et un angle donnés depuis la position actuelle.

    Paramètres :
    - gamma_deg (float) : cap en degrés (0 = nord, 90 = est)
    - distance_m (float) : distance en mètres

    Retour :
    - Position(lat_deg, lon_deg, relative_alt_m)
    """

    R = 6371000  # Rayon moyen de la Terre en mètres

    position = get_position()
    lat = position.lat_deg
    lon = position.lon_deg
    alt = position.relative_alt_m  # Altitude relative conservée

    # Conversion en radians
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    gamma = math.radians(gamma_deg)

    # Formules géodésiques
    lat2 = math.asin(math.sin(lat1) * math.cos(distance_m / R) +
                     math.cos(lat1) * math.sin(distance_m / R) * math.cos(gamma))

    lon2 = lon1 + math.atan2(math.sin(gamma) * math.sin(distance_m / R) * math.cos(lat1),
                             math.cos(distance_m / R) - math.sin(lat1) * math.sin(lat2))

    # Retour en degrés
    lat2_deg = math.degrees(lat2)
    lon2_deg = math.degrees(lon2)

    return Position(lat_deg=lat2_deg, lon_deg=lon2_deg, relative_alt_m=alt)

# Fonction logarithme utilisee pour genere le nuage
def f(x):
	if 1.0*math.log(x)<=0:
		y=0
	else:
		y=1.0*math.log(x)
	return y

def compute_distance(i, j, center_i, center_j, fac_li, fac_col):
    """Calcule la distance pondérée puis applique la fonction f."""
    dist = math.sqrt((center_i - i) ** 2 * fac_li + (center_j - j) ** 2 * fac_col)
    return f(dist)


# Fonction pour generer le nuage
# longu: Longueur en metre de la zone ou le nuage est genere
# larg: Largeur en metre de la zone ou le nuage est genere
# pt : Localisation GPS du coeur du nuage
# ligne : position en X dans la matrice du coeur du nuage
# col : position en Y dans la matrice du coeur du nuage
# fac_liHG,fac_colHG,fac_liHD,fac_colHD,fac_liBG,fac_colBG,fac_liBD,fac_colBD: facteur d'etirement du nuage

def NuageRectangle(longu, larg, sample, pt, ligne, col, sigma_x=20.0, sigma_y=35.0, C0=100.0):
    """
    Génère un nuage en forme de panache de fumée avec concentration selon une gaussienne 2D.

    longu, larg : dimensions (mètres) de la zone
    sample : nombre d'échantillons sur chaque axe (sample x sample)
    pt : point GPS central (objet avec lat_deg et lon_deg)
    ligne, col : indices du centre dans la matrice (int)
    sigma_x, sigma_y : étalement gaussien (contrôle la forme du panache)
    C0 : concentration max au centre du nuage
    """

    GPSLat = np.zeros((sample, sample))
    GPSLon = np.zeros((sample, sample))
    SensorValue = np.zeros((sample, sample))

    # Constantes pour conversion degrés <-> mètres (approximations)
    Klat = 111320.0  # mètres par degré latitude (varie peu)
    Klon = 111320.0 * math.cos(math.radians(pt.lat_deg))  # correction selon latitude

    for i in range(sample):
        for j in range(sample):
            # Calcul GPS centré autour de (ligne, col)
            lat = pt.lat_deg + ((longu * (i - ligne) / sample) / Klat)
            lon = pt.lon_deg + ((larg * (j - col) / sample) / Klon)
            GPSLat[i, j] = lat
            GPSLon[i, j] = lon

            # Coordonnées relatives au centre (ligne,col)
            dx = i - ligne
            dy = j - col

            # Calcul concentration 2D gaussienne anisotrope
            val = C0 * math.exp(-((dx**2) / (2 * sigma_x**2) + (dy**2) / (2 * sigma_y**2)))
            SensorValue[i, j] = val

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

def get_closest_sensor_value(GPSLat, GPSLon, SensorValue, alt):
    """
    Renvoie la valeur de capteur associée au point GPS le plus proche du drone.

    Paramètres:
        GPSLat (2D list): Matrice des latitudes des points
        GPSLon (2D list): Matrice des longitudes des points
        SensorValue (2D list): Matrice des valeurs de capteur
        alt (float): Altitude pour construire les objets Position

    Retours:
        float: Valeur de capteur du point le plus proche
    """
    position = get_position()
    min_dist = float('inf')
    closest_value = None

    for i in range(len(GPSLat)):
        for j in range(len(GPSLat[i])):
            point = Position(lat_deg=GPSLat[i][j], lon_deg=GPSLon[i][j], relative_alt_m=alt)
            dist = get_distance_metres(position, point)

            if dist < min_dist:
                min_dist = dist
                closest_value = SensorValue[i][j]

    return closest_value


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

def Zigzag():
	global gamma, m, trajectory_points

	print("--- ZigZag ---")

	# Calcul du nouveau waypoint
	waypoint = segment_angle_dist(theta + gamma, taille_segment)

	# Ordre au drone de se déplacer vers ce waypoint
	goto(waypoint.lat_deg, waypoint.lon_deg, waypoint.relative_alt_m)

	# Récupérer la position GPS actuelle du drone
	current_pos = get_position()

	# Ajouter le point à la liste
	trajectory_points.append((current_pos.lat_deg, current_pos.lon_deg))

	# Si au moins 2 points, tracer le segment entre les deux derniers sur la carte
	if len(trajectory_points) >= 2:
		start = trajectory_points[-2]
		end = trajectory_points[-1]
		folium.PolyLine(locations=[start, end], color="blue", weight=3).add_to(m)

		# Sauvegarder la carte mise à jour
		m.save("carte_trajectoire.html")


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

# Points cardinaux defini pour la procedure de fin de suivi de frontiere


est = Position(lat_deg=centre.lat_deg, lon_deg=centre.lon_deg + larg / Klon, relative_alt_m=alt)
ouest = Position(lat_deg=centre.lat_deg, lon_deg=centre.lon_deg - larg / Klon, relative_alt_m=alt)
nord = Position(lat_deg=centre.lat_deg + larg / Klat, lon_deg=centre.lon_deg, relative_alt_m=alt)
sud = Position(lat_deg=centre.lat_deg - larg / Klat, lon_deg=centre.lon_deg, relative_alt_m=alt)

# Centre du nuage genere
ptGPS = Position(lat_deg=(centre.lat_deg - (longu / 2) / Klat), lon_deg=(centre.lon_deg - (larg / 2) / Klon), relative_alt_m=alt)
Max_capteur = 100  # Valeur maximale du capteur pour le nuage

ok_i = 1
ok_j = 1

# Generation nuage
GPSLat, GPSLon, SensorValue = NuageRectangle(longu, larg, sample, centre, sample // 2, sample // 2)
#SensorValue = SensorVal(SensorValue, sample, Max_capteur)
print("Min SensorValue:", np.min(SensorValue))
print("Max SensorValue:", np.max(SensorValue))
print(f"Min: {SensorValue.min():.2f}, Max: {SensorValue.max():.2f}, Mean: {SensorValue.mean():.2f}, Std: {SensorValue.std():.2f}")
afficher_nuage_sur_carte(GPSLat, GPSLon, SensorValue)

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
	print("ICI")
	if ToutDroitStop(GPSLat, GPSLon, SensorValue, Seuil_entree_debut, alt) == True:
		break
print("ICI2")
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
		Zigzag()
		if tour == True or seuil_crit == True:
			# Si le seuil du capteur est depasse ou que le seuil critique est depasse on stoppe le parcours des points de l'ellipse
			print("--- Tour terminé ou seuil dépassé ! ---")
			break
		if get_battery_cap() < batt_min:
			# Si le niveau de batterie est trop faible on stoppe le parcours des points de l'ellipse
			print("--- Batterie faible, veuillez la recharger ---")
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
