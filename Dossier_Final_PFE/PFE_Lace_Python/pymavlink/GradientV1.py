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

# Classe Position
Position = namedtuple('Position', ['lat_deg', 'lon_deg', 'relative_alt_m'])
trajectory_points = []  # Liste des tuples (lat, lon)

# --- Constantes de configuration ---
STEP_SIZE = 10.0  # Distance du pas en mètres
MIN_MOVEMENT = 0.5  # Seuil pour considérer un déplacement significatif (en mètres)


# --- Variables d'état ---
prev_pos_enu = None
prev_concentration_pm = None
prev_direction = None
prev_gradC = None


##Parametres nuage##
Seuil_Critique = 100  # Seuil critique qui declenche la procedure de fin de cartographie du nuage

Seuil_entree_debut = 15
Seuil_entree = 12
Seuil_sortie = 60

seuil_crit = False

larg = 120
longu = 300

sample = 250


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


def set_param(param_id, value):
	""" Change un paramètre du drone via MAVLink """
	master.mav.param_set_send(
		master.target_system, master.target_component,
		param_id.encode(),  # Nom du paramètre
		float(value),  # Valeur du paramètre
		mavutil.mavlink.MAV_PARAM_TYPE_REAL32  # Type de paramètre
	)
	print(f"Paramètre {param_id} réglé à {value}")


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
			#concentration_pm = get_closest_sensor_value(GPSLat, GPSLon, SensorValue, alt)
			#print("Concentration en PM2.5 : ", concentration_pm)


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


def estimate_gradient_incremental(p0, p1, c0, c1):
	"""Estime le gradient local en utilisant deux points et leurs concentrations."""
	delta_p = p1 - p0
	norm_sq = np.dot(delta_p, delta_p)
	if norm_sq < 1e-6:
		return None
	dC = c1 - c0
	return (dC / norm_sq) * delta_p


def update_gradient(grad_old, p0, p1, c0, c1, alpha=0.8):
	delta_p = p1 - p0
	norm_sq = np.dot(delta_p, delta_p)
	if norm_sq < 1e-6:
		return grad_old

	dC_measured = c1 - c0
	dC_predicted = np.dot(grad_old, delta_p)

	# Estimation du gradient basée sur cette mesure uniquement
	error = dC_measured - dC_predicted
	direction = delta_p / np.sqrt(norm_sq)
	grad_from_measurement = grad_old + (error / np.sqrt(norm_sq)) * direction

	# TON mélange alpha
	return (1 - alpha) * grad_old + alpha * grad_from_measurement


def orthogonal_direction(gradC, prev_dir=None):
	if np.linalg.norm(gradC) < 1e-6:
		return prev_dir if prev_dir is not None else np.array([1.0, 0.0])

	v1 = np.array([-gradC[1], gradC[0]])
	v2 = np.array([gradC[1], -gradC[0]])

	if prev_dir is None:
		return v1 / np.linalg.norm(v1)

	chosen = v1 if np.dot(prev_dir, v1) > 0 else v2
	return chosen / np.linalg.norm(chosen)  # Normalise le bon vecteur !


def compute_next_target(pos_enu, direction, ref):
	"""Calcule le prochain point cible en ENU et le convertit en GPS."""
	target_enu = pos_enu + STEP_SIZE * direction
	target_lat, target_lon, _ = enu_to_gps(target_enu[0], target_enu[1], 0.0, ref.lat_deg, ref.lon_deg, ref.relative_alt_m)
	return target_lat, target_lon

def estimate_gradient_from_points(points):
    """
    Estimation du gradient via régression linéaire sur les points (x,y,C)
    points: liste de tuples [(x, y, C), ...]
    Retourne: gradient numpy array [dC/dx, dC/dy]
    """
    if len(points) < 2:
        return np.array([0.0, 0.0])  # Pas assez de points

    x0, y0, C0 = points[0] # Référence
    A = []
    b = []
    for (x, y, C) in points[1:]:
        A.append([x - x0, y - y0])
        b.append(C - C0)
    A = np.array(A)
    b = np.array(b).reshape(-1, 1)

    # Résolution par moindres carrés
    grad, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    grad = grad.flatten()
    return grad


def init_gradient(r_init=5):
	"""
    Effectue un sondage dans 4 directions cardinales pour estimer un gradient initial.
    ref: Position GPS d'origine
    r_init: distance du sondage (en mètres)
    """
	pos_centre = get_position()
	centre_enu = get_enu_position(pos_centre)
	mesures = []

	directions = {
		"est": np.array([r_init, 0]),
		"ouest": np.array([-r_init, 0]),
		"nord": np.array([0, r_init]),
		"sud": np.array([0, -r_init])
	}

	for dir_name, offset in directions.items():
		target_enu = centre_enu + offset
		lat, lon = enu_to_gps(target_enu[0], target_enu[1], 0.0, pos_centre.lat_deg, pos_centre.lon_deg, pos_centre.relative_alt_m)[:2]

		print(f"Déplacement vers {dir_name} pour sondage initial.")
		goto(lat, lon, pos_centre.relative_alt_m)
		time.sleep(1)  # Temps pour stabiliser
		pos = get_position()
		conc = get_closest_sensor_value(GPSLat, GPSLon, SensorValue, pos.relative_alt_m)
		pos_enu = get_enu_position(pos)
		mesures.append((pos_enu[0], pos_enu[1], conc))

	# Ajout de la position centrale

	goto(pos_centre.lat_deg, pos_centre.lon_deg, pos_centre.relative_alt_m)
	conc_centre = get_closest_sensor_value(GPSLat, GPSLon, SensorValue, pos_centre.relative_alt_m)
	pos_centre_enu = get_enu_position(pos_centre)
	mesures.insert(0, (pos_centre_enu[0], pos_centre_enu[1], conc_centre))

	# Régression linéaire
	gradC = estimate_gradient_from_points(mesures)

	return gradC, pos_centre_enu, conc_centre

# Points cardinaux definis pour la procedure de fin de suivi de frontiere

est = Position(lat_deg=centre.lat_deg, lon_deg=centre.lon_deg + larg / Klon, relative_alt_m=alt)
ouest = Position(lat_deg=centre.lat_deg, lon_deg=centre.lon_deg - larg / Klon, relative_alt_m=alt)
nord = Position(lat_deg=centre.lat_deg + larg / Klat, lon_deg=centre.lon_deg, relative_alt_m=alt)
sud = Position(lat_deg=centre.lat_deg - larg / Klat, lon_deg=centre.lon_deg, relative_alt_m=alt)

# Centre du nuage genere
ptGPS = Position(lat_deg=(centre.lat_deg - (longu / 2) / Klat), lon_deg=(centre.lon_deg - (larg / 2) / Klon), relative_alt_m=alt)
Max_capteur = 100  # Valeur maximale du capteur pour le nuage

ok_i = 1
ok_j = 1


# ----- Avant la boucle principale -----

# On initialise les positions précédentes à None
prev_pos_enu = None
prev_conc = None
prev_direction = None

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

print("En route vers le centre du nuage ...")

while True:
	if ToutDroitStop(GPSLat, GPSLon, SensorValue, Seuil_entree_debut, alt) == True:
		break
print("Fin du déplacement vers le centre du nuage, seuil cible atteint")

time.sleep(2)

print("Procédure d'initialisation du gradient local de concentration ...")
# Sondage initial pour estimer le gradient
gradC, pos_enu, conc = init_gradient(r_init=5)
direction = orthogonal_direction(gradC)
prev_direction = direction
prev_conc = conc
print("Gradient initial calculé !")
print(gradC)

while True:
	print("--- Méthode du gradient constant ---")
	# On réccupère la position d'origine
	pos = get_position()

	# On réccupère la concentration en pm d'origine
	concentration_pm = get_closest_sensor_value(GPSLat, GPSLon, SensorValue, pos.relative_alt_m)
	print("Concentration mesurée : ", concentration_pm)

	# On convertit les coordonnées dans le repère ENU local (x,y,z)
	pos_enu = get_enu_position(pos)

	if prev_pos_enu is None:
		# Première itération
		prev_pos_enu = pos_enu
		prev_concentration_pm = concentration_pm
		print("DEBUG concentration pm :", concentration_pm)
		prev_gradC = gradC
		continue #Se rend à l'itération suivante

	# Estimation du gradient
	#gradC = estimate_gradient_incremental(prev_pos_enu, pos_enu, prev_concentration_pm, concentration_pm)
	print("prev_gradC = ",prev_gradC)
	print("GradC = ", gradC)
	print("C0",prev_concentration_pm)
	print("C1",concentration_pm)
	gradC = update_gradient(prev_gradC,prev_pos_enu, pos_enu, prev_concentration_pm, concentration_pm)

	# Calcul de la direction orthogonale et du waypoint
	direction = orthogonal_direction(gradC, prev_direction)
	target_lat, target_lon = compute_next_target(pos_enu, direction, origine)

	# Envoyer la commande au drone
	goto(target_lat, target_lon, alt)

	# Mise à jour des états
	prev_pos_enu = pos_enu
	prev_concentration_pm = concentration_pm
	prev_direction = direction
	prev_gradC = gradC


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
