#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math
import numpy as np
from pymavlink import mavutil
import argparse
import folium
import pymavlink_utils as utils
from pymavlink_utils import Position
from folium import plugins
import matplotlib.cm as cm
import matplotlib.colors as colors
from branca.colormap import linear


##################### PARAMETRES ###########################################

#### Donnees en entrees du programme à rentrer ou une valeur par défaut est donnee ######

parser = argparse.ArgumentParser()
arg1 = parser.add_argument('--NbPts', type=int, default=25)
arg2 = parser.add_argument('--Rayon', type=float, default=18)
arg3 = parser.add_argument('--it', type=int, default=4)
arg4 = parser.add_argument('--fact_dist', type=float, default=2)
arg5 = parser.add_argument('--VitesseCercle', type=float, default=3)
arg6 = parser.add_argument('--GAxe', type=float, default=1)
arg7 = parser.add_argument('--PAxe', type=float, default=1)
arg8 = parser.add_argument('--Vent', type=float, default=0)
arg9 = parser.add_argument('--altitude', type=float, default=10)
ar10 = parser.add_argument('--angle', type=float, default=-180)
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
Seuil_Critique = 1500000000  # Seuil critique qui declenche la procedure de fin de cartographie du nuage

Seuil_entree_debut = 700000
Seuil_entree = 500000
Seuil_sortie = 500000

sample = 150

#################################################################################################################

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
	rayon = rayon
	theta = theta * (math.pi / 180)
	direction = (math.pi / 180) * direction
	position = utils.get_position(master)
	#Lat_act = position.lat_deg + rayon * a * math.cos(direction) / Klat
	Lat_act = position.lat_deg + rayon * b * math.cos(direction) / Klat
	#Lon_act = position.lon_deg + rayon * b * math.sin(direction) / Klon
	Lon_act = position.lon_deg + rayon * a * math.sin(direction) / Klon
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
	dist = 0
	aLoc = Position(lat_deg=coords[0][0], lon_deg=coords[0][1], relative_alt_m=alt)
	for i in range(0, len(coords)):
		LocPoint = Position(lat_deg=coords[i][0], lon_deg=coords[i][1], relative_alt_m=alt)
		if get_distance_metres(LocPoint, aLoc) > dist:
			dist = get_distance_metres(LocPoint, aLoc)
	return dist


# Fonction pour determiner le cap entre deux points GPS (bearing)
# aLocation1 : position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
# aLocation2 : position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
def get_bearing(aLocation1, aLocation2):
	"""
    Calcule le bearing entre deux points GPS
    Utilise la formule standard de géodésie
    """
	# Conversion en radians
	lat1_rad = math.radians(aLocation1.lat_deg)
	lat2_rad = math.radians(aLocation2.lat_deg)
	lon1_rad = math.radians(aLocation1.lon_deg)
	lon2_rad = math.radians(aLocation2.lon_deg)

	# Différence de longitude
	delta_lon = lon2_rad - lon1_rad

	# Formule du bearing
	y = math.sin(delta_lon) * math.cos(lat2_rad)
	x = (math.cos(lat1_rad) * math.sin(lat2_rad) -
		 math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))

	# Calcul du bearing en radians puis conversion en degrés
	bearing_rad = math.atan2(y, x)
	bearing_deg = math.degrees(bearing_rad)

	# Normalisation entre 0 et 360 degrés
	bearing_deg = (bearing_deg + 360) % 360

	return bearing_deg


def position_relative_pos(centre, pos, direction_degres):
	"""
    Détermine si aLocation2 se trouve à droite ou à gauche de la droite
    formée par le vecteur porteur de la direction passant par aLocation1.

    Args:
        centre: position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
        pos: position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
        direction_degres: direction en degrés (0° = Nord, 90° = Est, etc.)

    Returns:
        int: 1 : droite, -1 = gauche, ou 0 = sur_la_ligne
    """
	# Extraction des coordonnées depuis les objets LocationGlobalRelative
	lat1 = centre.lat_deg
	lon1 = centre.lon_deg
	lat2 = pos.lat_deg
	lon2 = pos.lon_deg

	# Utilisation de la même logique que votre fonction get_bearing
	# pour être cohérent avec votre système de coordonnées
	off_x = lon2 - lon1  # Décalage en longitude (Est-Ouest)
	off_y = lat2 - lat1  # Décalage en latitude (Nord-Sud)

	# Conversion de la direction en radians
	# Votre convention : 0° = Nord, 90° = Est (sens horaire)
	direction_rad = math.radians(direction_degres)

	# Vecteur directeur de la droite dans le système de coordonnées
	# Compatible avec votre fonction get_bearing
	vx = math.cos(math.radians(90 - direction_degres))  # Composante Est
	vy = math.sin(math.radians(90 - direction_degres))  # Composante Nord

	# Calcul du produit vectoriel pour déterminer la position relative
	# Si le produit vectoriel est positif : point à gauche
	# Si le produit vectoriel est négatif : point à droite
	# Si le produit vectoriel est nul : point sur la ligne
	produit_vectoriel = vx * off_y - vy * off_x

	# Seuil de tolérance pour considérer qu'un point est sur la ligne
	tolerance = 1e-10

	if abs(produit_vectoriel) < tolerance:
		return 0 # sur la ligne
	elif produit_vectoriel > 0:
		return -1 # à gauche
	else:
		return 1 # à droite

def GenereNuage(npy_3d_path, npy_alt_path, altitude_coupe, centre):
	"""
    Extrait une coupe à altitude fixe d'un champ 3D de concentration,
    convertit en µg/m³, et géoréférence le résultat autour du point GPS
    correspondant au maximum de concentration.

    Args:
        npy_3d_path (str): Chemin vers le fichier .npy contenant le champ 3D (X, Y, Z).
        npy_alt_path (str): Chemin vers le fichier .npy contenant les altitudes (m) des couches du champ 3D.
        altitude_coupe (float): Altitude à laquelle extraire la coupe horizontale.
        centre (dict): Dictionnaire {"lat_deg": float, "lon_deg": float} du centre GPS.
        longueur_zone (float): Longueur de la zone (axe nord-sud), en mètres.
        largeur_zone (float): Largeur de la zone (axe est-ouest), en mètres.

    Returns:
        GPSLat (2D np.array), GPSLon (2D np.array), SensorValue (2D np.array, µg/m³)
    """

	CONVERSION_KG_M3_TO_UG_M3 = 1e9

	# Charger le tableau 3D
	data_3d = np.load(npy_3d_path)  # Dimensions (X, Y, Z) = (500, 200, 200)

	# Charger le tableau des altitudes
	altitudes = np.load(npy_alt_path)

	# Trouver l'index de l'altitude la plus proche
	altitudes_array = np.array(altitudes)
	idx_alt = np.argmin(np.abs(altitudes_array - altitude_coupe))

	# Extraire la coupe 2D à cette altitude
	slice_2d_kg_m3 = data_3d[:, :, idx_alt]  # Dimensions (X, Y)

	# Convertir en µg/m³
	SensorValue = slice_2d_kg_m3 * CONVERSION_KG_M3_TO_UG_M3

	# Obtenir les dimensions
	rows, cols = SensorValue.shape
	longueur_zone = rows  # m
	largeur_zone = cols  # m

	# Trouver le centre de concentration maximale
	ligne_centre, col_centre = np.unravel_index(np.argmax(SensorValue), SensorValue.shape)

	# Création des grilles GPS
	GPSLat = np.zeros_like(SensorValue)
	GPSLon = np.zeros_like(SensorValue)

	# Calculer les pas de discrétisation en mètres
	pas_nord_sud = longueur_zone / rows  # mètres par ligne
	pas_est_ouest = largeur_zone / cols  # mètres par colonne

	for i in range(rows):
		for j in range(cols):
			# Calcul des coordonnées ENU pour chaque point de la grille
			# Décalage par rapport au centre de concentration maximale
			delta_north = (i - ligne_centre) * pas_nord_sud
			delta_east = (j - col_centre) * pas_est_ouest

			# Coordonnées ENU du point courant (relatives au centre souhaité)
			east = delta_east
			north = delta_north
			up = altitude_coupe

			# Conversion vers GPS en utilisant le centre GPS souhaité comme référence
			lat, lon, alt = utils.enu_to_gps(east, north, up, centre.lat_deg, centre.lon_deg, altitude_coupe)

			GPSLat[i, j] = lat
			GPSLon[i, j] = lon

	return GPSLat, GPSLon, SensorValue

def afficher_nuage_sur_carte(GPSLat, GPSLon, SensorValue, save_path='nuage_map.html', seuil_affichage=None):
	"""
    Affiche le nuage de particules sur une carte Folium

    Args:
        GPSLat, GPSLon: Coordonnées GPS (arrays 2D)
        SensorValue: Valeurs de concentration (array 2D)
        save_path: Chemin de sauvegarde
        seuil_affichage: Seuil minimum pour afficher un point (None = automatique)
    """
	global m

	# Statistiques pour diagnostic
	print(f"=== STATISTIQUES DU NUAGE ===")
	print(f"Valeurs min/max: {SensorValue.min():.2e} / {SensorValue.max():.2e}")
	print(f"Nombre de valeurs > 0: {np.sum(SensorValue > 0)}")
	print(f"Nombre total de points: {SensorValue.size}")

	# Définir le seuil d'affichage
	if seuil_affichage is None:
		# Prendre 30% de la valeur max comme seuil automatique
		seuil_affichage = SensorValue.max() * 0.02
		print(f"Seuil automatique: {seuil_affichage:.2e}")

	# Filtrer les valeurs significatives
	mask = SensorValue > seuil_affichage
	nb_points_affiches = np.sum(mask)
	print(f"Nombre de points à afficher: {nb_points_affiches}")

	if nb_points_affiches == 0:
		print("ATTENTION: Aucun point à afficher avec ce seuil!")
		return

	# Définir les bornes de la colormap : échelle log
	vmin = seuil_affichage
	vmax = SensorValue.max()

	# Créer la colormap
	color_map = linear.plasma.scale(vmin, vmax)
	color_map.caption = 'Concentration en particules fines (µg/m³)'

	# Ajouter les points significatifs
	points_ajoutes = 0
	for i in range(GPSLat.shape[0]):
		for j in range(GPSLat.shape[1]):
			val = SensorValue[i, j]
			if val > seuil_affichage:  # Afficher seulement les valeurs significatives
				folium.CircleMarker(
					location=[GPSLat[i, j], GPSLon[i, j]],
					radius=4,  # Légèrement plus grand pour plus de visibilité
					fill=True,
					fill_color=color_map(val),
					color=color_map(val),
					fill_opacity=0.7,  # Plus opaque
					weight=1,
					popup = f"Concentration: {val:.2e} µg/m³"
				).add_to(m)
				points_ajoutes += 1

	print(f"Points effectivement ajoutés: {points_ajoutes}")

	# Ajouter la légende
	color_map.add_to(m)

	# Ajouter un marqueur au centre pour référence
	folium.Marker(
		location=[centre.lat_deg, centre.lon_deg],
		popup="Centre du nuage",
		icon=folium.Icon(color='red', icon='info-sign')
	).add_to(m)

	# Sauvegarder
	m.save(save_path)
	print(f"Carte sauvegardée : {save_path}")

# On retourne une liste pour les latitudes, longitude et les valeurs de capteurs


# Fonction qui va lire et selectionner la valeur de capteur la plus proche 
# et retourner True lorque le drone est a proximite de cette derniere et que le seuil est depasse
def ToutDroitStop(GPSLat, GPSLon, SensorValue, Seuil_entree_debut, alt):
    position = utils.get_position(master)

    # Création de matrices de distance entre chaque point GPS du nuage et la position actuelle
    delta_lat = (GPSLat - position.lat_deg) * Klat  # en mètres
    delta_lon = (GPSLon - position.lon_deg) * Klon  # en mètres

    distances = np.sqrt(delta_lat**2 + delta_lon**2)

    # Trouve l’indice du point le plus proche
    i_min, j_min = np.unravel_index(np.argmin(distances), distances.shape)

    dist_min = distances[i_min, j_min]
    valeur = SensorValue[i_min, j_min]

    # Vérifie les conditions
    if valeur > Seuil_entree_debut and dist_min < 5:
        return True

    return False



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

def Traj(Seuil_entree, Seuil_sortie, SensorValue, centre, angle, RayonCercle, NbPoints, a, b,
				   fact_dist, fact_temps, theta, alt, Seuil_Critique):

	coords = Cercle(RayonCercle, angle, NbPoints, a, b, theta)
	print("Traj - ")
	position = utils.get_position(master)
	dist, rang = NearestCoord(coords, position)
	coordsF = Tri(coords, rang, -1)
	points = []
	for lat, lon in coordsF:
		points.append(Position(lat_deg=lat, lon_deg=lon, relative_alt_m=alt))

	ok_j = 1
	pt = 0
	ok = 0

	while True:
		if utils.get_battery_cap(master) < batt_min:
			return False, True

		if pt == len(points) - 1:
			pt = 0
			return False, False

		if ok == 0:
			utils.simple_goto(master,points[pt].lat_deg, points[pt].lon_deg, points[pt].relative_alt_m)
			start = time.time()
			ok = 1

		nxt = pt - 1
		if nxt == -1:
			nxt = 1
		D = get_distance_metres(points[nxt], points[pt])

		if get_distance_metres(utils.get_position(master), points[pt]) < fact_dist * D or time.time() - start > D / fact_temps:
			pt = pt + 1
			ok = 0

		#print("--- Début While recherche point + proche ---")
		position = utils.get_position(master)

		# Conversion en arrays numpy pour vectorisation
		GPSLat_array = np.array(GPSLat)
		GPSLon_array = np.array(GPSLon)
		SensorValue_array = np.array(SensorValue)

		# Calcul vectorisé des distances
		distances = np.full_like(GPSLat_array, np.inf)

		for i in range(GPSLat_array.shape[0]):
			for j in range(GPSLat_array.shape[1]):
				try:
					distances[i, j] = get_distance_metres(
						position,
						Position(lat_deg=GPSLat_array[i, j], lon_deg=GPSLon_array[i, j], relative_alt_m=alt)
					)
				except:
					distances[i, j] = np.inf

		# Trouve l'index du minimum
		Indice = np.unravel_index(np.argmin(distances), distances.shape)
		print("Concentration PM2.5 : ", SensorValue_array[Indice])
		#print("Distance : ", distances[Indice])
		#print("ok_j = ", ok_j)

		if (SensorValue_array[Indice] > Seuil_Critique and distances[Indice] < 15):
			print("Seuil critique dépassé ! Concentration  = ", SensorValue_array[Indice])
			return True, True

		if (SensorValue_array[Indice] > Seuil_entree and distances[Indice] < 15 and ok_j == 2):
			print("Deuxième sortie ", SensorValue_array[Indice])
			ok_j = 1
			return False, True

		elif (SensorValue_array[Indice] < Seuil_sortie and ok_j == 1 and distances[Indice] < 15):
			ok_j = 2



#Réccupération de l'origine du drone pour le repère ENU local

origine = utils.get_position(master)

# Generation nuage
GPSLat, GPSLon, SensorValue = GenereNuage(fichier_concentration, fichier_altitude, zslice, centre)


print("Min SensorValue:", np.min(SensorValue))
print("Max SensorValue:", np.max(SensorValue))
print(f"Min: {SensorValue.min():.2f}, Max: {SensorValue.max():.2f}, Mean: {SensorValue.mean():.2f}, Std: {SensorValue.std():.2f}")
afficher_nuage_sur_carte(GPSLat, GPSLon, SensorValue, seuil_affichage=500000)

#set_param('RTL_ALT', alt * 100)
# vehicle.parameters['WP_YAW_BEHAVIOR'] = 0 # Angle de lacet (yaw) constant lors vol
utils.set_param(master, 'WP_YAW_BEHAVIOR',0)  # On définit WP_YAW_BEHAVIOR à 0 pour avoir un Angle de lacet (yaw) constant lors vol

########## Procedure de decollage  du drone ################
utils.set_mode(master, "STABILIZE")
# Le drone decolle
while True:
	print("En attente de auto ...")
	if (utils.get_mode(master) == "AUTO"):  # Attente du passage du mode "AUTO" manuellement sur QGC
		utils.set_param(master, 'WP_YAW_BEHAVIOR',
						0)  # On définit WP_YAW_BEHAVIOR à 0 pour avoir un Angle de lacet (yaw) constant lors vol
		print("Lancement de la mission")
		print("Decollage")
		utils.arm_and_takeoff(master, alt)  # Decollage
		break
	time.sleep(0.25)

# Le drone se deplace en direction de point GPS dans le nuage
# vehicle.simple_goto(centre, groundspeed=VitesseDrone)
utils.simple_goto(master, centre.lat_deg, centre.lon_deg, centre.relative_alt_m)

while True:
	if ToutDroitStop(GPSLat, GPSLon, SensorValue, Seuil_entree_debut, alt) == True:
		break
print("Nuage détecté !")
# vehicle.channels.overrides['3'] = 1500 #Poussee vertical du drone
#set_throttle_override(1500)
#set_mode("LOITER")  # Passage du mode "LOITER" le drone garde son altitude et sa position
#time.sleep(2)

while True:
	if utils.get_battery_cap(master) < batt_min:
		# Si le niveau de batterie est trop faible on lance la procedure de fin
		break
	utils.set_mode(master, "GUIDED")
	# Passage en mode "GUIDED" pour parcourir les points GPS de l'ellipse
	#time.sleep(1)
	tour = False
	a_inc = 0
	b_inc = 0
	ang_inc = 0
	while True:  # Parcours des points GPS de l'ellipse
		position = utils.get_position(master)
		bearing = get_bearing(position, centre)
		#cote_drone = position_relative_pos(centre, position, theta) # détermine si le drone se trouve vers la droite ou la gauche du nuage
		#print("Coté drone : ", cote_drone)
		#angle = (bearing + ang_ellipse + ang_inc) % 360  # Calcul de l'angle d'orientation de l'ellipse
		# angle = (bearing + 180 + ang_inc) % 360  # Calcul de l'angle d'orientation de l'ellipse
		# angle = (bearing + 90 + ang_inc) % 360  # Calcul de l'angle d'orientation de l'ellipse
		# angle = (theta + (90 * cote_drone) + ang_inc) % 360  # Calcul de l'angle d'orientation de l'ellipse
		angle = theta
		print("Bearing ellipse : ", bearing)
		print("Angle ellipse : ", angle)
		# fonction de parcours des points GPS
		seuil_crit, tour = Traj(Seuil_entree, Seuil_sortie, SensorValue, centre, angle,
								RayonCercle, NbPoints, a + a_inc, b + b_inc, fact_dist, fact_temps, theta, alt,
								Seuil_Critique)
		if tour == True or seuil_crit == True:
			# Si le seuil du capteur est depasse ou que le seuil critique est depasse on stoppe le parcours des points de l'ellipse
			print("--- Tour terminé ou seuil dépassé ! ---")
			break
		if utils.get_battery_cap(master) < batt_min:
			# Si le niveau de batterie est trop faible on stoppe le parcours des points de l'ellipse
			print("--- Batterie faible, veuillez la recharger ---")
			break
		### Ajustements parametres ellipse lorsque le drone est perdu ##############
		print("--- Ajustement des paramètres de l'ellipse ---")
		a_inc = a_inc + (a + a_inc) * 0.3  # Ajustement du facteur du petit axe
		b_inc = b_inc + (b + b_inc) * 0.3  # Ajustement du facteur du grand axe
		ang_inc = ang_inc - 90  # Ajustement de l'angle d'orientation de l'ellipse
	if seuil_crit == True:
		# Si le seuil critique est depasse on passe a la procedure de fin de suivi de frontiere
		print("--- Seuil critique dépassé !!! ---")
		break
	#set_mode("LOITER")  # Mode "LOITER" lorsque le seuil du capteur est depasse
	time.sleep(0.1)

################# Procedure de fin de suivi de frontiere ######################
print("Passage en mode RTL, Arret du programme dans 10 secondes ...")
utils.set_mode(master, "RTL")
time.sleep(10)
print("Fin du programme.")

