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

##Parametres nuage##
Seuil_Critique = 100  # Seuil critique qui declenche la procedure de fin de cartographie du nuage

Seuil_entree = 15
Seuil_sortie = 60

larg = 400
sample_y = 100
longu = 1000
sample_x = 225



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


#### Donnees en entrees du programme à rentrer ou une valeur par défaut est donnee ######

parser = argparse.ArgumentParser()
arg1 = parser.add_argument('--altitude', type=float, default=10)
arg2 = parser.add_argument('--angle', type=float, default=85)
arg3 = parser.add_argument('--Min_battery', type=float, default=-10)
args = vars(parser.parse_args())

print("Altitude: %s" % arg1)
print("Angle: %s" % arg2)
print("Batt_min: %s" % arg3)

#### Drone #########

try:
    batt_min = args["Min_battery"]
except:
    batt_min = arg3.default


try:
    ang_ellipse = args["angle"]
except:
    ang_ellipse = arg2.default

##Parametres drone##
try:
    alt = args["altitude"]
except:
    alt = arg3.default
# vehicle.parameters['RTL_ALT'] = alt


# Point GPS du coeur du nuage de fumee
#centre = Position(lat_deg=48.629687, lon_deg = 7.787335, relative_alt_m = alt)
centre = Position(lat_deg=48.631846, lon_deg = 7.783315, relative_alt_m = alt)


########## Création de la carte ##########
m = folium.Map(
        location=[centre.lat_deg, centre.lon_deg],
        zoom_start=17,
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr='Esri'
    )

############################ Connexion a la simulation ########################
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


def get_mode(vehicle):
    """ Récupère et affiche le mode de vol actuel """
    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
    if not msg:
        print("Impossible de récupérer le mode.")
        return None
    while msg.type != 1: # MAV_TYPE_QUADROTOR
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)

    mode_id = msg.custom_mode
    mode_name = None

    for name, mode in vehicle.mode_mapping().items():
        if mode == mode_id:
            mode_name = name
            break

    if mode_name:
        print(f"Mode actuel : {mode_name}")
    else:
        print(f"Mode inconnu (ID: {mode_id})")

    return mode_name


def get_heading():
    """
    Récupère le cap (heading) actuel du drone en degrés via MAVLink.
    Args:
        master : connexion pymavlink (mavutil.mavlink_connection)
    Returns:
        heading (float) : cap en degrés (0-360°)
    """
    # Attend un message HEADING (timeout 1s)
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg is None:
        print("Pas de message HEADING reçu")
        return None

    # heading est en centièmes de degré dans certains firmwares, sinon en degré float
    # Ici on suppose float en degrés
    heading_deg = msg.hdg / 100  # 0-360°
    print("Heading : ", heading_deg)

    return heading_deg

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


def goto(latitude, longitude, altitude):
	""" Envoie le drone vers une position GPS en mode GUIDED """

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


def arm_and_takeoff_quadplane_guided(aTargetAltitude):
    """
    Décolle un QuadPlane en mode QGUIDED jusqu'à une altitude donnée.
    """

    print("Basic pre-arm checks")
    while not is_armable():
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Setting QGUIDED mode...")
    set_mode("QGUIDED")  # Utiliser le mode QGUIDED spécifique aux QuadPlanes

    print("Arming motors")
    master.arducopter_arm()  # Fonctionne aussi pour les quadplanes
    master.motors_armed_wait()
    print("Quadplane armé")

    print(f"Décollage vertical à {aTargetAltitude}m en QGUIDED...")

    # Envoi de la commande de décollage MAV_CMD_NAV_TAKEOFF
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,  # Params 1-4 inutilisés ici
        0, 0,        # Lat, Lon (non spécifiés = rester sur place)
        aTargetAltitude  # Altitude cible
    )

    # Attente que l'altitude soit atteinte
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            alt = msg.relative_alt / 1000.0  # Conversion mm → m
            print(f"Altitude : {alt:.1f}m")

            if alt >= aTargetAltitude * 0.95:
                print("Altitude cible atteinte !")
                break
        time.sleep(0.5)

    return True


def set_heading(cap_deg, distance_m=500):
	"""
    Calcule un point GPS situé à `distance_m` mètres dans la direction `cap_deg` à partir
    de la position actuelle du drone.

    Args:
        master : connexion pymavlink (pour récupérer la position actuelle)
        cap_deg (float) : cap en degrés (0 = nord, 90 = est, etc.)
        distance_m (float) : distance en mètres (par défaut 50)

    Returns:
        (new_lat, new_lon) : tuple des coordonnées GPS du point cible en degrés décimaux
    """

	pos=get_position()

	# Constantes
	R = 6378137  # rayon moyen de la Terre en mètres

	# Convertir cap en radians
	bearing = math.radians(cap_deg)

	# Calcul du nouveau point
	lat1 = math.radians(pos.lat_deg)
	lon1 = math.radians(pos.lon_deg)

	lat2 = math.asin(math.sin(lat1) * math.cos(distance_m / R) +
					 math.cos(lat1) * math.sin(distance_m / R) * math.cos(bearing))

	lon2 = lon1 + math.atan2(math.sin(bearing) * math.sin(distance_m / R) * math.cos(lat1),
							 math.cos(distance_m / R) - math.sin(lat1) * math.sin(lat2))

	# Convertir en degrés décimaux
	new_lat = math.degrees(lat2)
	new_lon = math.degrees(lon2)

	goto(new_lat,new_lon,alt)
	print("Heading réglé à ", cap_deg)

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


def NuageRectangle(longu, larg, sample_x, sample_y, pt, ligne, col, sigma_x=35.0, sigma_y=20.0, C0=100.0):
    """
    Génère un nuage en forme de panache de fumée avec concentration selon une gaussienne 2D.

    longu, larg : dimensions (mètres) de la zone
    sample_x : nombre d'échantillons sur l'axe X (longueur)
    sample_y : nombre d'échantillons sur l'axe Y (largeur)
    pt : point GPS central (objet avec lat_deg et lon_deg)
    ligne, col : indices du centre dans la matrice (int)
    sigma_x, sigma_y : étalement gaussien (contrôle la forme du panache)
    C0 : concentration max au centre du nuage
    """

    GPSLat = np.zeros((sample_x, sample_y))
    GPSLon = np.zeros((sample_x, sample_y))
    SensorValue = np.zeros((sample_x, sample_y))

    # Constantes pour conversion degrés <-> mètres (approximations)
    Klat = 111320.0  # mètres par degré latitude (varie peu)
    Klon = 111320.0 * math.cos(math.radians(pt.lat_deg))  # correction selon latitude

    for i in range(sample_x):
        for j in range(sample_y):
            # Calcul GPS centré autour de (ligne, col)
            lat = pt.lat_deg + ((longu * (i - ligne) / sample_x) / Klat)
            lon = pt.lon_deg + ((larg * (j - col) / sample_y) / Klon)
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


# Centre du nuage genere
ptGPS = Position(lat_deg=(centre.lat_deg - (longu / 2) / Klat), lon_deg=(centre.lon_deg - (larg / 2) / Klon), relative_alt_m=alt)
Max_capteur = 100  # Valeur maximale du capteur pour le nuage

ok_i = 1
ok_j = 1

# Generation nuage
GPSLat, GPSLon, SensorValue = NuageRectangle(longu, larg, sample_x, sample_y, centre, sample_x // 2, sample_y // 2)
#SensorValue = SensorVal(SensorValue, sample, Max_capteur)
print("Min SensorValue:", np.min(SensorValue))
print("Max SensorValue:", np.max(SensorValue))
print(f"Min: {SensorValue.min():.2f}, Max: {SensorValue.max():.2f}, Mean: {SensorValue.mean():.2f}, Std: {SensorValue.std():.2f}")
afficher_nuage_sur_carte(GPSLat, GPSLon, SensorValue)

########## Procedure de decollage  du drone ################
set_mode("QSTABILIZE")
# Le drone decolle
while True:
    print("en attente de guided")
    print("mode: %s" % get_mode(master))
    if (get_mode(master) == "GUIDED"):  # Attente du passage du mode "AUTO" manuellement sur QGC
        print("Decollage")
        arm_and_takeoff_quadplane_guided(alt)  # Decollage
        print("Lancement de la mission")
        break


time.sleep(0.25)
print("Passage en mode Plane")
set_mode("GUIDED")
print("Début du vol en ligne droite vers le centre")
goto(centre.lat_deg, centre.lon_deg, alt)
time.sleep(10)
set_heading(0)
while True:
    get_heading()
    time.sleep(1)

print("Fin du vol !")
time.sleep(2)


