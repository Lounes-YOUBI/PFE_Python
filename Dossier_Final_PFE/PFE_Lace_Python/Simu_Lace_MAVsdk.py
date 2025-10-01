#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math
import numpy as np
import asyncio
from mavsdk import System
from mavsdk.telemetry import Telemetry
from collections import namedtuple
from pymavlink import mavutil
import argparse

# Classe Position
Position = namedtuple('Position', ['latitude_deg', 'longitude_deg', 'relative_altitude_m'])

drone = System()

alt = 10 # altitude de vol en mètres
rayon_points_GPS = 0.5 # rayon des points à atteindre en mètres

batt_min = 50

Indice = (0, 0)

##Parametres nuage##
Seuil_Critique = 100  # Seuil critique qui declenche la procedure de fin de cartographie du nuage
Seuil_entree_debut = 18
Seuil_entree = 22
Seuil_sortie = 22
larg = 100 * 0.45
longu = 100 * 0.45
sample = 150

absolute_altitude = 0

Klat = 1.109462521e5  # Facteur de conversion de metres en coordonnees en latitude
Klon = Klat * 2 / 3  # Facteur de conversion de metres en coordonnees en longitude
#theta = theta * (math.pi / 180)
#direction = (math.pi / 180) * direction

# Point GPS du coeur du nuage de fumee
centre = Position(latitude_deg=48.582, longitude_deg=7.7638, relative_altitude_m=alt)

# Points cardinaux defini pour la procedure de fin de suivi de frontiere
est = Position(latitude_deg=centre.latitude_deg, longitude_deg=centre.longitude_deg + larg / Klon,
               relative_altitude_m=alt)  # Point a l'Est
ouest = Position(latitude_deg=centre.latitude_deg, longitude_deg=centre.longitude_deg - larg / Klon,
                 relative_altitude_m=alt)  # Point a l'Ouest
nord = Position(latitude_deg=centre.latitude_deg + larg / Klat, longitude_deg=centre.longitude_deg,
                relative_altitude_m=alt)  # Point au Nord
sud = Position(latitude_deg=centre.latitude_deg - larg / Klat, longitude_deg=centre.longitude_deg,
               relative_altitude_m=alt)  # Point au Sud

ptGPS = Position(latitude_deg=centre.latitude_deg - (longu / 2) / Klat,
                     longitude_deg=centre.longitude_deg - (larg / 2) / Klon, relative_altitude_m=alt)

################################ Connexion à la simulation #####################################
#export PX4_HOME_LAT=48.581918
#export PX4_HOME_LON=7.763784
#export PX4_HOME_ALT=0
################################



async def connect_drone():
    # Créer une instance de drone
    print('Connecting to vehicle')

    # Connexion à votre drone (adresse système UDP ici, remplacez par l'adresse correcte si nécessaire)
    await drone.connect(system_address="udp://:14540")

    # Vous pouvez maintenant interagir avec le drone
    print("Drone connecté")

    #status_text_task = asyncio.ensure_future(print_status_text(drone))

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            position = await getPositionDrone(drone)
            print("lat : ", position.latitude_deg, " °, long : ", position.longitude_deg, " °, alt : ", position.relative_altitude_m, " m.")
            break
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

################################# Fonctions #######################################

# Fonction de decollage du drone du GitHub de dronekit
async def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    # while not vehicle.is_armable:
    # print(" Waiting for vehicle to initialise...")
    # time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    await drone.action.arm()

    # Confirm vehicle armed before attempting to take off
    # while not vehicle.armed:
    # print(" Waiting for arming...")
    # time.sleep(1)

    print("Taking off!")
    #await drone.action.set_takeoff_altitude(aTargetAltitude)
    await drone.action.set_takeoff_altitude(aTargetAltitude+2)
    print("Set !")
    await drone.action.takeoff()  # Take off to target altitude
    print("Debug")
    position = await getPositionDrone(drone)
    print(" Altitude: ", position.relative_altitude_m, " m.")
    while True:
        position = await getPositionDrone(drone)
        print(" Altitude: ", position.relative_altitude_m, " m.")
        # Break and return from function just below target altitude.
        if position.relative_altitude_m >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

async def simple_goto(lat, long, alt):
    goal = Position(latitude_deg=lat, longitude_deg = long, relative_altitude_m=alt)

    # Lecture de l'altitude absolue du terrain
    position = await getPositionDrone(drone)
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

    await drone.action.goto_location(lat, long, absolute_altitude + alt, 0)
    while True:
        #flight_mode = await anext(drone.telemetry.flight_mode())
        #print("FlightMode:", flight_mode)
        position = await getPositionDrone(drone)
        dist = get_distance_metres(position, goal)
        print(" Distance au prochain point : ", dist, " m.")
        # Break and return from function just below target altitude.
        if dist <= rayon_points_GPS:
            print("Point GPS cible atteint")
            break
        time.sleep(0.1)

async def landDrone():
    print("Landing drone")
    await drone.action.land()

# Fonction pour récupérer la position du drone et la transformer en objet Position
async def getPositionDrone(drone):
    # Récupérer la position actuelle via la télémétrie
    position_data = await anext(drone.telemetry.position())
        #async for position_data in drone.telemetry.position():
        # Créer un objet Position personnalisé
    position_object = Position(
            latitude_deg=position_data.latitude_deg,
            longitude_deg=position_data.longitude_deg,
            relative_altitude_m=position_data.relative_altitude_m
        )
    return position_object


# Fonction de calcul de distance entre 2 points GPS
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.latitude_deg - aLocation1.latitude_deg
    dlong = aLocation2.longitude_deg - aLocation1.longitude_deg
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


# Fonction qui calcule les coordonnees des points GPS du cercle/ellipse a realiser
# rayon: Rayon du cercle
# direction : Direction cardinale dans laquelle l'ellipse est direigee par rapport au drone
# pointsCercle : Nombre de points generes sur le cercle/ellipse
# a,b: Facteur d'allongement du petit axe et grand axe de l'ellipse
# theta : Angle de rotation de l'ellipse

async def Cercle(rayon, direction, pointsCercle, a, b, theta):
    Klat = 1.109462521e5  # Facteur de conversion de metres en coordonnees en latitude
    Klon = Klat * 2 / 3  # Facteur de conversion de metres en coordonnees en longitude
    rayon = rayon
    theta = theta * (math.pi / 180)
    direction = (math.pi / 180) * direction
    position = await getPositionDrone(drone)
    Lat_act = position.latitude_deg + rayon * a * math.cos(direction) / Klat
    Lon_act = position.longitude_deg + rayon * b * math.sin(direction) / Klon
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
        LocPoint = Position(latitude_deg=coords[i][0], longitude_deg=coords[i][1], relative_altitude_m=alt)
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
    aLoc = Position(latitude_deg=coords[0][0], longitude_deg=coords[0][1], relative_altitude_m=alt)
    for i in range(0, len(coords)):
        LocPoint = Position(latitude_deg=coords[i][0], longitude_deg=coords[i][1], relative_altitude_m=alt)
        if get_distance_metres(LocPoint, aLoc) > dist:
            dist = get_distance_metres(LocPoint, aLoc)
    return dist


# Fonction pour determiner le cap entre deux pointqs GPS (bearing)
# aLocation1 : position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
# aLocation2 : position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.longitude_deg - aLocation1.longitude_deg
    off_y = aLocation2.latitude_deg - aLocation1.latitude_deg
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing  # retourne le cap entre deux points GPS


# Fonction logarithme utilisee pour genere le nuage
def f(x):
    if 0.5 * math.log(x) <= 0:
        y = 0
    else:
        y = 0.5 * math.log(x)
    return y


# Fonction pour generer le nuage
# longu: Longueur en metre de la zone ou le nuage est genere
# larg: Largeur en metre de la zone ou le nuage est genere
# pt : Localisation GPS du coeur du nuage
# ligne : position en X dans la matrice du coeur du nuage
# col : position en Y dans la matrice du coeur du nuage
# fac_liHG,fac_colHG,fac_liHD,fac_colHD,fac_liBG,fac_colBG,fac_liBD,fac_colBD: facteur d'etirement du nuage

def NuageRectangle(longu, larg, sample, pt, ligne, col, fac_liHG, fac_colHG, fac_liHD, fac_colHD, fac_liBG,
                   fac_colBG, fac_liBD, fac_colBD):
    column, row = sample, sample
    # coords=[]
    GPSLat = np.ones((column, row))
    GPSLon = np.ones((column, row))
    SensorValue = np.ones((column, row))
    for i in range(0, sample):
        for j in range(0, sample):
            lat = ptGPS.latitude_deg + ((longu * (i / sample)) / Klat)
            lon = ptGPS.longitude_deg + ((larg * (j / sample)) / Klon)
            GPSLat[i][j] = lat
            GPSLon[i][j] = lon
            if (sample % 2) == 0:  # Si echantillonage paire
                if i <= (ligne) - 1 and j <= (col) - 1:
                    y1 = (ligne) - 1
                    y2 = (col) - 1
                    dist = math.sqrt(pow(y1 - i, 2) * fac_liHG + pow(y2 - j, 2) * fac_colHG)
                    dist = f(dist)
                    SensorValue[i][j] = dist
                elif i <= (ligne) - 1 and j > (col) - 1:
                    y1 = (ligne) - 1
                    y2 = (col)
                    dist = math.sqrt(pow(y1 - i, 2) * fac_liHD + pow(y2 - j, 2) * fac_colHD)
                    dist = f(dist)
                    SensorValue[i][j] = dist
                elif i > (ligne) - 1 and j <= (col) - 1:
                    y1 = (ligne)
                    y2 = (col) - 1
                    dist = math.sqrt(pow(y1 - i, 2) * fac_liBG + pow(y2 - j, 2) * fac_colBG)
                    dist = f(dist)
                    SensorValue[i][j] = dist
                else:
                    y1 = (ligne)
                    y2 = (col)
                    dist = math.sqrt(pow(y1 - i, 2) * fac_liBD + pow(y2 - j, 2) * fac_colBD)
                    dist = f(dist)
                    SensorValue[i][j] = dist
            else:  # Si echantillonage impaire
                if i == ligne and j == col:
                    SensorValue[i][j] = 0
                elif i <= (ligne) and j < (col):
                    y1 = (ligne) - 1
                    y2 = (col) - 1
                    dist = math.sqrt(pow(y1 - i, 2) * fac_liHG + pow(y2 - j, 2) * fac_colHG)
                    dist = f(dist)
                    SensorValue[i][j] = dist
                elif i < (ligne) and j >= (col):
                    y1 = (ligne) - 1
                    y2 = (col)
                    dist = math.sqrt(pow(y1 - i, 2) * fac_liHD + pow(y2 - j, 2) * fac_colHD)
                    dist = f(dist)
                    SensorValue[i][j] = dist
                elif i > ligne and j < col:
                    y1 = (ligne)
                    y2 = (col) - 1
                    dist = math.sqrt(pow(y1 - i, 2) * fac_liBG + pow(y2 - j, 2) * fac_colBG)
                    dist = f(dist)
                    SensorValue[i][j] = dist
                else:
                    y1 = (ligne)
                    y2 = (col)
                    dist = math.sqrt(pow(y1 - i, 2) * fac_liBD + pow(y2 - j, 2) * fac_colBD)
                    dist = f(dist)
                    SensorValue[i][j] = dist
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
async def ToutDroitStop(GPSLat, GPSLon, SensorValue, Seuil_entree_debut, alt):
    print("ToutDroitStop")
    i = 0
    Dist_min = 100000
    position = await getPositionDrone(drone)
    Indice = (0, 0)
    print(SensorValue[Indice[0]][Indice[1]])
    # On calcul a chaque instant le point GPS du nuage le plus proche du drone
    while i < len(GPSLat):
        j = 0
        while j < len(GPSLon):
            Dist_act = get_distance_metres(position,
                                           Position(latitude_deg=GPSLat[i][j], longitude_deg=GPSLon[i][j],
                                                    relative_altitude_m=alt))
            if Dist_act < Dist_min:
                Dist_min = Dist_act  # Distance avec le pt GPS d unuage le plus proche
                Indice = (i, j)
            # Indice du point du nuage le plus proche du drone a l'instant T dans la base de donnees
            j = j + 1
        i = i + 1
    if SensorValue[Indice[0]][Indice[1]] > Seuil_entree_debut and get_distance_metres(
            position,
            Position(latitude_deg=GPSLat[Indice[0]][Indice[1]], longitude_deg=GPSLon[Indice[0]][Indice[1]],
                     relative_altitude_m=alt)) < 5:
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

async def Traj(VitesseCercle, Seuil_entree, Seuil_sortie, SensorValue, centre, angle, RayonCercle, NbPoints, a, b,
         fact_dist, fact_temps, theta, alt, Seuil_Critique):
    print("Traj - ")
    coords = await Cercle(RayonCercle, angle, NbPoints, a, b, theta)  # Calcul des points de l'ellipse a parcourir
    position = await getPositionDrone(drone)
    dist, rang = NearestCoord(coords, position)
    # Detection du point de l'ellipse le plus proche du drone
    coordsF = Tri(coords, rang, -1)  # Tri des points GPS a parcourir dans l'ordre
    points = []
    for lat, lon in coordsF:
        points.append(Position(latitude_deg=lat, longitude_deg=lon, relative_altitude_m=alt))
    # Ajout des points GPS a parcourir dans une liste de "LocationGlobalRelative"
    ok_j = 1
    pt = 0
    ok = 0

    while True:  # Boucle while
        battery = await anext(drone.telemetry.battery())
        if battery.remaining_percent < batt_min:
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
            print("Déplacement vers le prochain point de la trajectoire")
            await simple_goto(points[pt].latitude_deg, points[pt].longitude_deg, points[pt].relative_altitude_m)
            start = time.time()
            ok = 1
        nxt = pt - 1
        if nxt == -1:
            nxt = 1
        D = get_distance_metres(points[nxt], points[pt])
        # print("D",D)
        position = await getPositionDrone(drone)
        if get_distance_metres(position,
                               points[pt]) < fact_dist * D or time.time() - start > D / fact_temps:
            pt = pt + 1
        ok = 0
    i = 0
    Dist_min = 100000
    while i < len(GPSLat):
        j = 0
        while j < len(GPSLon):
            position = await getPositionDrone(drone)
            Dist_act = get_distance_metres(position,
                                           Position(latitude_deg=GPSLat[i][j], longitude_deg=GPSLon[i][j],
                                                    relative_altitude_m=alt))
            if Dist_act < Dist_min:
                Dist_min = Dist_act
                Indice = (i, j)  # Point GPS du nuage le plus proche du drone a l'instant t
            j = j + 1
        i = i + 1
        position = await getPositionDrone(drone)
    if SensorValue[Indice[0]][Indice[1]] > Seuil_Critique and get_distance_metres(position, Position(
            latitude_deg=GPSLat[Indice[0]][Indice[1]], longitude_deg=GPSLon[Indice[0]][Indice[1]],
            relative_altitude_m=alt)) < 15:
        return True, True  # Return True,True (seuil critique depasse et arret de la traj en ellipse)


    if SensorValue[Indice[0]][Indice[1]] > Seuil_entree and get_distance_metres(position, Position(
            latitude_deg=GPSLat[Indice[0]][Indice[1]], longitude_deg=GPSLon[Indice[0]][Indice[1]],
            relative_altitude_m=alt)) < 15 and ok_j == 2:
        ok_j = 1
        return False, True  # Return False,True (seuil critique pas depasse et arret de la traj en ellipse)

    elif SensorValue[Indice[0]][Indice[1]] < Seuil_sortie and ok_j == 1 and get_distance_metres(position,
         Position(latitude_deg=GPSLat[Indice[0]][Indice[1]], longitude_deg=GPSLon[Indice[0]][Indice[1]],
                  relative_altitude_m=alt)) < 15:

        ok_j = 2
#### Donnees en entrees du programme a rentrer ou une valeur par defaut est donnee ######

async def main():


    await connect_drone()

    parser = argparse.ArgumentParser()
    arg1 = parser.add_argument('--NbPts', type=int, default=35)
    arg2 = parser.add_argument('--Rayon', type=float, default=4)
    arg3 = parser.add_argument('--it', type=int, default=4)
    arg4 = parser.add_argument('--fact_dist', type=float, default=2)
    arg5 = parser.add_argument('--VitesseCercle', type=float, default=1.4)
    arg6 = parser.add_argument('--GAxe', type=float, default=1.2)
    arg7 = parser.add_argument('--PAxe', type=float, default=0.8)
    arg8 = parser.add_argument('--Vent', type=float, default=90)
    arg9 = parser.add_argument('--altitude', type=float, default=10)
    ar10 = parser.add_argument('--angle', type=float, default=85)
    ar11 = parser.add_argument('--Min_battery', type=float, default=50)
    args = vars(parser.parse_args())

    #### Drone #########
    VitesseDrone = 2
    try:
        max_iterations = args["it"]  # Nb d'iterations
    except:
        max_iterations = arg3.default
    try:
        batt_min = args["Min_battery"]
    except:
        batt_min = arg11.default

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
        ang_ellipse = arg10.default

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
    #vehicle.parameters['RTL_ALT'] = alt

    ##Parametres nuage##
    Seuil_Critique = 100  # Seuil critique qui declenche la procedure de fin de cartographie du nuage
    Seuil_entree_debut = 18
    Seuil_entree = 22
    Seuil_sortie = 22
    larg = 100 * 0.45
    longu = 100 * 0.45
    sample = 150

    # Point GPS du coeur du nuage de fumee
    centre = Position(latitude_deg=47.39, longitude_deg=8.54, relative_altitude_m=alt)

    # Points cardinaux defini pour la procedure de fin de suivi de frontiere
    est = Position(latitude_deg=centre.latitude_deg, longitude_deg=centre.longitude_deg + larg / Klon,
                   relative_altitude_m=alt)  # Point a l'Est
    ouest = Position(latitude_deg=centre.latitude_deg, longitude_deg=centre.longitude_deg - larg / Klon,
                     relative_altitude_m=alt)  # Point a l'Ouest
    nord = Position(latitude_deg=centre.latitude_deg + larg / Klat, longitude_deg=centre.longitude_deg,
                    relative_altitude_m=alt)  # Point au Nord
    sud = Position(latitude_deg=centre.latitude_deg - larg / Klat, longitude_deg=centre.longitude_deg,
                   relative_altitude_m=alt)  # Point au Sud

    # Centre du nuage genere
    ptGPS = Position(latitude_deg=centre.latitude_deg - (longu / 2) / Klat,
                     longitude_deg=centre.longitude_deg - (larg / 2) / Klon, relative_altitude_m=alt)
    Max_capteur = 100  # Valeur maximale du capteur pour le nuage

    ok_i = 1
    ok_j = 1
    # Generation nuage
    GPSLat, GPSLon, SensorValue = NuageRectangle(longu, larg, sample, centre, sample / 2, sample / 4,
                                                 10, 40, 10, 1, 10, 40, 10, 1)
    SensorValue = SensorVal(SensorValue, sample, Max_capteur)

    ########## Procedure de decollage  du drone ################
    # vehicle.mode=VehicleMode("STABILIZE")
    # Le drone decolle
    # while True:
    #	print ("en attente de auto")
    #	print ("mode: %s" % vehicle.mode)
    #	if (vehicle.mode == VehicleMode("AUTO")):  # Attente du passage du mode "AUTO" manuellement sur QGC
    #		arm_and_takeoff(alt)#Decollage
    #		break
    #	time.sleep(0.25)
    #	print ("Lancement de la mission")
    #	print ("Decollage")
    # vehicle.parameters['WP_YAW_BEHAVIOR'] = 0 # Angle de lacet (yaw) constant lors vol

    await arm_and_takeoff(alt) # Decollage

    # Le drone se deplace en direction de point GPS dans le nuage
    # vehicle.simple_goto(centre, groundspeed=VitesseDrone)
    position = await getPositionDrone(drone)
    print(" Altitude: ", position.relative_altitude_m, " m.")
    print("Direction le centre du nuage ! ")
    await simple_goto(centre.latitude_deg, centre.longitude_deg, centre.relative_altitude_m)

    #await drone.action.goto_location(centre.latitude_deg, centre.longitude_deg, absolute_altitude + centre.relative_altitude_m, 0)
    #await simple_goto(centre.latitude_deg, centre.longitude_deg, centre.relative_altitude_m)
    

    while True:
        if await ToutDroitStop(GPSLat, GPSLon, SensorValue, Seuil_entree_debut, alt) == True:
            print("ICI4")
            break
    # vehicle.channels.overrides['3'] = 1500 #Poussee vertical du drone
    # vehicle.mode = VehicleMode("LOITER")
    await drone.action.hold() #Passage du mode "LOITER" le drone garde son altitude et sa position
    #await drone.action.hold()
    time.sleep(2)

    print("ICI5")

    while True:
        battery = await anext(drone.telemetry.battery())
        if battery.remaining_percent < batt_min:
            # Si le niveau de batterie est trop faible on lance la procedure de fin
            break
        # vehicle.mode = VehicleMode("GUIDED")
        # Passage en mode "GUIDED" pour parcourir les points GPS de l'ellipse
        time.sleep(1)
        tour = False
        a_inc = 0
        b_inc = 0
        ang_inc = 0
        while True:  # Parcours des points GPS de l'ellipse
            position = await getPositionDrone(drone)
            bearing = get_bearing(position, centre)
            angle = (bearing + ang_ellipse + ang_inc) % 360  # Calcul de l'angle d'orientation de l'ellipse
            # vehicle.parameters['WP_YAW_BEHAVIOR'] = 0 #Angle de lacet (yaw) constant lors du vol
            # fonction de parcours des points GPS
            seuil_crit, tour = await Traj(VitesseCercle, Seuil_entree, Seuil_sortie, SensorValue, centre, angle,
                                    RayonCercle, NbPoints, a + a_inc, b + b_inc, fact_dist, fact_temps, theta, alt,
                                    Seuil_Critique)
            if tour == True or seuil_crit == True:
                # Si le seuil du capteur et depasse ou que le seuil critique est depasse on stoppe le parcours des points de l'ellipse
                break
            battery = await anext(drone.telemetry.battery())
            if battery.remaining_percent < batt_min:
                # Si le niveau de batterie est trop faible on stoppe le parcours des points de l'ellipse
                break
            ### Ajustements parametres ellipse lorsque le drone est perdu ##############
            a_inc = a_inc + (a + a_inc) * 0.3  # Ajustement du facteur du petit axe
            b_inc = b_inc + (b + b_inc) * 0.3  # Ajustement du facteur du grand axe
            ang_inc = ang_inc + 40  # Ajustement de l'angle d'orientation de l'ellipse
        if seuil_crit == True:
            # Si le seuil critique est depasse on passe a la procedure de fin de suivi de frontiere
            break
        # vehicle.mode = VehicleMode("LOITER")#Mode "LOITER" lorsque le seuil du capteur est depasse
        await drone.action.hold() #Passage du mode "LOITER" le drone garde son altitude et sa position

        time.sleep(0.3)

    ################# Procedure de fin de suivi de frontiere ######################
    # On calcule le cap du drone par rapport au point au coeur du nuage
    position = await getPositionDrone(drone)
    bear = get_bearing(centre, position)
    print("bearing :", bear)
    # vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)

    # Le drone se dirige vers le point cardinal defini le plus proche
    if bear > 315 or bear <= 45:
        print("Nord")
        # vehicle.simple_goto(nord, groundspeed=VitesseDrone)
        await simple_goto(nord.latitude_deg, nord.longitude_deg, nord.relative_altitude_m)
        time.sleep(25)
        # vehicle.mode = VehicleMode("LAND")
        landDrone()
    elif bear > 45 and bear <= 135:
        print("Est")
        # vehicle.simple_goto(est, groundspeed=VitesseDrone)
        await simple_goto(est.latitude_deg, est.longitude_deg, est.relative_altitude_m)
        time.sleep(25)
        landDrone()
    elif bear > 135 and bear <= 225:
        print("Sud")
        # vehicle.simple_goto(sud, groundspeed=VitesseDrone)
        await simple_goto(sud.latitude_deg, sud.longitude_deg, sud.relative_altitude_m)
        time.sleep(25)
        landDrone()
    else:
        print("Ouest")
        # vehicle.simple_goto(ouest, groundspeed=VitesseDrone)
        await simple_goto(ouest.latitude_deg, ouest.longitude_deg, ouest.relative_altitude_m)
        time.sleep(25)
        landDrone()


if __name__ == "__main__":
    asyncio.run(main())