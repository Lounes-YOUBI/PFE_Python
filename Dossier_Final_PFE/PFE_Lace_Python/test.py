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

##Parametres nuage##
Seuil_Critique = 100  # Seuil critique qui declenche la procedure de fin de cartographie du nuage
Seuil_entree_debut = 18
Seuil_entree = 22
Seuil_sortie = 22
larg = 100 * 0.45
longu = 100 * 0.45
sample = 150

Klat = 1.109462521e5  # Facteur de conversion de metres en coordonnees en latitude
Klon = Klat * 2 / 3  # Facteur de conversion de metres en coordonnees en longitude
#theta = theta * (math.pi / 180)
#direction = (math.pi / 180) * direction

# Point GPS du coeur du nuage de fumee
centre = Position(latitude_deg=47.39, longitude_deg=8.54, relative_altitude_m=alt)

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
    await drone.action.set_takeoff_altitude(aTargetAltitude+1)
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
    while True:
        await drone.action.goto_location(lat, long, alt, 0)
        flight_mode = await anext(drone.telemetry.flight_mode())
        print("FlightMode:", flight_mode)
        position = await getPositionDrone(drone)
        dist = get_distance_metres(position, goal)
        print(" Distance au prochain point : ", dist, " m.")
        # Break and return from function just below target altitude.
        if dist <= rayon_points_GPS:
            print("Point GPS cible atteint")
            break
        time.sleep(1)

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


# retourne la distance entre la position du drone et le point GPS le plus proche,
# retourne l'indice dans le tableau du point GPS le plus proche


# Fonction pour determiner le cap entre deux pointqs GPS (bearing)
# aLocation1 : position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
# aLocation2 : position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.longitude - aLocation1.longitude
    off_y = aLocation2.latitude - aLocation1.latitude
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing  # retourne le cap entre deux points GPS

async def main():


    await connect_drone()

    ####### Conversion ########
    Klat = 1.109462521e5
    Klon = Klat * 2 / 3

    # Point GPS du coeur du nuage de fumee
    centre = Position(latitude_deg=47.40, longitude_deg=8.55, relative_altitude_m=alt)


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
    await drone.action.goto_location(centre.latitude_deg, centre.longitude_deg, centre.relative_altitude_m, 0)
    time.sleep(10)
    #await simple_goto(centre.latitude_deg, centre.longitude_deg, centre.relative_altitude_m)

    while True:
        print("Staying connected, press Ctrl-C to exit")
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())