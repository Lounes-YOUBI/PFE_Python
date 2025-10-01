#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import math
import numpy as np
#from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import pymavlink_utils as utils
from pymavlink_utils import Position
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from math import sin, cos, acos, pi, atan2, asin
import argparse

########## Connexion ####################
# Connect to the Vehicle
print('Connecting to vehicle')
vehicle = mavutil.mavlink_connection('udp:127.0.0.1:14551')

################## Fonctions ###########################

		
# Distance entre 2 points GPS
def get_distance_metres(aLocation1, aLocation2):
	dlat = aLocation2.lat_deg - aLocation1.lat_deg
	dlong = aLocation2.lon_deg - aLocation1.lon_deg
	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
	
	
#Fonction qui calcule les coordonnées des points GPS du cercle/ellipse à réaliser    
def Cercle(rayon,direction,pointsCercle,a,b,theta):
	Klat=1.109462521e5
	Klon=Klat*2/3
	rayon=rayon
	theta=theta*(math.pi/180)
	direction = (math.pi/180)*direction
	pos = utils.get_position(vehicle)
	Lat_act = pos.lat_deg+rayon *a* math.cos(direction)/Klat
	Lon_act = pos.lon_deg+ rayon* b*math.sin(direction)/Klon
	DroneLat=Lat_act
	DroneLon=Lon_act
	coords=[]
	for i in range(0,pointsCercle):
		degrees = (i/pointsCercle)*360
		radians = (math.pi/180)*degrees
		x = rayon *a * math.cos(radians)
		y = rayon  *b* math.sin(radians)
		x_r=x*math.cos(theta)-y*math.sin(theta)
		y_r=x*math.sin(theta)+y*math.cos(theta)
		x_rC=x_r/Klat
		y_rC=y_r/Klon
		coords.append((Lat_act+x_rC,Lon_act+y_rC))	
	return coords
	
#Fonction qui retourne la distance et la position dans le tableau "coords" du point GPS le plus proche d'une localisation 
def NearestCoord(coords,aLoc):
	alt=10
	dist=100000
	for i in range(0,len(coords)):
		LocPoint=Position(lat_deg=coords[i][0],lon_deg=coords[i][1],relative_alt_m=alt)
		if get_distance_metres(LocPoint, aLoc)<dist:
			dist=get_distance_metres(LocPoint, aLoc)
			rang=i
	return dist,rang

#Tri le tableau des points GPS pour avoir dans le bon ordre les WayPoints
def Tri(coords,i,sens):
	coordsF=[]
	for j in range(i,len(coords)):
		coordsF.append(coords[j])
	for k in range(0,i):
		coordsF.append(coords[k])
	if sens==-1:#le drone va dans le sens anti-horaire(sens trigo)
		coordsF.reverse()
	coordsF.append(coordsF[0])
	return coordsF

#Calcul le diamètre du cercle		
def Diametre(coords):
	alt=10
	dist=0
	aLoc=Position(lat_deg=coords[0][0],lon_deg=coords[0][1],relative_alt_m=alt)
	for i in range(0,len(coords)):
		LocPoint=Position(lat_deg=coords[i][0],lon_deg=coords[i][1],relative_alt_m=alt)
		if get_distance_metres(LocPoint, aLoc)>dist:
			dist=get_distance_metres(LocPoint, aLoc)
	return dist

#Détermine le cap du drone
def get_bearing(aLocation1, aLocation2):
	off_x = aLocation2.lon_deg - aLocation1.lon_deg
	off_y = aLocation2.lat_deg - aLocation1.lat_deg
	bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
	if bearing < 0:
		bearing += 360.00
	return bearing
	
#Génére un nuage rectangle avec des points GPS
def f(x):
	if 0.5*math.log(x)<=0:
		y=0
	else:	
		y=0.5*math.log(x)
	return y

def NuageRectangle(longu,larg,sample,pt,ligne,col,fac_liHG,fac_colHG,fac_liHD,fac_colHD,fac_liBG,fac_colBG,fac_liBD,fac_colBD):
	column, row = sample, sample
	#coords=[]
	GPSLat=np.ones((column,row))
	GPSLon=np.ones((column,row))
	SensorValue=np.ones((column, row))
	for i in range (0,sample):
		for j in range (0,sample):
			lat=ptGPS.lat_deg+((longu*(i/sample))/Klat)
			lon=ptGPS.lon_deg+((larg*(j/sample))/Klon)
			GPSLat[i][j]=lat
			GPSLon[i][j]=lon
			if (sample % 2) == 0:
				if i<=(ligne)-1 and j<=(col)-1:
					y1=(ligne)-1
					y2=(col)-1
					dist=math.sqrt(pow(y1-i,2)*fac_liHG+pow(y2-j,2)*fac_colHG)
					dist=f(dist)
					SensorValue[i][j]=dist
				elif i<=(ligne)-1 and j>(col)-1:
					y1=(ligne)-1
					y2=(col)
					dist=math.sqrt(pow(y1-i,2)*fac_liHD+pow(y2-j,2)*fac_colHD)
					dist=f(dist)
					SensorValue[i][j]=dist
				elif i>(ligne)-1 and j<=(col)-1:
					y1=(ligne)
					y2=(col)-1
					dist=math.sqrt(pow(y1-i,2)*fac_liBG+pow(y2-j,2)*fac_colBG)
					dist=f(dist)
					SensorValue[i][j]=dist
				else:
					y1=(ligne)
					y2=(col)
					dist=math.sqrt(pow(y1-i,2)*fac_liBD+pow(y2-j,2)*fac_colBD)
					dist=f(dist)
					SensorValue[i][j]=dist
			else:
				if i==ligne and j==col:
					SensorValue[i][j]=0
				elif i<=(ligne) and j<(col):
					y1=(ligne)-1
					y2=(col)-1
					dist=math.sqrt(pow(y1-i,2)*fac_liHG+pow(y2-j,2)*fac_colHG)
					dist=f(dist)
					SensorValue[i][j]=dist
				elif i<(ligne) and j>=(col):
					y1=(ligne)-1
					y2=(col)
					dist=math.sqrt(pow(y1-i,2)*fac_liHD+pow(y2-j,2)*fac_colHD)
					dist=f(dist)
					SensorValue[i][j]=dist
				elif i>ligne and j<col:
					y1=(ligne)
					y2=(col)-1
					dist=math.sqrt(pow(y1-i,2)*fac_liBG+pow(y2-j,2)*fac_colBG)
					dist=f(dist)
					SensorValue[i][j]=dist
				else:
					y1=(ligne)
					y2=(col)
					dist=math.sqrt(pow(y1-i,2)*fac_liBD+pow(y2-j,2)*fac_colBD)
					dist=f(dist)
					SensorValue[i][j]=dist
	return GPSLat,GPSLon,SensorValue
	
#Normalise les données capteurs du nuage
def SensorVal(SensorValue,sample,Max):
	# Vectorisation de la normalisation
	max_val = np.max(SensorValue)
	SensorValue = 1 - (SensorValue / max_val)
	SensorValue = SensorValue * Max
	# SensorValueAff = np.round(SensorValue, 0)  # Si besoin d'un affichage arrondi
	return SensorValue
			
############### Etat prgm ##################################

def ToutDroitStop(GPSLat, GPSLon, SensorValue, Seuil_entree_debut, alt):
	"""
	Détecte si le drone entre dans le nuage de particules (version vectorisée).
	"""
	print("[ToutDroitStop] Début de la fonction ToutDroitStop (vectorisée)")
	pos = utils.get_position(vehicle)
	# Constantes de conversion (mètres par degré)
	constante_latitude = 1.113195e5
	constante_longitude = 1.113195e5 * 2/3
	# Calcul des distances vectorisées
	deltas_lat = (GPSLat - pos.lat_deg) * constante_latitude
	deltas_lon = (GPSLon - pos.lon_deg) * constante_longitude
	distances = np.sqrt(deltas_lat ** 2 + deltas_lon ** 2)
	# Recherche du point le plus proche
	index_ligne, index_colonne = np.unravel_index(np.argmin(distances), distances.shape)
	distance_min = distances[index_ligne, index_colonne]
	valeur_proche = SensorValue[index_ligne, index_colonne]
	print(f"[ToutDroitStop] Indice min: ({index_ligne}, {index_colonne}), distance_min={distance_min}, valeur_proche={valeur_proche}")
	# Vérification des conditions d'entrée
	if valeur_proche > Seuil_entree_debut and distance_min < 5:
		print(f"[ToutDroitStop] Condition d'arrêt atteinte (valeur capteur={valeur_proche}, distance={distance_min}), return True")
		return True
	else:
		print(f"[ToutDroitStop] Condition NON atteinte (valeur capteur={valeur_proche}, distance={distance_min}), return False")
		return False

def Traj(VitesseCercle,Seuil_entree,Seuil_sortie,SensorValue,centre,angle,RayonCercle,NbPoints,a,b,sens,fact_dist,fact_temps,theta,alt):
	print("[Traj] Début de la fonction Traj")
	coords=Cercle(RayonCercle,angle,NbPoints,a,b,theta)
	print(f"[Traj] Coordonnées générées: {len(coords)} points")
	pos = utils.get_position(vehicle)
	dist,rang=NearestCoord(coords,pos)
	print(f"[Traj] Point le plus proche: rang={rang}, dist={dist}")
	coordsF=Tri(coords,rang,sens)
	points=[]
	for lat,lon in coordsF:
		points.append(Position(lat_deg=lat,lon_deg=lon,relative_alt_m=alt))
	pt=0
	ok=0
	print(f"[Traj] Nombre de points dans la trajectoire: {len(points)}")
	while True:
		print(f"[Traj] pt={pt}, ok={ok}")
		D=get_distance_metres(points[0],points[1])
		if pt==len(points)-1:
			pt=0
			print("[Traj] Fin de la trajectoire, return False")
			return False
		if ok==0:
			print(f"[Traj] Déplacement vers le point {pt}: {points[pt]}")
			utils.simple_goto(vehicle, points[pt].lat_deg, points[pt].lon_deg, points[pt].relative_alt_m)
			start=time.time()
			ok=1
		nxt=pt-1
		if nxt==-1:
			nxt=1
		D=get_distance_metres(points[nxt],points[pt])
		pos = utils.get_position(vehicle)
		print(f"[Traj] Distance au point courant: {get_distance_metres(pos,points[pt])}, D={D}")
		if get_distance_metres(pos,points[pt])<fact_dist*D or time.time()-start>D/fact_temps:
			print(f"[Traj] Passage au point suivant (pt={pt+1})")
			pt=pt+1
			ok=0
		# Vectorisation de la recherche du point du nuage le plus proche
		constante_latitude = 1.113195e5
		constante_longitude = 1.113195e5 * 2/3
		GPSLat_local = globals().get('GPSLat', None)
		GPSLon_local = globals().get('GPSLon', None)
		SensorValue_local = globals().get('SensorValue', None)
		if GPSLat_local is not None and GPSLon_local is not None and SensorValue_local is not None:
			deltas_lat = (GPSLat_local - pos.lat_deg) * constante_latitude
			deltas_lon = (GPSLon_local - pos.lon_deg) * constante_longitude
			distances = np.sqrt(deltas_lat ** 2 + deltas_lon ** 2)
			index_ligne, index_colonne = np.unravel_index(np.argmin(distances), distances.shape)
			valeur_proche = SensorValue_local[index_ligne, index_colonne]
			distance_min = distances[index_ligne, index_colonne]
			if sens==-1:
				print(f"[Traj] sens=-1, SensorValue={valeur_proche}, Seuil_entree={Seuil_entree}")
				if valeur_proche > Seuil_entree and distance_min < 15:
					print("[Traj] Condition d'entrée atteinte, return True")
					return True
			else:
				print(f"[Traj] sens=1, SensorValue={valeur_proche}, Seuil_sortie={Seuil_sortie}")
				if valeur_proche < Seuil_sortie and distance_min < 15:
					print("[Traj] Condition de sortie atteinte, return True")
					return True
					
############### Début du programme #########################
#Pos debut simu: 48.706522600000000,7.734146700000000
pos = utils.get_position(vehicle)
#fichier=open("LogVolLaceV1.txt","w")
#@vehicle.on_attribute('attitude')
#def attitude_listener(self, name, msg): 	
	#fichier.write(str(pos.lat_deg))
	#fichier.write(" ")
	#fichier.write(str(pos.lon_deg))
	#fichier.write(" ")
	#fichier.write(str(pos.relative_alt_m))
	#fichier.write(" ")
	#fichier.write(str(time.time()))
	#fichier.write(";")
	#fichier.write("\n")

#python3 Simu.py --NbPts 35 --Rayon 9 --it 4 --fact_dist 2 --VitesseCercle 1.4


parser=argparse.ArgumentParser()
arg1 = parser.add_argument('--NbPts',type=int, default=35)
arg2= parser.add_argument('--Rayon',type=float, default=9)
arg3= parser.add_argument('--it',type=int, default=4)
arg4= parser.add_argument('--fact_dist',type=float, default=2)
arg5= parser.add_argument('--VitesseCercle',type=float, default=1.4)
arg6= parser.add_argument('--GAxe',type=float, default=1.2)
arg7= parser.add_argument('--PAxe',type=float, default=0.8)
arg8= parser.add_argument('--Vent',type=float, default=180)
arg9= parser.add_argument('--altitude',type=float, default=10)
ar10=parser.add_argument('--angle',type=float, default=85)
args = vars(parser.parse_args())


#Drone
VitesseDrone=3
iterations=1
sens=1
try:
	max_iterations = args["it"] #Nb d'iterations
except:
	max_iterations= arg3.default
	
	
#Conversion
Klat=1.109462521e5
Klon=Klat*2/3

##Paramètres ellipse/cercle##

try:
	VitesseCercle = args["VitesseCercle"] #Vitesse Cercle
except:
	VitesseCercle = arg5.default

	
try:
	RayonCercle = args["Rayon"] #Rayon du cercle en m
except:
	RayonCercle = arg2.default

try:
	NbPoints = args["NbPts"]#Points GPS pour un cercle
except:
	NbPoints = arg1.default
	
try:
	fact_dist = args["fact_dist"]
except:
	fact_dist = arg4.default

fact_temps=0.8

try:
	ang_ellipse=args["angle"]
except:
	ang_ellipse=ar10.default

try:
	b=args["GAxe"]
except:
	b=arg6.default
	
try:
	a=args["PAxe"]
except:
	a=arg7.default
	
try:
	theta=args["Vent"]
except:
	theta=arg8.default
theta=(theta+90)%360

##Paramètres drone##
try:
	alt=args["altitude"]
except:
	alt=arg9.default

#vehicle.parameters['RTL_ALT'] = alt


##Paramètres nuage##
Seuil_entree_debut=18
Seuil_entree=22
Seuil_sortie= 22
larg=100*0.45
longu=100*0.45
sample=150
#centre=LocationGlobalRelative(48.582914, 7.764817, 10)
#centre=LocationGlobalRelative(48.581478, 7.766986,10)
#centre=Position(lat_deg=48.628168,lon_deg=7.784950,relative_alt_m=alt)
centre = Position(lat_deg=48.629687, lon_deg=7.787335, relative_alt_m=alt)
ptGPS=Position(lat_deg=centre.lat_deg-(longu/2)/Klat, lon_deg=centre.lon_deg-(larg/2)/Klon, relative_alt_m=alt)
Max_capteur=100

ok_i=1
ok_j=1
#Génération nuage
GPSLat,GPSLon,SensorValue=NuageRectangle(longu,larg,sample,centre,sample/2,sample/4,10,40,10,1,10,40,10,1)
SensorValue=SensorVal(SensorValue,sample,Max_capteur)


########## Décollage drone ################
utils.set_mode(vehicle, "STABILIZE")
#Le drone décolle
while True:
	print ("en attente de auto")
	print ("mode : ", utils.get_mode(vehicle))
	if utils.get_mode(vehicle) == "AUTO":
		utils.arm_and_takeoff(vehicle, alt)
		break
	time.sleep(0.25)
	print ("Lancement de la mission")
	print ("Decollage")
utils.set_param(vehicle, 'WP_YAW_BEHAVIOR', 0)
		
#Le drone se déplace en direction de point GPS dans le nuage
#vehicle.simple_goto(centre, groundspeed=VitesseDrone)
utils.simple_goto(vehicle, centre.lat_deg, centre.lon_deg, centre.relative_alt_m)	

				
while True:
	if ToutDroitStop(GPSLat,GPSLon,SensorValue,Seuil_entree_debut,alt)==True:	
		print("Arrivé au point de départ du nuage")
		break
#vehicle.channels.overrides['3'] = 1500
utils.set_mode(vehicle, "LOITER")
time.sleep(2)

while True:
	print(f"[Main] Début de la boucle principale, iteration={iterations}")
	if iterations>max_iterations:
		print("[Main] Nombre maximal d'itérations atteint, sortie de la boucle.")
		break
	utils.set_mode(vehicle, "GUIDED")
	time.sleep(1)
	tour=False
	a_inc=0
	b_inc=0
	ang_inc=0
	while True:
		print(f"[Main] Nouvelle trajectoire, a_inc={a_inc}, b_inc={b_inc}, ang_inc={ang_inc}")
		pos = utils.get_position(vehicle)
		bearing=get_bearing(pos, centre)
		angle=(bearing+ang_ellipse+ang_inc)%360
		utils.set_param(vehicle, 'WP_YAW_BEHAVIOR', 0)
		tour=Traj(VitesseCercle,Seuil_entree,Seuil_sortie,SensorValue,centre,angle,RayonCercle,NbPoints,a+a_inc,b+b_inc,sens,fact_dist,fact_temps,theta,alt)
		sens=sens*(-1)
		if tour==True:
			print("[Main] Trajectoire terminée (tour==True), sortie de la boucle interne.")
			break
		a_inc=a_inc*1.3
		b_inc=b_inc*1.3
		ang_inc=ang_inc+40    
	utils.set_mode(vehicle, "LOITER")
	time.sleep(0.3)
	iterations=iterations+1
	print(f"[Main] Fin d'itération, passage à iteration={iterations}")
#vehicle.parameters['WPNAV_SPEED'] = 600 
utils.set_mode(vehicle, "GUIDED")
time.sleep(1)
utils.set_mode(vehicle, "RTL")
