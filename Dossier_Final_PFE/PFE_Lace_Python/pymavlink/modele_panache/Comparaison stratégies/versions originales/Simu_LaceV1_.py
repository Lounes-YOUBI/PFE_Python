#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import math
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from math import sin, cos, acos, pi, atan2, asin
import argparse

########## Connexion ####################
# Connect to the Vehicle
print('Connecting to vehicle')
vehicle = connect('udp:127.0.0.1:14551', wait_ready=True)

################## Fonctions ###########################

# Décollage
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
        
# Distance entre 2 points GPS
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    
    
#Fonction qui calcule les coordonnées des points GPS du cercle/ellipse à réaliser    
def Cercle(rayon,direction,pointsCercle,a,b,theta):
	Klat=1.109462521e5
	Klon=Klat*2/3
	rayon=rayon
	theta=theta*(math.pi/180)
	direction = (math.pi/180)*direction
	Lat_act = vehicle.location.global_relative_frame.lat+rayon *a* math.cos(direction)/Klat
	Lon_act = vehicle.location.global_relative_frame.lon+ rayon* b*math.sin(direction)/Klon
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
		LocPoint=LocationGlobalRelative(coords[i][0],coords[i][1],alt)
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
	aLoc=LocationGlobalRelative(coords[0][0],coords[0][1],alt)
	for i in range(0,len(coords)):
		LocPoint=LocationGlobalRelative(coords[i][0],coords[i][1],alt)
		if get_distance_metres(LocPoint, aLoc)>dist:
			dist=get_distance_metres(LocPoint, aLoc)
	return dist

#Détermine le cap du drone
def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;
    
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
			lat=ptGPS.lat+((longu*(i/sample))/Klat)
			lon=ptGPS.lon+((larg*(j/sample))/Klon)
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
	for l in range(0,sample):
		for i in range(0,sample):
			SensorValue[l][i]=1-(SensorValue[l][i]/np.max(SensorValue))
	SensorValueAff=np.ones((sample,sample))
	for j in range(0,sample):
		for k in range(0,sample):
			SensorValue[j][k]=SensorValue[j][k]*Max
			SensorValueAff[j][k]=round(SensorValue[j][k],0)
	return SensorValue		
			
############### Etat prgm ##################################

def ToutDroitStop(GPSLat,GPSLon,SensorValue,Seuil_entree_debut,alt):
	i=0
	Dist_min=100000
	#On calcul à chaque instant le point GPS du nuage le plus proche du drone
	while i<len(GPSLat):
		j=0
		while j<len(GPSLon):
			Dist_act=get_distance_metres(vehicle.location.global_relative_frame, LocationGlobalRelative(GPSLat[i][j],GPSLon[i][j],alt))
			if Dist_act< Dist_min:
					Dist_min=Dist_act#Distance avec le pt GPS d unuage le plus proche
					Indice=(i,j)#Indice du point du nuage le plus proche du drone à l'instant T dans la base de données
			j=j+1
		i=i+1
	if SensorValue[Indice[0]][Indice[1]]> Seuil_entree_debut and get_distance_metres(vehicle.location.global_relative_frame, LocationGlobalRelative(GPSLat[Indice[0]][Indice[1]],GPSLon[Indice[0]][Indice[1]],alt))<5:
		return True

def Traj(VitesseCercle,Seuil_entree,Seuil_sortie,SensorValue,centre,angle,RayonCercle,NbPoints,a,b,sens,fact_dist,fact_temps,theta,alt):
	coords=Cercle(RayonCercle,angle,NbPoints,a,b,theta)	
	dist,rang=NearestCoord(coords,vehicle.location.global_relative_frame)
	coordsF=Tri(coords,rang,sens)	
	points=[]
	for lat,lon in coordsF:
		points.append(LocationGlobalRelative(lat,lon,alt))
	pt=0
	ok=0
	
	while True:

		D=get_distance_metres(points[0],points[1])	
		if pt==len(points)-1:
			pt=0
			print("return false")
			return False	
		if ok==0:
			vehicle.simple_goto(points[pt], groundspeed=VitesseCercle)
			start=time.time()
			ok=1
		nxt=pt-1
		if nxt==-1:
			nxt=1
		D=get_distance_metres(points[nxt],points[pt])
		if get_distance_metres(vehicle.location.global_relative_frame,points[pt])<fact_dist*D or time.time()-start>D/fact_temps:
			pt=pt+1
			ok=0
		i=0
		Dist_min=100000
		while i<len(GPSLat):
			j=0
			while j<len(GPSLon):
				Dist_act=get_distance_metres(vehicle.location.global_relative_frame, LocationGlobalRelative(GPSLat[i][j],GPSLon[i][j],10))
				if Dist_act< Dist_min:
					Dist_min=Dist_act
					Indice=(i,j)
				j=j+1
			i=i+1	
		#print("sensor :" ,SensorValue[Indice[0]][Indice[1]])
		#print("ok_j",ok_j)
		#print("dist point" ,get_distance_metres(vehicle.location.global_relative_frame, LocationGlobalRelative(GPSLat[Indice[0]][Indice[1]],GPSLon[Indice[0]][Indice[1]],10)))
		if sens==-1:
			if SensorValue[Indice[0]][Indice[1]]> Seuil_entree and get_distance_metres(vehicle.location.global_relative_frame, LocationGlobalRelative(GPSLat[Indice[0]][Indice[1]],GPSLon[Indice[0]][Indice[1]],10))<15:
				print("return true")
				return True
		else:
			if SensorValue[Indice[0]][Indice[1]]< Seuil_sortie and get_distance_metres(vehicle.location.global_relative_frame, LocationGlobalRelative(GPSLat[Indice[0]][Indice[1]],GPSLon[Indice[0]][Indice[1]],10))<15:
				print("return true")
				return True
					
############### Début du programme #########################
#Pos debut simu: 48.706522600000000,7.734146700000000
fichier=open("LogVolLaceV1.txt","w")
@vehicle.on_attribute('attitude')
def attitude_listener(self, name, msg): 	
	fichier.write(str(vehicle.location.global_relative_frame.lat))
	fichier.write(" ")
	fichier.write(str(vehicle.location.global_relative_frame.lon))
	fichier.write(" ")
	fichier.write(str(vehicle.location.global_relative_frame.alt))
	fichier.write(" ")
	fichier.write(str(time.time()))
	fichier.write(";")
	fichier.write("\n")

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
	ang_ellipse=arg10.default

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
vehicle.parameters['RTL_ALT'] = alt


##Paramètres nuage##
Seuil_entree_debut=18
Seuil_entree=22
Seuil_sortie= 22
larg=100*0.45
longu=100*0.45
sample=150
#centre=LocationGlobalRelative(48.582914, 7.764817, 10)
#centre=LocationGlobalRelative(48.581478, 7.766986,10)
centre=LocationGlobalRelative(48.628168,7.784950,alt)
ptGPS=LocationGlobalRelative(centre.lat-(longu/2)/Klat, centre.lon-(larg/2)/Klon, alt)
Max_capteur=100

ok_i=1
ok_j=1
#Génération nuage
GPSLat,GPSLon,SensorValue=NuageRectangle(longu,larg,sample,centre,sample/2,sample/4,10,40,10,1,10,40,10,1)
SensorValue=SensorVal(SensorValue,sample,Max_capteur)


########## Décollage drone ################
vehicle.mode=VehicleMode("STABILIZE")
#Le drone décolle
while True:
	print ("en attente de auto")
	print ("mode: %s" % vehicle.mode)
	if (vehicle.mode == VehicleMode("AUTO")):
		arm_and_takeoff(alt)
		break
	time.sleep(0.25)
	print ("Lancement de la mission")
	print ("Decollage")
vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
		
#Le drone se déplace en direction de point GPS dans le nuage
vehicle.simple_goto(centre, groundspeed=VitesseDrone)	

				
while True:
	if ToutDroitStop(GPSLat,GPSLon,SensorValue,Seuil_entree_debut,alt)==True:	
		break
vehicle.channels.overrides['3'] = 1500
vehicle.mode = VehicleMode("LOITER")
time.sleep(2)

while True:
	print(iterations)
	if iterations>max_iterations:
		break
		
	vehicle.mode = VehicleMode("GUIDED")
	time.sleep(1)
	tour=False
	a_inc=0
	b_inc=0
	ang_inc=0
	while True:
		bearing=get_bearing(vehicle.location.global_relative_frame, centre)
		angle=(bearing+ang_ellipse+ang_inc)%360
		vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
		tour=Traj(VitesseCercle,Seuil_entree,Seuil_sortie,SensorValue,centre,angle,RayonCercle,NbPoints,a+a_inc,b+b_inc,sens,fact_dist,fact_temps,theta,alt)
		sens=sens*(-1)
		if tour==True:
			break
		a_inc=a_inc*1.3
		b_inc=b_inc*1.3
		ang_inc=ang_inc+40	
	vehicle.mode = VehicleMode("LOITER")
	time.sleep(0.3)
	iterations=iterations+1
vehicle.parameters['WPNAV_SPEED'] = 600 
vehicle.mode = VehicleMode("GUIDED")
time.sleep(1)
vehicle.mode = VehicleMode("RTL")	
