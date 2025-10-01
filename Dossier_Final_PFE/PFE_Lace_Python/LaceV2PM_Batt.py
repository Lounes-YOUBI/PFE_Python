#!/usr/bin/env python
# -*- coding: utf-8 -*-

################## Import ########################################################################
import Fonctions
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import argparse
import threading
import serial


###############  Donnees en entrees du programme a rentrer ou une valeur par defaut est donnee #####
parser=argparse.ArgumentParser()
arg1 = parser.add_argument('--NbPts',type=int, default=35)#Nombre de points de l'ellipse
arg2= parser.add_argument('--Rayon',type=float, default=5)# Taille du cercle en m
arg3= parser.add_argument('--it',type=int, default=8)#Nombre d'iterations max (inutilise actuellement)
arg4= parser.add_argument('--fact_dist',type=float, default=2)#Facteur de dist pour arriver a un point GPS lors de la traj en ellipse
arg5= parser.add_argument('--VitesseCercle',type=float, default=1.4)#Vitesse max lors de la traj en ellipse
arg6= parser.add_argument('--GAxe',type=float, default=1.2)#facteur allongement grand axe
arg7= parser.add_argument('--PAxe',type=float, default=0.8)#facteur retrecissement petit axe
arg8= parser.add_argument('--Vent',type=float, default=45)# Angle cardinal de provenance du vent
arg9= parser.add_argument('--altitude',type=float, default=10)#Altitude de vol du drone
ar10=parser.add_argument('--angle',type=float, default=85)# Angle entre chaque ellipse realisee
arg11= parser.add_argument('--Seuil_PM_2_5_entree',type=float, default=70)# Seuil d'entree dans le nuage des PM2_5 
arg12= parser.add_argument('--Seuil_PM1_entree',type=int, default=70)# Seuil d'entree dans le nuage des PM1 
arg13= parser.add_argument('--Seuil_PM_2_5_sortie',type=float, default=70) #Seuil de sortie du nuage des PM2_5 
arg14= parser.add_argument('--Seuil_PM1_sortie',type=int, default=70) #Seuil de sortie du nuage des PM1 
arg15=parser.add_argument('--Min_battery',type=float, default=14)#tension minimum de la batterie
args = vars(parser.parse_args())


#### Drone ########
VitesseDrone=1.5
iterations=1
try:
    max_iterations = args["it"] #Nb d'iterations
except:
	max_iterations= arg3.default
    
try:
	Volt_min=args["Min_battery"]
except:
	Volt_min=arg15.default
    

####### Conversion ########

Klat=1.109462521e5
Klon=Klat*2/3

########## Parametres ellipse/cercle ##########

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

##Parametres drone##
try:
	alt=args["altitude"]
except:
	alt=arg9.default
ok_i=1
ok_j=1

#Point GPS du coeur du nuage de fumee
centre=LocationGlobalRelative(48.8581097, 7.2049803,alt)

############################### Debut du programme ##############################################

fichier=open("Mesure.txt","a") # Creation d'un fichier texte pour enregistrer les valeurs mesurees 
fichier.write("-----------------")
fichier.write("\n")
global vehicle
vehicle = Fonctions.connection()# Connection au drone
debut=time.time()
chaine=0
fl=0
serialArduino=serial.Serial('/dev/ttyACM2',9600)# liaison serie entre l'Arduino et la RPi
#serialArduino.reset_input_buffer()

# Fonction de Lecture des donnees sur le bus serie qui va etre threader en parallele du programme principal ########
def lecture(serialArduino,fl):
    # variables globales pour pouvoir y acceder n'importe ou dans le programme
    global fichier # nom du fichier de sauvegarde
    global PM1,PM2_5,PM10 # Valeurs des concentrations en PM
    global nb_mesure # nombre de mesures effectuees
    global event # variable event qui stoppe la lecture des donnees sur le bus serie
    event=0
    PM1=0
    PM2_5=0
    PM10=0
    chrono2=0
    chrono1=0
    nb_mesure=0
    while event==0:
        print("event", event)
        chrono1=time.time()
        chaine,PM1,PM2_5,PM10=Fonctions.ReadOPCData(serialArduino,fl) # lecture des donnees sur le bus serie
        print("ok2")
        nb_mesure=nb_mesure+1
        chrono2=chrono2+(time.time()-chrono1)

        ## Affichage console ###
        print(" ")
        print(" ")
        print("chaine",chaine)
        print("PM1 (en ug/m^3) : ",PM1)
        print("PM2_5 (en ug/m^3) : ",PM2_5)     
        print("PM10 (en ug/m^3) : ",PM10)
        print("chaine : ",chaine)
        print("GPS coord : ",vehicle.location.global_relative_frame.lat,
        	vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.alt)
        print(" ")

        ## Ecriture dans le fichier ##
        fichier.write(str(PM1))
        fichier.write(" ")
        fichier.write(str(PM2_5))
        fichier.write(" ")
        fichier.write(str(chrono2))
        fichier.write(" ")
        fichier.write(str(vehicle.location.global_relative_frame.lat))
        fichier.write(" ")
        fichier.write(str(vehicle.location.global_relative_frame.lon))
        fichier.write(" ")
        fichier.write(str(vehicle.location.global_relative_frame.alt))
        fichier.write(";")
        fichier.write("\n")

############### Seuils PM ###############################					

try:
	Seuil_PM_2_5_entree=args["Seuil_PM_2_5_entree"]
except:
	Seuil_PM_2_5_entree=arg11.default

try:
	Seuil_PM1_entree=args["Seuil_PM1_entree"]
except:
	Seuil_PM1_entree=arg12.default

try:
	Seuil_PM_2_5_sortie=args["Seuil_PM_2_5_sortie"]
except:
	Seuil_PM_2_5_sortie=arg13.default

try:
	Seuil_PM1_sortie=args["Seuil_PM1_sortie"]
except:
	Seuil_PM1_sortie=arg14.default


vehicle.parameters['RTL_ALT'] = alt #Altitude du RTL a l'altiitude de vol
time.sleep(1)
t1 = threading.Thread(target=lecture, args=(serialArduino,fl,))
# Configuration du thred de lecture du bus serie
t1.start()# Demarrage du thread
time.sleep(15)


print("")
print("-------------")
print("Debut")
print("-------------")
print("")

########## Procedure de decollage  du drone ################
vehicle.mode=VehicleMode("STABILIZE")
#Le drone decolle
while True:
	print ("en attente de auto")
	print ("mode: %s" % vehicle.mode)
	if (vehicle.mode == VehicleMode("AUTO")): # Attente du passage du mode "AUTO" par le telepilote
		Fonctions.arm_and_takeoff(alt)#Decollage
		break
	time.sleep(0.25)
print ("Lancement de la mission")
print ("Decollage")
vehicle.parameters['WP_YAW_BEHAVIOR'] = 0 # Angle de lacet (yaw) constant lors vol
		
#Le drone se deplace en direction de point GPS dans le nuage
vehicle.simple_goto(centre, groundspeed=VitesseDrone)

				
while True:
	if float(PM1)>Seuil_PM1_entree or float(PM2_5)>Seuil_PM_2_5_entree: 
	#Si le seuil des PM est depasse le drone passe en mode "LOITER"
		break
vehicle.channels.overrides['2'] = 1500 #Poussee vertical du drone
vehicle.mode = VehicleMode("LOITER")#Passage du mode "LOITER" le drone garde son altitude et sa position 
print("-----------")
print("LOITER")
print("-----------")
time.sleep(2)

#Le drone commence son schema de vol adaptatif avec ses trajectoires en ellipse
while True:   
	print(iterations)
	if vehicle.battery.voltage<Volt_min: 
	# Si le niveau de batterie est trop faible on lance la procedure de fin 
		break
	vehicle.mode = VehicleMode("GUIDED")
	#Passage en mode "GUIDED" pour parcourir les points GPS de l'ellipse
	print("-----------")
	print("GUIDED")
	print("-----------")
	time.sleep(1)
	tour=False
	a_inc=0
	b_inc=0
	ang_inc=0
	while True:   #Parcours des points GPS de l'ellipse 
		bearing=Fonctions.get_bearing(vehicle.location.global_relative_frame, centre)
		angle=(bearing+ang_ellipse+ang_inc)%360 #Calcul de l'angle d'orientation de l'ellipse
		vehicle.parameters['WP_YAW_BEHAVIOR'] = 0 #Angle de lacet (yaw) constant lors du vol
		print("-----------")
		print("ELLIPSE")
		print("-----------")
		#fonction de parcours des points GPS
		tour=Fonctions.Traj(VitesseCercle,Seuil_PM1_entree,Seuil_PM_2_5_entree,Seuil_PM1_sortie,
		Seuil_PM_2_5_sortie,centre,angle,RayonCercle,NbPoints,a+a_inc,b+b_inc,fact_dist,
		fact_temps,theta,alt)
		if tour==True:#Si le seuil des PM est depasse on stoppe le parcours des points de l'ellipse
			break
		if vehicle.battery.voltage<Volt_min:
		#Si le niveau de batterie est trop faible on stoppe le parcours des points de l'ellipse
			break
		### Ajustements parametres ellipse lorsque le drone est perdu ##############
		a_inc=a_inc+(a+a_inc)*0.3 #Ajustement du facteur du petit axe
		b_inc=b_inc+(b+b_inc)*0.3 #Ajustement du facteur du grand axe
		ang_inc=ang_inc+40#Ajustement de l'angle d'orientation de l'ellipse
	vehicle.mode = VehicleMode("LOITER")#Mode "LOITER" lorsque le seuil des PM est depasse
	print("-----------")
	print("LOITER ELLIPSE")
	print("-----------")
	time.sleep(0.3)
	iterations=iterations+1

####### Procedure de fin du programme et de fin de suivi de la frontiere ############
event=1 # Fin de la lecture sur le bus serie vehicle.mode =
VehicleMode("LAND")# Mode "LAND" le drone atterit print
print("-----------") 
print("LAND") 
print("-----------") 
time.sleep(1)		


######################### Fonctions ###########################################################

# Fonction de connection au drone
# Pixhawk connecte sur le port USB de la RPi
def connetion():
    print("Connection au Drone ...")
    drone = connect("/dev/ttyACM0", wait_ready=False, baud=57600, heartbeat_timeout=2)
    time.sleep(3)
    if drone is not None:
        print("Drone connecte : " + str(drone.version))
        return drone
    else:
        print("Drone pas connecte")
    sys.exit()


# Fonction de decollage du drone du GitHub de dronekit
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


# Fonction de calcul de distance entre 2 points GPS
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


# Fonction qui calcule les coordonnees des points GPS du cercle/ellipse a realiser
# rayon: Rayon du cercle
# direction : Direction cardinale dans laquelle l'ellipse est dirigee par rapport au drone
# pointsCercle : Nombre de points generes sur le cercle/ellipse
# a,b: Facteur d'allongement du petit axe et grand axe de l'ellipse
# theta : Angle de rotation de l'ellipse

def Cercle(rayon, direction, pointsCercle, a, b, theta):
    Klat = 1.109462521e5  # Facteur de conversion de metres en coordonnees en latitude
    Klon = Klat * 2 / 3  # Facteur de conversion de metres en coordonnees en longitude
    rayon = rayon
    theta = theta * (math.pi / 180)
    direction = (math.pi / 180) * direction
    Lat_act = vehicle.location.global_relative_frame.lat + rayon * a * math.cos(direction) / Klat
    # Calcul de la position du centre du cercle en latitude
    Lon_act = vehicle.location.global_relative_frame.lon + rayon * b * math.sin(direction) / Klon
    # Calcul de la position du centre du cercle en longitude
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


# Fonction qui retourne la distance
# et la position dans le tableau "coords" du point GPS le plus proche d'une localisation
# coords : Liste avec les coordonnees GPS (latitude,longitude)
# aLoc : position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
def NearestCoord(coords, aLoc):
    alt = 10
    dist = 100000
    for i in range(0, len(coords)):
        LocPoint = LocationGlobalRelative(coords[i][0], coords[i][1], alt)
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
    aLoc = LocationGlobalRelative(coords[0][0], coords[0][1], alt)
    for i in range(0, len(coords)):
        LocPoint = LocationGlobalRelative(coords[i][0], coords[i][1], alt)
        if get_distance_metres(LocPoint, aLoc) > dist:
            dist = get_distance_metres(LocPoint, aLoc)
    return dist


# Fonction pour determiner le cap entre deux pointqs GPS (bearing)
# aLocation1 : position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
# aLocation2 : position GPS "LocationGlobalRelative" (latitude,longitude,altitude)
def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing  # retourne le cap entre deux points GPS


# Fonction de lecture des mesures l'OPC envoyees par l'Arduino
# serialArduino : liaison serie
# Si fl=1 : les mesures sont renvoyees en float sinon en string
def ReadOPCData(serialArduino, fl):
    chaine = serialArduino.readline().decode('ascii').rstrip()
    # on lit la ligne envoye sur le bus serie
    Pos1 = -1
    Pos2 = -1
    Pos3 = -1
    print("chaine :", chaine)
    for i in range(len(chaine)):
        # Les donnees sont envoyes sous la forme "PM1sPM2_5mPM10g"
        # on detecte les separateurs des 3 valeurs
        if chaine[i] == "s":
            Pos1 = i
        # print("Pos1:",Pos1)
        elif chaine[i] == "m":
            Pos2 = i
        # print("Pos2:",Pos2)
        elif chaine[i] == "g":
            Pos3 = i
        # print("Pos3:",Pos3)
    if Pos1 != -1 and Pos2 != -1 and Pos3 != -1:
        # Si on arrive a lire les valeurs on retourne les valeurs de concentrations des PM
        if fl == 1:  # Conversion en float
            sensorValPM1 = float(chaine[0:Pos1])
            sensorValPM2_5 = float(chaine[Pos1 + 1:Pos2])
            sensorValPM10 = float(chaine[Pos2 + 1:Pos3])
        else:
            sensorValPM1 = chaine[0:Pos1]
            sensorValPM2_5 = chaine[Pos1 + 1:Pos2]
            sensorValPM10 = chaine[Pos2 + 1:Pos3]
        return chaine, sensorValPM1, sensorValPM2_5, sensorValPM10
    else:
        return chaine, -1, -1, -1
        # Si on arrive pas a lire de valeurs on retourne -1 avec la chaine de caracteres detectee

        # Fonction qui realise la trajectoire en ellipse du drone
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
        # fact_dist : facteur de distance,
        #   lorsque le drone arrive a une certaine distance du point GPS voulu
        #   alors il passe au point suivant de l'ellipse
        # fact_temps : facteur de temps,
        #	au bout d'un certain temps si le drone ne parvient pas dans la zone du point GPS voulu
        #	alors il se dirige vers le suivant
        # theta : Angle de rotation de l'ellipse
        # alt : altitude de vol du drone


def Traj(VitesseCercle, Seuil_PM1_entree, Seuil_PM_2_5_entree, Seuil_PM1_sortie, Seuil_PM_2_5_sortie, centre, angle, RayonCercle, NbPoints, a, b, fact_dist, fact_temps, theta, alt):
	coords = Cercle(RayonCercle, angle, NbPoints, a, b, theta)  # Calcul des points de l'ellipse a parcourir
    dist, rang = NearestCoord(coords, vehicle.location.global_relative_frame)
    #Detection du point de l'ellipse le plus proche du drone
    coordsF = Tri(coords, rang, -1)  # Tri des points GPS a parcourir dans l'ordre
    points = []
    for lat, lon in coordsF:
        points.append(LocationGlobalRelative(lat, lon, alt))
    # Ajout des points GPS a parcourir dans une liste de "LocationGlobalRelative"
    ok_j = 1
    pt = 0
    ok = 0
    while True:  # Boucle while
        if vehicle.battery.voltage < Volt_min:
            # Si le niveau de batterie est trop faible on sors de la fonction "Traj()"
            # et on arrete de parcourir les points GPS de l'ellipse
            return True
        # return True, la procedure de fin de programme va etre enclenchee
        if pt == len(points) - 1:
            # Si le point a parcourir est le dernier du tableau de point
            # alors le drone a parcouru tout les points de l'ellipse sans sortir du nuage
            # (perte de la frontiere)
            pt = 0
            return False
        # return False on sort de la fonction "Traj()",
        # les parametres vont etre ajustes pour retrouver la frontiere du nuage
        if ok == 0:
            # Si le point precedent a ete atteint
            # alors le drone se deplace en ligne droite vers le point GPS suivant
            vehicle.simple_goto(points[pt], groundspeed=VitesseCercle)
            start = time.time()
            ok = 1
        nxt = pt - 1
        if nxt == -1:
            nxt = 1
        D = get_distance_metres(points[nxt], points[pt])
        # print("D",D)
        if get_distance_metres(vehicle.location.global_relative_frame, points[pt]) < fact_dist * D or time.time() - start > D / fact_temps:
        # Si le drone arrive assez proche du point GPS demande
        # ou que le temps depuis le dernier point GPS parcouru depasse un certain seuil
        # alors le drone passe au point suivant
        pt = pt + 1
        ok = 0
    i = 0

    if (float(PM1) > Seuil_PM1_entree or float(PM2_5) > Seuil_PM_2_5_entree) and ok_j == 2:
        # Second passage de la frontiere le drone reentre dans le nuage
        ok_j = 1
        return True  # return True le processus de parcours des points de l'ellipse se stoppe
    elif (float(PM1) < Seuil_PM1_sortie or float(PM2_5) > Seuil_PM_2_5_sortie) and ok_j == 1:
        # Premier passage de la frontiere du nuage le drone ressort du nuage
        # le drone continue de parcourir l'ellipse
        ok_j = 2