from dronekit import connect, VehicleMode
import time



time.sleep(1)
print("Connecting...")
#~ chemin_drone = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0'
chemin_drone = '/dev/ttyACM0'
#~ drone = connect(chemin_drone, wait_ready=True, baud=57600, heartbeat_timeout=2)

#chemin_drone = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0'
vehicle = connect(chemin_drone, wait_ready=False, baud=57600, heartbeat_timeout=2)
#drone = connect('127.0.0.1:14550', wait_ready=True)

# Coefficients de l'asservissement PID de l'atterrissage
kp_atterrissage = 0 # Coefficient mis à 0 car initialisé plus tard
kd_atterrissage = 0.0002
ki_atterrissage = 0.000001
coefficient_kp_atterrissage = 0.5

# Initialisation des coefficients pour le calcul des erreurs dérivées et intégrales
erreurIntegraleEst_atterrissage = 0
erreurIntegraleNord_atterrissage = 0
erreurAnterieureEst_atterrissage = 0
erreurAnterieureNord_atterrissage = 0

# Initialisation des coefficients pour le calcul des erreurs dérivées et intégrales
erreurIntegraleEst_suivi_vehicule = 0
erreurIntegraleNord_suivi_vehicule = 0
erreurAnterieureEst_suivi_vehicule = 0
erreurAnterieureNord_suivi_vehicule = 0

previousLatitude = 0
previousLongitude = 0
previousCarPosition = None
previousMeasuredTime = None

# Connect to the Vehicle (in this case a UDP endpoint)
#vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print ("Basic pre-arm checks")
    # Don't try to arm until autwinopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

# Définition de la consigne de vitesse selon le repère x,y,z du drone
def set_velocity(velocity_x, velocity_y, velocity_z):
    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0x0DC7,  # type_mask (ignore pos | ignore acc)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,
        # x, y, z velocity in m/s -- X positive forward or North/ Y positive right or East / Z positive down
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)  # yaw, yaw_rate (not used)
    # Envoie de la consigne de vitesse au drone
    vehicle.send_mavlink(msg)

def asservissement_atterrissage(aruco_center_x, aruco_center_y):

    # Si l'aruco n'est pas détecté, on l'affiche et on quitte la fonction
    if aruco_center_x == None:
        set_velocity(0, 0, 0.2)  # sens z positif -> vers le sol
        return None, None, None, None

    # Récupération de l'altitude du drone
    altitude = vehicle.rangefinder.distance
    # Calcul de la valeur du coefficient du correcteur P en fonction de l'altitude du drone
    kp_atterrissage = 0.005 if altitude < 5 else 0.008
    kp_atterrissage *= coefficient_kp_atterrissage

    # Distance en pixel entre le centre de l'aruco trouvé et le centre de la caméra selon les axes x et y de la camera
    erreurX = camera.x_imageCenter - aruco_center_x
    erreurY = camera.y_imageCenter - aruco_center_y
    # Passage en coordonnées cylindriques avec comme origine le centre de la caméra
    dist_center = sqrt(erreurX ** 2 + erreurY ** 2)
    dist_angle = atan2(erreurY, erreurX)
    # Rotation de la base pour correspondre au repère du drone
    alpha = dist_angle + vehicle.attitude.yaw
    erreurEst = dist_center * cos(alpha)
    erreurNord = dist_center * sin(alpha)

    # Calcul des erreurs intégrale et dérivée
    # Erreur dérivée
    erreurDeriveeEst = (erreurEst - erreurAnterieureEst_atterrissage)
    erreurDeriveeNord = (erreurNord - erreurAnterieureNord_atterrissage)
    # Erreur intégrale
    erreurIntegraleEst_atterrissage += erreurEst
    erreurIntegraleNord_atterrissage += erreurNord
    # Stockage des erreurs en X et Y pour le future calcul de l'erreur dérivée
    erreurAnterieureEst_atterrissage = erreurEst
    erreurAnterieureNord_atterrissage = erreurNord

    # Calcul de la vitesse corrigée
    vEst = kp_atterrissage * erreurEst + kd_atterrissage * erreurDeriveeEst + ki_atterrissage * erreurIntegraleEst_atterrissage
    vNord = kp_atterrissage * erreurNord + kd_atterrissage * erreurDeriveeNord + ki_atterrissage * erreurIntegraleNord_atterrissage
    # Bornage des vitesses à +/- 5 m/s
    vEst = -min(max(vEst, -5.0), 5.0)  # Inversion de signe pour que ça marche
    vNord = min(max(vNord, -5.0), 5.0)

    # Calcul de la distance planaire à partir de laquelle on considère que le drone est au-dessus du drone
    dist_center_threshold = 50 if altitude < 2 else 1000
    # Si n'est dans un rayon d'un mètre autour du drone, il ne change pas son altitude
    if dist_center > dist_center_threshold:
        vz = 0
    # Sinon on le fait se rapprocher du sol avec une vitesse variant en fonction de l'altitude du drone
    else:
        # Choix de la vitesse verticale en fonction de l'altitude
        if altitude > 8:
            vz = 1.5
        elif altitude > 5:
            vz = 1
        elif altitude > 1:
            vz = 0.5
        else:
            vz = 0

    # Envoie de la consigne de vitesse au drone
    print("Consigne en vitesse : VEst = " + str(vEst) + " ; VNord = " + str(vNord) + " ; VZ = " + str(vz))
    set_velocity(vNord, vEst, vz)  # Pour le sense de la camera, X controle le 'east' et Y controle le 'North'
    return erreurX, erreurY, vEst, vNord


def atterrissage_aruco():

    # Récupération de l'altitude du drone
    altitude = vehicle.rangefinder.distance
    # Début du chronomètre
    start_time = time.time()

   # Tant que le drone n'est pas à 1 m du sol ou que le temps écoulé est inférieur à 30 secondes,
    # on lance l'asservissement du drone
    while altitude > 1 and (time.time() - start_time) < 30:

        # Récupération de l'altitude du drone
        altitude = vehicle.rangefinder.distance
        print(datetime.now().strftime("%d-%m %H:%M:%S") + " : Altitude = " + str(altitude))

        # Si le robot est à plus de 7.5 mètres du sol on le fait descendre
        if altitude > 7.5:
            print("Descente du drone")
            set_velocity(0, 0, 1)  # sens z positif -> vers le sol
            continue

        # Si le robot est à moins de 7.5 mètres on détecte directement l'aruco et on récupère les coordonnées de son centre
        else:
            centre_aruco_X, centre_aruco_Y, _, image = camera.detection_aruco(True)
            print("Aruco détecté" if centre_aruco_X != None else "Aruco non détecté")
            # Asservissement par rapport au centre de l'aruco
            erreurX, erreurY, vx, vy = asservissement_atterrissage(centre_aruco_X, centre_aruco_Y)
            if chemin_dossier != "":
                # Affichage de l'erreur et de la vitesse
               image = cv2.putText(image, "Erreur : EX = " + str(erreurX) + " ; EY = " + str(erreurY), (0, 25),
                                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
               image = cv2.putText(image, "Vitesse : Vx = " + str(vx) + " ; Vy = " + str(vy), (0, 50),
                                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
               # Traçage d'un cercle au centre de l'image
               cv2.circle(image, (camera.x_imageCenter, camera.y_imageCenter), 4, (0, 255, 0), -1)
                # Sauvegarde de la photo
               enregistrement_photo_date_position(image, chemin_dossier,
                                                       "yes" if centre_aruco_X != None else "no")

    # Une fois que le robot est assez bas, on le fait atterrir
    print("Atterrissage")
    set_mode("LAND")


# Attente du mode "STABIIZE"
while vehicle.mode != VehicleMode("STABILIZE"):
    print("En attente du mode STABILIZE")
    time.sleep(1)

        # Attente du mode "AUTO"
while vehicle.mode != VehicleMode("AUTO"):    
    print("En attente du mode AUTO")
    time.sleep(1)

        # Passage en mode "GUIDED"    
vehicle.mode = VehicleMode("GUIDED")
print ("Véhicule en mode guidé !")

arm_and_takeoff(10)

atterrissage_aruco()

time.sleep(2)


print("Mise en mode atterrissage")
vehicle.mode = VehicleMode("LAND")

print ("Deconnexion du vehicule")
vehicle.close()

print ("Arret du script")
