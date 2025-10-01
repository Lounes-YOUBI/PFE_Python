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
from pymavlink_utils import DataPoint
from folium import plugins
import matplotlib.cm as cm
import matplotlib.colors as colors
from branca.colormap import linear

##################### PARAMETRES DE CONFIGURATION ###########################################

# Arguments d'entrée du programme avec valeurs par défaut
parser = argparse.ArgumentParser()
arg_nb_points = parser.add_argument('--NbPts', type=int, default=20)
arg_rayon = parser.add_argument('--Rayon', type=float, default=15)
arg_iterations = parser.add_argument('--it', type=int, default=4)
arg_facteur_distance = parser.add_argument('--fact_dist', type=float, default=2)
arg_vitesse_cercle = parser.add_argument('--VitesseCercle', type=float, default=3)
arg_grand_axe = parser.add_argument('--GAxe', type=float, default=1.7)
arg_petit_axe = parser.add_argument('--PAxe', type=float, default=1)
arg_vent = parser.add_argument('--Vent', type=float, default=90)
arg_altitude_max = parser.add_argument('--altitude_max', type=float, default=10)
arg_altitude_min = parser.add_argument('--altitude_min', type=float, default=8)
arg_pas_altitude = parser.add_argument('--pas_altitude', type=float, default=1)
arg_angle = parser.add_argument('--angle', type=float, default=-180)
arg_batterie_min = parser.add_argument('--Min_battery', type=float, default=-10)
args = vars(parser.parse_args())

print("Nombre de points: %s" % arg_nb_points)
print("Rayon: %s" % arg_rayon)
print("Itérations: %s" % arg_iterations)
print("Facteur distance: %s" % arg_facteur_distance)
print("Vitesse cercle: %s" % arg_vitesse_cercle)
print("Grand axe: %s" % arg_grand_axe)
print("Petit axe: %s" % arg_petit_axe)
print("Vent: %s" % arg_vent)
print("Altitude max: %s" % arg_altitude_max)
print("Altitude min: %s" % arg_altitude_min)
print("Pas d'altitude: %s" % arg_pas_altitude)
print("Angle: %s" % arg_angle)
print("Batterie min: %s" % arg_batterie_min)

# Paramètres du drone
vitesse_drone = 2

# Récupération des paramètres avec gestion des exceptions
try:
    nb_iterations_max = args["it"]
except:
    nb_iterations_max = arg_iterations.default

try:
    niveau_batterie_min = args["Min_battery"]
except:
    niveau_batterie_min = arg_batterie_min.default

# Paramètres de l'ellipse/cercle
try:
    vitesse_cercle = args["VitesseCercle"]
except:
    vitesse_cercle = arg_vitesse_cercle.default

try:
    rayon_cercle = args["Rayon"]
except:
    rayon_cercle = arg_rayon.default

try:
    nombre_points = args["NbPts"]
except:
    nombre_points = arg_nb_points.default

try:
    facteur_distance = args["fact_dist"]
except:
    facteur_distance = arg_facteur_distance.default

facteur_temps = 0.8

try:
    angle_ellipse = args["angle"]
except:
    angle_ellipse = arg_angle.default

try:
    facteur_grand_axe = args["GAxe"]
except:
    facteur_grand_axe = arg_grand_axe.default

try:
    facteur_petit_axe = args["PAxe"]
except:
    facteur_petit_axe = arg_petit_axe.default

try:
    angle_vent = args["Vent"]
except:
    angle_vent = arg_vent.default
    

# Paramètres du drone

try:
    altitude_max = args["altitude_max"]
except:
    altitude_max = arg_altitude_max.default

try:
    altitude_min = args["altitude_min"]
except:
    altitude_min = arg_altitude_min.default

try:
    pas_altitude = args["pas_altitude"]
except:
    pas_altitude = arg_pas_altitude.default

altitude_vol = altitude_max


#Liste pour stocker les points de données
datapoints = [] 

# Paramètres du nuage de particules
seuil_critique = 150000000  # Seuil critique qui déclenche la procédure de fin de cartographie
seuil_entree_debut = 350000
seuil_entree = 400000
seuil_sortie = 400000

angle_inc = 0
position_depart_cartographie = None  # Position de départ de la cartographie
temps_depart_cartographie = None  # Temps de départ de la cartographie
#temps_min_tour_complet = 120  # Temps minimum en secondes avant de considérer qu'un tour complet est possible
temps_min_tour_complet = 120  # Temps minimum en secondes avant de considérer qu'un tour complet est possible
tour_nuage_complet = False  # Indique si le tour complet du nuage a été effectué
cartographie_complete = False  # Indique si la cartographie est complète
#################################################################################################################

# Centre GPS du nuage
centre_nuage = Position(lat_deg=48.629687, lon_deg=7.787335, relative_alt_m=altitude_vol)

# Création de la carte
carte_folium = folium.Map(
    location=[centre_nuage.lat_deg, centre_nuage.lon_deg],
    zoom_start=14,
    tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attr='Esri'
)

# Constantes de conversion GPS
constante_latitude = 111320
constante_longitude = 111320 * math.cos(math.radians(centre_nuage.lat_deg))

# Connexion à la simulation
print('Connexion au véhicule en cours...')
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()
print("Drone connecté")


# Paramètres du modèle de concentration
fichier_concentration = "concentration.npy"
fichier_altitude = "altitudes.npy"
altitude_coupe = altitude_max  # Altitude à extraire (en mètres)


################################# FONCTIONS #######################################

def calculer_distance_metres(position1, position2):
    """
    Calcule la distance sphérique (Haversine) entre deux points GPS en mètres.

    Args:
        position1: Premier point GPS (Position)
        position2: Deuxième point GPS (Position)

    Returns:
        float: Distance en mètres
    """
    # Rayon moyen de la Terre en mètres
    R = 6371000
    lat1_rad = math.radians(position1.lat_deg)
    lat2_rad = math.radians(position2.lat_deg)
    delta_lat = math.radians(position2.lat_deg - position1.lat_deg)
    delta_lon = math.radians(position2.lon_deg - position1.lon_deg)

    a = math.sin(delta_lat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

def generer_coordonnees_ellipse_complete(rayon, direction_cardinale, nb_points,
                                         facteur_petit_axe, facteur_grand_axe):
    """
    Génère les coordonnées GPS d'une ellipse complète, alignée selon une direction donnée,
    avec le GRAND axe comme base.

    Args:
        rayon: Rayon de base (mètres)
        direction_cardinale: Orientation de l’ellipse (de 0 à 360, 0 = Nord, 90 = Est, etc.)
        nb_points: Nombre de points à générer
        facteur_petit_axe: Allongement du petit axe (vertical)
        facteur_grand_axe: Allongement du grand axe (horizontal)

    Returns:
        list: Liste des coordonnées GPS [(latitude, longitude), ...]
    """

    # Obtenir le point de départ GPS (utilisé pour calculer le centre)
    point_depart = utils.get_position(master)

    # Redéfinition des axes
    a = rayon * facteur_grand_axe  # demi-grand axe
    b = rayon * facteur_petit_axe  # demi-petit axe

    # Conversion en angle mathématique (0 = Est, 90 = Nord, etc.)
    alpha = math.radians(direction_cardinale)

    # Calcul du centre de l'ellipse à partir du point de départ (gauche)
    delta_x_centre = a * math.cos(alpha)
    delta_y_centre = a * math.sin(alpha)

    delta_lat_centre = delta_y_centre / constante_latitude
    delta_lon_centre = delta_x_centre / constante_longitude

    lat_centre = point_depart.lat_deg + delta_lat_centre
    lon_centre = point_depart.lon_deg + delta_lon_centre

    # Angles pour une ellipse complète (0° à 360°)
    angles = [math.radians(i) for i in range(0, 360, max(1, int(360 / nb_points)))]

    points = []
    for theta in angles:
        # Coordonnées dans le repère local (ellipse horizontale)
        x = a * math.cos(theta)
        y = b * math.sin(theta)

        # Rotation dasn le sen horaire de l'ellipse selon la direction donnée
        x_rot = x * math.cos(alpha) + y * math.sin(alpha)
        y_rot = -x * math.sin(alpha) + y * math.cos(alpha)

        # Conversion en coordonnées GPS
        delta_lat = y_rot / constante_latitude
        delta_lon = x_rot / constante_longitude

        lat = lat_centre + delta_lat
        lon = lon_centre + delta_lon

        points.append((lat, lon))

    return points

def trouver_point_plus_proche(coordonnees_ellipse, position_actuelle):
    """
    Trouve le point GPS le plus proche d'une position donnée.

    Args:
        coordonnees_ellipse: Liste des coordonnées GPS
        position_actuelle: Position GPS actuelle

    Returns:
        tuple: (distance_minimale, index_point_proche)
    """
    distance_min = 100000
    index_proche = 0

    for i in range(0, len(coordonnees_ellipse)):
        point_ellipse = Position(lat_deg=coordonnees_ellipse[i][0], lon_deg=coordonnees_ellipse[i][1],
                                 relative_alt_m=altitude_vol)
        distance = calculer_distance_metres(point_ellipse, position_actuelle)

        if distance < distance_min:
            distance_min = distance
            index_proche = i

    return distance_min, index_proche


def organiser_points_ellipse(coordonnees_ellipse, index_depart, sens_parcours):
    """
    Organise les points GPS dans l'ordre de parcours de l'ellipse.

    Args:
        coordonnees_ellipse: Liste des coordonnées GPS
        index_depart: Index du point de départ
        sens_parcours: Sens de parcours (-1 pour anti-horaire, 1 pour horaire)

    Returns:
        list: Liste ordonnée des coordonnées GPS
    """
    coordonnees_finales = []

    # Ajouter les points à partir de l'index de départ
    for j in range(index_depart, len(coordonnees_ellipse)):
        coordonnees_finales.append(coordonnees_ellipse[j])

    # Ajouter les points avant l'index de départ
    for k in range(0, index_depart):
        coordonnees_finales.append(coordonnees_ellipse[k])

    # Inverser si sens anti-horaire
    if sens_parcours == -1:
        coordonnees_finales.reverse()

    # Boucler l'ellipse en ajoutant le premier point à la fin
    coordonnees_finales.append(coordonnees_finales[0])

    return coordonnees_finales

def calculer_cap_gps(position1, position2):
    """
    Calcule le cap (bearing) entre deux points GPS.

    Args:
        position1: Position GPS de départ
        position2: Position GPS d'arrivée

    Returns:
        float: Cap en degrés (0-360)
    """
    # Conversion en radians
    lat1_rad = math.radians(position1.lat_deg)
    lat2_rad = math.radians(position2.lat_deg)
    lon1_rad = math.radians(position1.lon_deg)
    lon2_rad = math.radians(position2.lon_deg)

    # Différence de longitude
    delta_lon = lon2_rad - lon1_rad

    # Calcul du bearing
    y = math.sin(delta_lon) * math.cos(lat2_rad)
    x = (math.cos(lat1_rad) * math.sin(lat2_rad) -
         math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))

    # Conversion en degrés et normalisation
    bearing_rad = math.atan2(y, x)
    bearing_deg = math.degrees(bearing_rad)
    bearing_deg = (bearing_deg + 360 + 90) % 360  #Pour avoir Nord à 90°

    return bearing_deg

def generer_nuage_particules(fichier_concentration_3d, fichier_altitudes, altitude_extraction, centre_gps):
    """
    Génère le nuage de particules à partir des données 3D.

    Args:
        fichier_concentration_3d: Chemin vers le fichier de concentration 3D
        fichier_altitudes: Chemin vers le fichier des altitudes
        altitude_extraction: Altitude d'extraction de la coupe
        centre_gps: Centre GPS de référence

    Returns:
        tuple: (latitudes_gps, longitudes_gps, valeurs_capteur)
    """
    facteur_conversion = 1e9  # Conversion kg/m³ vers µg/m³

    # Chargement des données
    donnees_3d = np.load(fichier_concentration_3d)
    altitudes = np.load(fichier_altitudes)

    # Sélection de l'altitude la plus proche
    altitudes_array = np.array(altitudes)
    index_altitude = np.argmin(np.abs(altitudes_array - altitude_extraction))

    # Extraction de la coupe 2D
    coupe_2d = donnees_3d[:, :, index_altitude]
    valeurs_capteur = coupe_2d * facteur_conversion

    # Dimensions de la zone
    nb_lignes, nb_colonnes = valeurs_capteur.shape
    longueur_zone = nb_lignes  # mètres
    largeur_zone = nb_colonnes  # mètres

    # Centre de concentration maximale
    ligne_centre, colonne_centre = np.unravel_index(np.argmax(valeurs_capteur), valeurs_capteur.shape)

    # Création des grilles GPS
    latitudes_gps = np.zeros_like(valeurs_capteur)
    longitudes_gps = np.zeros_like(valeurs_capteur)

    # Calcul des pas de discrétisation
    pas_nord_sud = longueur_zone / nb_lignes
    pas_est_ouest = largeur_zone / nb_colonnes

    for i in range(nb_lignes):
        for j in range(nb_colonnes):
            # Calcul des coordonnées relatives au centre de concentration
            delta_nord = (i - ligne_centre) * pas_nord_sud
            delta_est = (j - colonne_centre) * pas_est_ouest

            # Conversion vers coordonnées GPS
            latitude, longitude, _ = utils.enu_to_gps(delta_est, delta_nord, altitude_extraction,
                                                      centre_gps.lat_deg, centre_gps.lon_deg, altitude_extraction)

            latitudes_gps[i, j] = latitude
            longitudes_gps[i, j] = longitude

    return latitudes_gps, longitudes_gps, valeurs_capteur


def afficher_nuage_sur_carte(latitudes_gps, longitudes_gps, valeurs_capteur, chemin_sauvegarde='carte_nuage.html',
                             seuil_affichage=None):
    """
    Affiche le nuage de particules sur une carte Folium.

    Args:
        latitudes_gps: Coordonnées de latitude (array 2D)
        longitudes_gps: Coordonnées de longitude (array 2D)
        valeurs_capteur: Valeurs de concentration (array 2D)
        chemin_sauvegarde: Chemin de sauvegarde de la carte
        seuil_affichage: Seuil minimum pour afficher un point
    """
    global carte_folium

    # Statistiques du nuage
    print(f"=== STATISTIQUES DU NUAGE ===")
    print(f"Valeurs min/max: {valeurs_capteur.min():.2e} / {valeurs_capteur.max():.2e}")
    print(f"Nombre de valeurs > 0: {np.sum(valeurs_capteur > 0)}")
    print(f"Nombre total de points: {valeurs_capteur.size}")

    # Définition du seuil d'affichage
    if seuil_affichage is None:
        seuil_affichage = valeurs_capteur.max() * 0.02
        print(f"Seuil automatique: {seuil_affichage:.2e}")

    # Filtrage des valeurs significatives
    masque_significatif = valeurs_capteur > seuil_affichage
    nb_points_affiches = np.sum(masque_significatif)
    print(f"Nombre de points à afficher: {nb_points_affiches}")

    if nb_points_affiches == 0:
        print("ATTENTION: Aucun point à afficher avec ce seuil!")
        return

    # Création de la palette de couleurs
    valeur_min = seuil_affichage
    valeur_max = valeurs_capteur.max()
    palette_couleurs = linear.plasma.scale(valeur_min, valeur_max)
    palette_couleurs.caption = 'Concentration en particules fines (µg/m³)'

    # Ajout des points significatifs sur la carte
    points_ajoutes = 0
    for i in range(latitudes_gps.shape[0]):
        for j in range(latitudes_gps.shape[1]):
            valeur = valeurs_capteur[i, j]
            if valeur > seuil_affichage:
                folium.CircleMarker(
                    location=[latitudes_gps[i, j], longitudes_gps[i, j]],
                    radius=4,
                    fill=True,
                    fill_color=palette_couleurs(valeur),
                    color=palette_couleurs(valeur),
                    fill_opacity=0.7,
                    weight=1,
                    popup=f"Concentration: {valeur:.2e} µg/m³"
                ).add_to(carte_folium)
                points_ajoutes += 1

    print(f"Points effectivement ajoutés: {points_ajoutes}")

    # Ajout de la légende et du marqueur central
    palette_couleurs.add_to(carte_folium)
    folium.Marker(
        location=[centre_nuage.lat_deg, centre_nuage.lon_deg],
        popup="Centre du nuage",
        icon=folium.Icon(color='red', icon='info-sign')
    ).add_to(carte_folium)

    # Sauvegarde de la carte
    carte_folium.save(chemin_sauvegarde)
    print(f"Carte sauvegardée : {chemin_sauvegarde}")


def detecter_entree_nuage(latitudes_gps, longitudes_gps, valeurs_capteur, seuil_detection):
    """
    Détecte si le drone entre dans le nuage de particules.

    Args:
        latitudes_gps: Coordonnées de latitude du nuage
        longitudes_gps: Coordonnées de longitude du nuage
        valeurs_capteur: Valeurs de concentration
        seuil_detection: Seuil de détection d'entrée

    Returns:
        bool: True si entrée détectée, False sinon
    """
    position_actuelle = utils.get_position(master)

    # Calcul des distances entre la position actuelle et chaque point du nuage
    deltas_lat = (latitudes_gps - position_actuelle.lat_deg) * constante_latitude
    deltas_lon = (longitudes_gps - position_actuelle.lon_deg) * constante_longitude
    distances = np.sqrt(deltas_lat ** 2 + deltas_lon ** 2)

    # Recherche du point le plus proche
    index_ligne, index_colonne = np.unravel_index(np.argmin(distances), distances.shape)
    distance_min = distances[index_ligne, index_colonne]
    valeur_proche = valeurs_capteur[index_ligne, index_colonne]

    # Vérification des conditions d'entrée
    if valeur_proche > seuil_detection and distance_min < 5:
        print("Valeur PM 2.5 : ", valeur_proche)
        return True

    return False


def executer_trajectoire_ellipse(seuil_entree_nuage, seuil_sortie_nuage, valeurs_capteur, centre_nuage,
                                 angle_orientation, rayon_ellipse, nb_points_ellipse, facteur_petit_axe,
                                 facteur_grand_axe, facteur_dist, facteur_temps, angle_rotation,
                                 altitude_vol, seuil_critique_capteur):
    global angle_inc
    global dernier_bord_nuage
    global tour_nuage_complet
    global position_depart_cartographie
    global temps_depart_cartographie
    """
    Exécute la trajectoire en ellipse autour du nuage.

    Args:
        seuil_entree_nuage: Seuil d'entrée dans le nuage
        seuil_sortie_nuage: Seuil de sortie du nuage
        valeurs_capteur: Valeurs de concentration du nuage
        centre_nuage: Centre GPS du nuage
        angle_orientation: Angle d'orientation de l'ellipse
        rayon_ellipse: Rayon de l'ellipse
        nb_points_ellipse: Nombre de points de l'ellipse
        facteur_petit_axe: Facteur du petit axe
        facteur_grand_axe: Facteur du grand axe
        facteur_dist: Facteur de distance pour validation des waypoints
        facteur_temps: Facteur de temps pour validation des waypoints
        angle_rotation: Angle de rotation de l'ellipse
        altitude_vol: Altitude de vol
        seuil_critique_capteur: Seuil critique du capteur

    Returns:
        tuple: (seuil_critique_atteint, tour_termine)
    """



    # Génération des coordonnées de l'ellipse
    coordonnees_ellipse = generer_coordonnees_ellipse_complete(rayon_ellipse, angle_orientation, nb_points_ellipse,facteur_petit_axe, facteur_grand_axe)

    print("Exécution de la trajectoire ellipse")
    position_actuelle = utils.get_position(master)

    

    # ... reste de la fonction inchangé ...
    position_depart = position_actuelle
    distance_min, index_proche = trouver_point_plus_proche(coordonnees_ellipse, position_actuelle)
    coordonnees_ordonnees = organiser_points_ellipse(coordonnees_ellipse, index_proche, -1)

    # Conversion en objets Position
    points_ellipse = []
    for lat, lon in coordonnees_ordonnees:
        points_ellipse.append(Position(lat_deg=lat, lon_deg=lon, relative_alt_m=altitude_vol))

    # Variables de contrôle de la trajectoire
    waypoint_valide = 1
    index_point_actuel = 0
    commande_envoyee = 0

    while True:
        # Vérification du niveau de batterie
        if utils.get_battery_cap(master) < niveau_batterie_min:
            return False, True, angle_orientation

        # Vérification si on est revenu près du point de départ
        if position_depart_cartographie is not None:
            distance_au_depart = calculer_distance_metres(position_actuelle, position_depart_cartographie)
            temps_ecoule = time.time() - temps_depart_cartographie
            # Vérifie si on est près du point de départ ET qu'assez de temps s'est écoulé
            if distance_au_depart < 10 and temps_ecoule > temps_min_tour_complet:  
                if not tour_nuage_complet:
                    print(f"Tour complet du nuage effectué en {temps_ecoule:.1f} secondes!")
                    tour_nuage_complet = True
                    return False, True, angle_orientation
            
        # Fin de l'ellipse atteinte
        if index_point_actuel == len(points_ellipse) - 1:
            print("Tour complet effectué")
            index_point_actuel = 0
            return False, True, angle_orientation

        # Envoi de la commande de navigation
        if commande_envoyee == 0:
            utils.simple_goto(master, points_ellipse[index_point_actuel].lat_deg,
                              points_ellipse[index_point_actuel].lon_deg,
                              points_ellipse[index_point_actuel].relative_alt_m)
            temps_debut = time.time()
            commande_envoyee = 1

        # Calcul de la distance entre points consécutifs
        index_suivant = index_point_actuel - 1
        if index_suivant == -1:
            index_suivant = 1
        distance_inter_points = calculer_distance_metres(points_ellipse[index_suivant],
                                                         points_ellipse[index_point_actuel])

        # Vérification de l'arrivée au waypoint
        position_courante = utils.get_position(master)
        distance_au_waypoint = calculer_distance_metres(position_courante, points_ellipse[index_point_actuel])
        temps_ecoule = time.time() - temps_debut

        if distance_au_waypoint < facteur_dist * distance_inter_points or temps_ecoule > distance_inter_points / facteur_temps:
            index_point_actuel += 1
            commande_envoyee = 0

        # Recherche du point le plus proche dans le nuage
        position_actuelle = utils.get_position(master)
        distances_nuage = np.full_like(latitudes_gps, np.inf)

        for i in range(latitudes_gps.shape[0]):
            for j in range(latitudes_gps.shape[1]):
                try:
                    distances_nuage[i, j] = calculer_distance_metres(
                        position_actuelle,
                        Position(lat_deg=latitudes_gps[i, j], lon_deg=longitudes_gps[i, j], relative_alt_m=altitude_vol)
                    )
                except:
                    distances_nuage[i, j] = np.inf

        # Analyse de la concentration au point le plus proche
        index_min = np.unravel_index(np.argmin(distances_nuage), distances_nuage.shape)
        concentration_actuelle = valeurs_capteur[index_min]
        distance_min_nuage = distances_nuage[index_min]

        print("Concentration PM2.5 :", concentration_actuelle)

        # On ajoute le point de données à la liste
        position_actuelle = utils.get_position(master)
        datapoint = DataPoint(position_actuelle, concentration_actuelle)
        datapoints.append(datapoint)

        # Vérification du seuil critique
        if concentration_actuelle > seuil_critique_capteur and distance_min_nuage < 15:
            print("Seuil critique dépassé ! Concentration =", concentration_actuelle)
            utils.set_mode(master, "LOITER")
            return True, False, angle_orientation

        # Gestion des seuils d'entrée et de sortie du nuage
        if concentration_actuelle > seuil_entree_nuage and distance_min_nuage < 15 and waypoint_valide == 2:
            print("Deuxième sortie du nuage, concentration =", concentration_actuelle)
            angle_inc = 0 # On annule l'incrément sur l'angle d'orientation  des ellipses
            position_fin = utils.get_position(master)
            dernier_bord_nuage = position_fin
            bearing = (calculer_cap_gps(position_depart, position_fin)) % 360
            print("-- Nouvel angle ", bearing)
            waypoint_valide = 1
            utils.set_mode(master, "LOITER")
            return False, False, bearing
        elif concentration_actuelle < seuil_sortie_nuage and waypoint_valide == 1 and distance_min_nuage < 15:
            waypoint_valide = 2


#################################################################################################################
# PROGRAMME PRINCIPAL
#################################################################################################################
# Récupération de l'origine du drone pour le repère ENU local
origine_drone = utils.get_position(master)

# Génération du nuage de particules
latitudes_gps, longitudes_gps, valeurs_capteur = generer_nuage_particules(
    fichier_concentration,
    fichier_altitude,
    altitude_coupe,
    centre_nuage
)

# Affichage des statistiques du nuage
print("Valeur minimale du capteur:", np.min(valeurs_capteur))
print("Valeur maximale du capteur:", np.max(valeurs_capteur))
print(f"Min: {valeurs_capteur.min():.2f}, Max: {valeurs_capteur.max():.2f}, "
      f"Moyenne: {valeurs_capteur.mean():.2f}, Écart-type: {valeurs_capteur.std():.2f}")

# Affichage du nuage sur la carte
afficher_nuage_sur_carte(
    latitudes_gps,
    longitudes_gps,
    valeurs_capteur,
    seuil_affichage=seuil_entree
)

# Configuration initiale
utils.set_param(master, 'WP_YAW_BEHAVIOR', 0)  # Angle de lacet (yaw) constant lors du vol
utils.set_param(master, 'WPNAV_SPEED', 1200)  # Vitesse horizontale de 12 cm/s

########## Procédure de décollage du drone ##########
utils.set_mode(master, "STABILIZE")

# Attente du passage manuel en mode AUTO via QGC
while True:
    print("En attente de AUTO ...")
    if utils.get_mode(master) == "AUTO":
        utils.set_param(master, 'WP_YAW_BEHAVIOR', 0)
        print("Lancement de la mission")
        print("Décollage")
        utils.arm_and_takeoff(master, altitude_vol)
        break
    time.sleep(0.5)

# Déplacement vers le centre du nuage
utils.simple_goto(
    master,
    centre_nuage.lat_deg,
    centre_nuage.lon_deg,
    centre_nuage.relative_alt_m
)

# Détection de l'entrée dans le nuage
while True:
    if detecter_entree_nuage(
        latitudes_gps,
        longitudes_gps,
        valeurs_capteur,
        seuil_entree_debut
    ):
        break

entree_nuage = utils.get_position(master)
dernier_bord_nuage = entree_nuage
position_depart_cartographie = entree_nuage  # Mémorisation du point de départ


print("Nuage détecté !")



# Suivi de frontière
for altitude_vol in range(altitude_max,(altitude_min-1),-pas_altitude):
    print(f"--- Début de la cartographie à {altitude_vol} m ---")
    
    temps_depart_cartographie = time.time()  # Mémorisation du temps de départ
    tour_nuage_complet = False
    angle = angle_vent

    utils.goto(master, entree_nuage.lat_deg, entree_nuage.lon_deg, altitude_vol)
    
    # Génération du nuage de particules
    latitudes_gps, longitudes_gps, valeurs_capteur = generer_nuage_particules(
    fichier_concentration,
    fichier_altitude,
    altitude_vol,
    centre_nuage
    )


    while True:
        if utils.get_battery_cap(master) < niveau_batterie_min:
            print("--- Batterie trop faible !!! ---")
            break

        utils.set_mode(master, "GUIDED")

        tour = False
        a_inc = 0
        b_inc = 0
        ang_inc = 0

        if utils.get_battery_cap(master) < niveau_batterie_min:
            print("--- Batterie trop faible !!! ---")
            break

        if tour_nuage_complet == True:
                print("--- Cartographie à l'altitude ", altitude_vol, " m effectuée... ---")
                break

        while True:

            angle = (angle + ang_inc) % 360
            print("Angle ellipse :", angle)
            print("Angle Vent :", angle_vent)

            # Exécution de la trajectoire en ellipse
            seuil_crit, tour, nouvel_angle = executer_trajectoire_ellipse(
                seuil_entree,
                seuil_sortie,
                valeurs_capteur,
                centre_nuage,
                angle,
                rayon_cercle,
                nombre_points,
                facteur_petit_axe + a_inc,
                facteur_grand_axe + b_inc,
                facteur_distance,
                facteur_temps,
                angle_vent,
                altitude_vol,
                seuil_critique
            )

            angle = nouvel_angle

            print("Angle recalculé :", angle)

            if tour_nuage_complet == True:
                print("--- Tour du nuage à l'altitude effectué ---")
                break
                #if altitude_vol > altitude_min:
                    #altitude_vol -= pas_altitude
                    #print(f"--- Descente à {altitude_vol} m ---")
                    #utils.simple_goto(master, entree_nuage.lat_deg, entree_nuage.lon_deg, altitude_vol)
                    #continue
                #else:
                    #print("--- Altitude minimale atteinte, fin de la mission ---")
                    #cartographie_complete = True

            if seuil_crit:
                print("--- Seuil critique dépassé ! ---")
                print("--- Réccupération : retour au dernier point du nuage... ---")
                utils.goto(master, dernier_bord_nuage.lat_deg, dernier_bord_nuage.lon_deg, dernier_bord_nuage.relative_alt_m)
                tour = True # On active l'ajustement de l'ellipse
                print("Retour terminé.")

            if utils.get_battery_cap(master) < niveau_batterie_min:
                print("--- Batterie faible, veuillez la recharger ---")
                break

            if tour:
                # Ajustement des paramètres de l'ellipse
                print("--- Ajustement des paramètres de l'ellipse ---")
                a_inc += (facteur_petit_axe + a_inc) * 0.3
                b_inc += (facteur_grand_axe + b_inc) * 0.3
                ang_inc -= 60
                tour = False
                continue

            a_inc = 0
            b_inc = 0
            ang_inc = 0
        
        time.sleep(0.1)

    if utils.get_battery_cap(master) < niveau_batterie_min:
            print("--- Batterie trop faible !!! Arrêt de la mission ---")
            break

################## Fin de mission ##################
print(" --- Cartographie complétée. Fin de la mission ---")
print("Passage en mode RTL, arrêt du programme dans 10 secondes ...")
utils.set_mode(master, "RTL")
time.sleep(10)
print("Affichage des points de données sur la carte en 3D")
utils.plot_datapoints_3d(datapoints, save_path="resultats/graphique_3d_interactif.html")
print("Fin du programme.")
