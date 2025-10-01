#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ============================
# Imports et dépendances
# ============================
import time
import math
import numpy as np
import argparse
from pymavlink import mavutil
import folium
from folium import plugins
import matplotlib.cm as cm
import matplotlib.colors as colors
from branca.colormap import linear
import pymavlink_utils as utils
from pymavlink_utils import Position

# ============================
# Paramètres de configuration
# ============================
parser = argparse.ArgumentParser()
arg_nb_points = parser.add_argument('--NbPts', type=int, default=20)
arg_rayon = parser.add_argument('--Rayon', type=float, default=17)
arg_iterations = parser.add_argument('--it', type=int, default=4)
arg_facteur_distance = parser.add_argument('--fact_dist', type=float, default=2)
arg_vitesse_cercle = parser.add_argument('--VitesseCercle', type=float, default=3)
arg_grand_axe = parser.add_argument('--GAxe', type=float, default=1.6)
arg_petit_axe = parser.add_argument('--PAxe', type=float, default=1)
arg_vent = parser.add_argument('--Vent', type=float, default=90)
arg_altitude = parser.add_argument('--altitude', type=float, default=9)
arg_altitude_min = parser.add_argument('--altitude_min', type=float, default=6)
arg_angle = parser.add_argument('--angle', type=float, default=-180)
arg_batterie_min = parser.add_argument('--Min_battery', type=float, default=-10)
args = vars(parser.parse_args())

print("===== PARAMÈTRES DE LANCEMENT =====")
print(f"Nombre de points: {arg_nb_points}")
print(f"Rayon: {arg_rayon}")
print(f"Itérations: {arg_iterations}")
print(f"Facteur distance: {arg_facteur_distance}")
print(f"Vitesse cercle: {arg_vitesse_cercle}")
print(f"Grand axe: {arg_grand_axe}")
print(f"Petit axe: {arg_petit_axe}")
print(f"Vent: {arg_vent}")
print(f"Altitude: {arg_altitude}")
print(f"Altitude minimale: {arg_altitude_min}")
print(f"Angle: {arg_angle}")
print(f"Batterie min: {arg_batterie_min}")

# Paramètres du drone
vitesse_drone = 2

# ============================
# Récupération des paramètres utilisateur
# ============================
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


# ============================
# Paramètres du drone
# ============================
try:
    altitude_depart = args["altitude"]
except:
    altitude_depart = arg_altitude.default
    
    # Paramètres du drone
try:
    altitude_minimale = args["altitude_min"]
except:
    altitude_minimale = arg_altitude_min.default

# ============================
# Paramètres du nuage de particules
# ============================
seuil_critique = 150000000  # Seuil critique qui déclenche la procédure de fin de cartographie
seuil_entree_debut = 400000
seuil_entree = 500000
seuil_sortie = 500000  # Seuil de sortie du nuage pour le déplacement du drone

# ============================
# Paramètres de mission et initialisation
# ============================


# Variables de suivi de mission
angle_inc = 0
position_depart_cartographie = None  # Position de départ de la cartographie
temps_depart_cartographie = None     # Temps de départ de la cartographie
temps_min_tour_complet = 120        # Temps minimum avant de considérer qu'un tour complet est possible
tour_nuage_complet = False          # Indique si le tour complet du nuage a été effectué
altitude_actuelle = altitude_depart # Altitude actuelle du drone
pas_altitude = 1                    # Pas de descente en mètres

# ============================
# Centre GPS du nuage et carte Folium
# ============================
centre_nuage = Position(lat_deg=48.629687, lon_deg=7.787335, relative_alt_m=altitude_depart)

# Carte Folium pour visualisation
carte_folium = folium.Map(
    location=[centre_nuage.lat_deg, centre_nuage.lon_deg],
    zoom_start=17,
    tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attr='Esri'
)

# ============================
# Constantes de conversion GPS
# ============================
constante_latitude = 111320
constante_longitude = 111320 * math.cos(math.radians(centre_nuage.lat_deg))

# ============================
# Connexion MAVLink
# ============================
print('Connexion au véhicule en cours...')
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()
print("Drone connecté")

# ============================
# Paramètres du modèle de concentration
# ============================
fichier_concentration = "concentration.npy"
fichier_altitude = "altitudes.npy"
altitude_coupe = 10  # Altitude à extraire (en mètres)

# ============================
# Fonctions principales
# ============================


def calculer_distance_metres(position1, position2):
    # ...existing code...
    delta_lat = position2.lat_deg - position1.lat_deg
    delta_lon = position2.lon_deg - position1.lon_deg
    return math.sqrt((delta_lat * delta_lat) + (delta_lon * delta_lon)) * 1.113195e5



def generer_coordonnees_ellipse_complete(rayon, direction_cardinale, nb_points,
                                         facteur_petit_axe, facteur_grand_axe):
    """
    Génère les coordonnées GPS d'une ellipse complète sous forme d'objets Position.
    """
    point_depart = utils.get_position(master)
    a = rayon * facteur_grand_axe
    b = rayon * facteur_petit_axe
    alpha = math.radians(direction_cardinale)
    delta_x_centre = a * math.cos(alpha)
    delta_y_centre = a * math.sin(alpha)
    delta_lat_centre = delta_y_centre / constante_latitude
    delta_lon_centre = delta_x_centre / constante_longitude
    lat_centre = point_depart.lat_deg + delta_lat_centre
    lon_centre = point_depart.lon_deg + delta_lon_centre
    angles = np.linspace(0, 2 * np.pi, nb_points, endpoint=False)
    return [Position(
        lat_deg=lat_centre + (b * math.sin(theta) * math.cos(alpha) + a * math.cos(theta) * math.sin(alpha)) / constante_latitude,
        lon_deg=lon_centre + (a * math.cos(theta) * math.cos(alpha) - b * math.sin(theta) * math.sin(alpha)) / constante_longitude,
        relative_alt_m=point_depart.relative_alt_m
    ) for theta in angles]




def trouver_point_plus_proche(coordonnees_ellipse, position_actuelle):
    """
    Trouve le point GPS le plus proche d'une position donnée.

    Args:
        coordonnees_ellipse: Liste des coordonnées GPS
        position_actuelle: Position GPS actuelle

    Returns:
        tuple: (distance_minimale, index_point_proche)
    """
    distances = [calculer_distance_metres(point, position_actuelle) for point in coordonnees_ellipse]
    index_proche = int(np.argmin(distances))
    return distances[index_proche], index_proche


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
    coordonnees_finales = coordonnees_ellipse[index_depart:] + coordonnees_ellipse[:index_depart]
    if sens_parcours == -1:
        coordonnees_finales = list(reversed(coordonnees_finales))
    coordonnees_finales.append(coordonnees_finales[0])
    return coordonnees_finales

def calculer_cap_gps(position1, position2):
    """
    Calcule le cap (bearing) entre deux points GPS.
    Convention : Nord = 0°, Est = 90°, Sud = 180°, Ouest = 270°
    """
    lat1_rad = math.radians(position1.lat_deg)
    lat2_rad = math.radians(position2.lat_deg)
    lon1_rad = math.radians(position1.lon_deg)
    lon2_rad = math.radians(position2.lon_deg)
    delta_lon = lon2_rad - lon1_rad
    y = math.sin(delta_lon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
    bearing_rad = math.atan2(y, x)
    bearing_deg = (math.degrees(bearing_rad) + 360) % 360  # Nord=0°, Est=90°
    return bearing_deg

def determiner_position_relative(centre, position, direction_degres):
    """
    Détermine si une position se trouve à droite ou à gauche d'une direction donnée.
    Convention : direction_degres = 0° (Nord), 90° (Est), 180° (Sud), 270° (Ouest)

    Args:
        centre: Position GPS du centre (doit avoir lat_deg et lon_deg)
        position: Position GPS à tester (doit avoir lat_deg et lon_deg)
        direction_degres: Direction en degrés (0° = Nord, 90° = Est)

    Returns:
        int: 1 (droite), -1 (gauche), 0 (sur la ligne)
    """
    offset_x = position.lon_deg - centre.lon_deg  # Est-Ouest
    offset_y = position.lat_deg - centre.lat_deg  # Nord-Sud
    # Vecteur direction selon convention drone : Nord=0°, Est=90°
    angle_rad = math.radians(direction_degres)
    vecteur_x = math.sin(angle_rad)
    vecteur_y = math.cos(angle_rad)
    produit_vectoriel = vecteur_x * offset_y - vecteur_y * offset_x
    tolerance = 1e-10
    if abs(produit_vectoriel) < tolerance:
        return 0  # Sur la ligne
    elif produit_vectoriel > 0:
        return -1  # À gauche
    else:
        return 1  # À droite

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

    # Calcul des pas de discrétisation
    pas_nord_sud = longueur_zone / nb_lignes
    pas_est_ouest = largeur_zone / nb_colonnes

    # Création des grilles d'indices
    idx_i = np.arange(nb_lignes)
    idx_j = np.arange(nb_colonnes)
    grid_i, grid_j = np.meshgrid(idx_i, idx_j, indexing='ij')

    # Calcul vectorisé des coordonnées relatives au centre de concentration
    delta_nord = (grid_i - ligne_centre) * pas_nord_sud
    delta_est = (grid_j - colonne_centre) * pas_est_ouest

    # Conversion vers coordonnées GPS (vectorisé)
    # utils.enu_to_gps doit être vectorisé ou appliqué via np.vectorize
    enu_to_gps_vec = np.vectorize(lambda de, dn: utils.enu_to_gps(de, dn, altitude_extraction,
                                                                 centre_gps.lat_deg, centre_gps.lon_deg, altitude_extraction)[:2])
    latitudes_gps, longitudes_gps = enu_to_gps_vec(delta_est, delta_nord)

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


def detecter_entree_nuage(latitudes_gps, longitudes_gps, valeurs_capteur, seuil_detection, altitude_vol):
    """
    Détecte si le drone entre dans le nuage de particules.

    Args:
        latitudes_gps: Coordonnées de latitude du nuage
        longitudes_gps: Coordonnées de longitude du nuage
        valeurs_capteur: Valeurs de concentration
        seuil_detection: Seuil de détection d'entrée
        altitude_vol: Altitude de vol

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
    if valeur_proche > seuil_detection:
        print("Valeur PM 2.5 : ", valeur_proche)
        print(f"Entrée dans le nuage détectée à {distance_min:.2f} mètres : LOITER")
        utils.set_mode(master, "LOITER")
        return True

    return False


def executer_trajectoire_ellipse(seuil_entree_nuage, seuil_sortie_nuage, valeurs_capteur, centre_nuage,
                                 angle_orientation, rayon_ellipse, nb_points_ellipse, facteur_petit_axe,
                                 facteur_grand_axe, facteur_dist, facteur_temps, angle_rotation,
                                 altitude_actuelle, seuil_critique_capteur):
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
        altitude_actuelle : Altitude de vol
        seuil_critique_capteur: Seuil critique du capteur

    Returns:
        tuple: (seuil_critique_atteint, tour_termine)
    """



    # Génération des coordonnées de l'ellipse (déjà des objets Position)
    coordonnees_ellipse = generer_coordonnees_ellipse_complete(rayon_ellipse, angle_orientation, nb_points_ellipse, facteur_petit_axe, facteur_grand_axe)
    print(f"Exécution de la trajectoire ellipse à {altitude_actuelle}m")
    position_actuelle = utils.get_position(master)
    position_depart = position_actuelle
    distance_min, index_proche = trouver_point_plus_proche(coordonnees_ellipse, position_actuelle)
    points_ellipse = organiser_points_ellipse(coordonnees_ellipse, index_proche, -1)
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
            if distance_au_depart < 2 * rayon_ellipse and temps_ecoule > temps_min_tour_complet:  
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
        index_suivant = (index_point_actuel - 1) % len(points_ellipse)
        distance_inter_points = calculer_distance_metres(points_ellipse[index_suivant], points_ellipse[index_point_actuel])

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
                        Position(lat_deg=latitudes_gps[i, j], lon_deg=longitudes_gps[i, j], relative_alt_m=altitude_actuelle)
                    )
                except (TypeError, ValueError):
                    distances_nuage[i, j] = np.inf

        # Analyse de la concentration au point le plus proche
        index_min = np.unravel_index(np.argmin(distances_nuage), distances_nuage.shape)
        concentration_actuelle = valeurs_capteur[index_min]
        distance_min_nuage = distances_nuage[index_min]

        print("Concentration PM2.5 :", concentration_actuelle)

        # Vérification du seuil critique
        if concentration_actuelle > seuil_critique_capteur and distance_min_nuage < 15:
            print("Seuil critique dépassé ! Concentration =", concentration_actuelle)
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
            return False, False, bearing
        elif concentration_actuelle < seuil_sortie_nuage and waypoint_valide == 1 and distance_min_nuage < 15:
            waypoint_valide = 2


# ============================
# PROGRAMME PRINCIPAL
# ============================
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
    seuil_affichage=500000
)

# Configuration initiale
utils.set_param(master, 'WP_YAW_BEHAVIOR', 0)  # Angle de lacet (yaw) constant lors du vol
utils.set_param(master, 'WPNAV_SPEED', 1200)  # Vitesse horizontale de 12 cm/s

# ============================
# Procédure de décollage du drone
# ============================
utils.set_mode(master, "STABILIZE")

# Attente du passage manuel en mode AUTO via QGC
while True:
    print("En attente de AUTO ...")
    if utils.get_mode(master) == "AUTO":
        utils.set_param(master, 'WP_YAW_BEHAVIOR', 0)
        print("Lancement de la mission")
        print("Décollage")
        utils.arm_and_takeoff(master, altitude_depart)
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
entree_nuage = utils.get_position(master)
dernier_bord_nuage = entree_nuage
position_depart_cartographie = entree_nuage  # Mémorisation du point de départ
temps_depart_cartographie = time.time()  # Mémorisation du temps de départ

print("Nuage détecté !")

angle = calculer_cap_gps(origine_drone, centre_nuage) % 360
while True:
    if detecter_entree_nuage(
        latitudes_gps,
        longitudes_gps,
        valeurs_capteur,
        seuil_entree_debut,
        altitude_depart
    ):
        # Décalage du drone à l'extérieur du nuage
        entree_nuage = utils.get_position(master)
        # Vecteur du centre vers le drone
        delta_lat = entree_nuage.lat_deg - centre_nuage.lat_deg
        delta_lon = entree_nuage.lon_deg - centre_nuage.lon_deg
        norme = math.sqrt(delta_lat**2 + delta_lon**2)
        # Distance de recul en mètres
        recul_m = 5
        # Conversion en degrés
        recul_lat = (delta_lat / norme) * (recul_m / constante_latitude) if norme != 0 else 0
        recul_lon = (delta_lon / norme) * (recul_m / constante_longitude) if norme != 0 else 0
        nouvelle_lat = entree_nuage.lat_deg + recul_lat
        nouvelle_lon = entree_nuage.lon_deg + recul_lon
        # Déplacement du drone
        print(f"Décalage du drone de {recul_m}m à l'extérieur du nuage...")
        #utils.goto(master, nouvelle_lat, nouvelle_lon, entree_nuage.relative_alt_m)

        # Mise à jour de la position de départ
        entree_nuage = utils.get_position(master)
        dernier_bord_nuage = entree_nuage
        position_depart_cartographie = entree_nuage  # Mémorisation du point de départ
        temps_depart_cartographie = time.time()  # Mémorisation du temps de départ
        print("Nuage détecté !")
        #angle = calculer_cap_gps(origine_drone, centre_nuage) % 360
        angle = angle_vent % 360
        break

# ============================
# Suivi de frontière
# ============================
while True:
    
    if utils.get_battery_cap(master) < niveau_batterie_min:
        break

    utils.set_mode(master, "GUIDED")

    tour = False
    a_inc = 0
    b_inc = 0
    ang_inc = 0

    while True:
        angle = (angle + ang_inc) % 360
        print(f"Altitude actuelle : {altitude_actuelle}m")
        print("Angle ellipse :", angle)
        print("Angle_inc, :", ang_inc)
        #print("Angle Vent :", angle_vent)

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
            altitude_actuelle,
            seuil_critique
        )

        angle = nouvel_angle

        print("Angle recalculé :", angle)


        if seuil_crit:
            print("--- Seuil critique dépassé ! ---")
            print("--- Réccupération : retour au dernier point du nuage... ---")
            utils.goto(master, dernier_bord_nuage.lat_deg, dernier_bord_nuage.lon_deg, dernier_bord_nuage.relative_alt_m)
            tour = True # On active l'ajustement de l'ellipse
            print("Retour terminé.")

        if utils.get_battery_cap(master) < niveau_batterie_min:
            print("--- Batterie faible, veuillez la recharger ---")
            break
        
        if tour_nuage_complet:
            # Réinitialiser l'indicateur de tour complet
            tour = False
            a_inc = 0
            b_inc = 0
            ang_inc = 0
            
            # Calculer la nouvelle altitude
            nouvelle_altitude = altitude_actuelle - pas_altitude
            
            if nouvelle_altitude < altitude_minimale:
                print("--- Altitude minimale atteinte, fin de la cartographie ---")
                break
                
            print(f"--- Tour du nuage effectué à {altitude_actuelle}m ---")
            print(f"--- Descente à {nouvelle_altitude}m ---")
            
            # Descendre à la nouvelle altitude
            altitude_actuelle = nouvelle_altitude
            position_actuelle = utils.get_position(master)
            utils.goto(
                master,
                position_depart_cartographie.lat_deg,
                position_depart_cartographie.lon_deg,
                altitude_actuelle
            )

            time.sleep(2)
            

            
            # Générer une nouvelle coupe du nuage à cette altitude
            latitudes_gps, longitudes_gps, valeurs_capteur = generer_nuage_particules(
                fichier_concentration,
                fichier_altitude,
                altitude_actuelle,
                centre_nuage
            )
            
            # Réinitialiser les variables pour un nouveau tour
            #position_depart_cartographie = utils.get_position(master)
            temps_depart_cartographie = time.time()
            break  # Sortir de la boucle interne pour recommencer avec les nouveaux paramètres

        if tour:
            # Ajustement des paramètres de l'ellipse
            print("--- Ajustement des paramètres de l'ellipse ---")
            #a_inc += (facteur_petit_axe + a_inc) * 0.3
            #b_inc += (facteur_grand_axe + b_inc) * 0.3
            ang_inc -= 60
            tour = False
            continue

        a_inc = 0
        b_inc = 0
        ang_inc = 0

    if utils.get_battery_cap(master) < niveau_batterie_min:
        print("--- Batterie trop faible !!! ---")
        break

    # La condition de fin globale est déplacée à la fin
    if altitude_actuelle <= altitude_minimale:
        print("--- Cartographie 3D terminée ---")
        break
    
    time.sleep(0.1)

# ============================
# Fin de mission
# ============================
print("Passage en mode RTL, arrêt du programme dans 10 secondes ...")
utils.set_mode(master, "RTL")
time.sleep(10)
print("Fin du programme.")
