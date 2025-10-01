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

##################### PARAMETRES DE CONFIGURATION ###########################################

# Arguments d'entrée du programme avec valeurs par défaut
parser = argparse.ArgumentParser()
arg_nb_points = parser.add_argument('--NbPts', type=int, default=20)
arg_rayon = parser.add_argument('--Rayon', type=float, default=17)
arg_iterations = parser.add_argument('--it', type=int, default=4)
arg_facteur_distance = parser.add_argument('--fact_dist', type=float, default=2)
arg_vitesse_cercle = parser.add_argument('--VitesseCercle', type=float, default=3)
arg_grand_axe = parser.add_argument('--GAxe', type=float, default=1.6)
arg_petit_axe = parser.add_argument('--PAxe', type=float, default=1)
arg_vent = parser.add_argument('--Vent', type=float, default=90)
arg_altitude = parser.add_argument('--altitude', type=float, default=10)
arg_altitude_min = parser.add_argument('--altitude_min', type=float, default=6)
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
print("Altitude: %s" % arg_altitude)
print("Altitude minimale: %s" % arg_altitude_min)
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
    altitude_depart = args["altitude"]
except:
    altitude_depart = arg_altitude.default
    
    # Paramètres du drone
try:
    altitude_minimale = args["altitude_min"]
except:
    altitude_minimale = arg_altitude_min.default

# Paramètres du nuage de particules
seuil_critique = 150000000  # Seuil critique qui déclenche la procédure de fin de cartographie
seuil_entree_debut = 500000
seuil_entree = 500000
seuil_sortie = 500000

angle_inc = 0
position_depart_cartographie = None  # Position de départ de la cartographie
temps_depart_cartographie = None  # Temps de départ de la cartographie
temps_min_tour_complet = 120  # Temps minimum en secondes avant de considérer qu'un tour complet est possible
tour_nuage_complet = False  # Indique si le tour complet du nuage a été effectué
altitude_actuelle = altitude_depart  # Altitude actuelle du drone
pas_altitude = 1  # Pas de descente en mètres
#################################################################################################################

# Centre GPS du nuage
centre_nuage = Position(lat_deg=48.629687, lon_deg=7.787335, relative_alt_m=altitude_depart)

# Création de la carte
carte_folium = folium.Map(
    location=[centre_nuage.lat_deg, centre_nuage.lon_deg],
    zoom_start=17,
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
altitude_coupe = 10  # Altitude à extraire (en mètres)


################################# FONCTIONS #######################################

def calculer_distance_metres(position1, position2):
    """
    Calcule la distance entre deux points GPS en mètres.

    Args:
        position1: Premier point GPS (Position)
        position2: Deuxième point GPS (Position)

    Returns:
        float: Distance en mètres
    """
    delta_lat = position2.lat_deg - position1.lat_deg
    delta_lon = position2.lon_deg - position1.lon_deg
    return math.sqrt((delta_lat * delta_lat) + (delta_lon * delta_lon)) * 1.113195e5


def generer_coordonnees_ellipse(rayon, direction_cardinale, nb_points_ellipse, facteur_petit_axe, facteur_grand_axe,
                                angle_rotation):
    """
    Calcule les coordonnées GPS des points de l'ellipse à réaliser.
    Logique calquée sur la fonction Cercle pour assurer la cohérence.

    Args:
        rayon: Rayon du cercle de base
        direction_cardinale: Direction cardinale de l'ellipse par rapport au drone
        nb_points_ellipse: Nombre de points générés sur l'ellipse
        facteur_petit_axe: Facteur d'allongement (paramètre 'a' dans Cercle)
        facteur_grand_axe: Facteur d'allongement (paramètre 'b' dans Cercle)
        angle_rotation: Angle de rotation de l'ellipse en degrés

    Returns:
        list: Liste des coordonnées GPS [(lat, lon), ...]
    """
    angle_rotation_rad = angle_rotation * (math.pi / 180)
    direction_rad = (math.pi / 180) * direction_cardinale

    position_actuelle = utils.get_position(master)

    # Calcul du centre de l'ellipse - même logique que fonction Cercle
    # facteur_petit_axe (a) -> latitude, facteur_grand_axe (b) -> longitude
    latitude_centre = position_actuelle.lat_deg + rayon * facteur_petit_axe * math.cos(
        direction_rad) / constante_latitude
    longitude_centre = position_actuelle.lon_deg + rayon * facteur_grand_axe * math.sin(
        direction_rad) / constante_longitude

    coordonnees_ellipse = []
    for i in range(0, nb_points_ellipse):
        #angle_degres = (i / nb_points_ellipse) * 360
        angle_degres = (i / nb_points_ellipse) * 180
        angle_radians = (math.pi / 180) * angle_degres

        # Position en mètres par rapport au centre - même logique que fonction Cercle
        # facteur_petit_axe (a) -> x, facteur_grand_axe (b) -> y
        x_local = 2 * rayon * facteur_petit_axe * math.cos(angle_radians)
        y_local = rayon * facteur_grand_axe * math.sin(angle_radians)

        # Application de la rotation
        x_rotated = x_local * math.cos(angle_rotation_rad) - y_local * math.sin(angle_rotation_rad)
        y_rotated = x_local * math.sin(angle_rotation_rad) + y_local * math.cos(angle_rotation_rad)

        # Conversion en coordonnées GPS - même logique que fonction Cercle
        # x_rotated -> variation de latitude, y_rotated -> variation de longitude
        delta_lat = x_rotated / constante_latitude
        delta_lon = y_rotated / constante_longitude

        coordonnees_ellipse.append((latitude_centre + delta_lat, longitude_centre + delta_lon))

    return coordonnees_ellipse

def generer_coordonnees_demi_ellipse(rayon, direction_cardinale, nb_points,
                                     facteur_petit_axe, facteur_grand_axe):
    """
    Génère les coordonnées GPS d'une demi-ellipse (forme ◡) à partir de son point d'extrémité gauche,
    en l’alignant selon une direction donnée, avec le GRAND axe comme base.

    Args:
        rayon: Rayon de base (mètres)
        direction_cardinale: Orientation de l’ellipse (de 0 à 360, 0 = Nord, 90 = Est, etc.)
        nb_points: Nombre de points à générer
        facteur_petit_axe: Allongement du petit axe (vertical)
        facteur_grand_axe: Allongement du grand axe (horizontal)
        (Note: facteur_grand_axe est la base de la demi-ellipse)

    Returns:
        list: Liste des coordonnées GPS [(latitude, longitude), ...]
    """

    # Obtenir le point de départ GPS (extrémité gauche de l’ellipse)
    point_depart = utils.get_position(master)

    # Redéfinition des axes
    a = rayon * facteur_grand_axe  # demi-grand axe = base (horizontal)
    b = rayon * facteur_petit_axe  # demi-petit axe = hauteur (vertical)

    alpha = math.radians(direction_cardinale)  # direction de l'axe horizontal

    # Centre de l'ellipse = à mi-chemin entre les extrémités gauche et droite (dans la direction de base)
    delta_x_centre = a * math.cos(alpha)
    delta_y_centre = a * math.sin(alpha)

    delta_lat_centre = delta_y_centre / constante_latitude
    delta_lon_centre = delta_x_centre / constante_longitude

    lat_centre = point_depart.lat_deg + delta_lat_centre
    lon_centre = point_depart.lon_deg + delta_lon_centre

    # Angles pour dessiner la demi-ellipse (de 0° à 180°, en radians)
    angles = [math.radians(i) for i in range(0, 181, max(1, int(180 / nb_points)))]

    points = []
    for theta in angles:
        # Point local (dans repère ellipse, base = axe x)
        x = a * math.cos(theta)  # longe la base (grand axe)
        y = b * math.sin(theta)  # hauteur (petit axe)

        # Rotation selon direction_cardinale
        x_rot = x * math.cos(alpha) - y * math.sin(alpha)
        y_rot = x * math.sin(alpha) + y * math.cos(alpha)

        # Conversion en GPS
        delta_lat = y_rot / constante_latitude
        delta_lon = x_rot / constante_longitude

        lat = lat_centre + delta_lat
        lon = lon_centre + delta_lon

        points.append((lat, lon))

    return points

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


def generer_demi_ellipse_avec_segment(rayon, direction_cardinale, nb_points_ellipse,
                                       facteur_petit_axe, facteur_grand_axe):
    """
    Génère une demi-ellipse orientée (forme ◡) suivie d’un seul point supplémentaire,
    dans la direction du petit axe (perpendiculaire au grand axe).

    Args:
        rayon: Rayon de base (m)
        direction_cardinale: Orientation du grand axe (°)
        nb_points_ellipse: Nombre de points pour la demi-ellipse
        facteur_petit_axe: Facteur du petit axe
        facteur_grand_axe: Facteur du grand axe
        longueur_segment: Longueur du segment à ajouter (m)

    Returns:
        list: Liste de coordonnées GPS [(lat, lon), ...]
    """

    point_depart = utils.get_position(master)

    longueur_segment = 2*rayon

    a = rayon * facteur_petit_axe
    b = rayon * facteur_grand_axe

    alpha = math.radians(direction_cardinale)

    # Centre de l'ellipse
    delta_x_centre = a * math.cos(alpha)
    delta_y_centre = a * math.sin(alpha)

    lat_centre = point_depart.lat_deg + delta_y_centre / constante_latitude
    lon_centre = point_depart.lon_deg + delta_x_centre / constante_longitude

    # Demi-ellipse : angles de 0° à 180°
    angles = [math.radians(i) for i in range(0, 181, int(180 / nb_points_ellipse))]
    points = []

    for theta in angles:
        x = a * math.cos(theta)
        y = b * math.sin(theta)

        x_rot = x * math.cos(alpha) - y * math.sin(alpha)
        y_rot = x * math.sin(alpha) + y * math.cos(alpha)

        lat = lat_centre + y_rot / constante_latitude
        lon = lon_centre + x_rot / constante_longitude
        points.append((lat, lon))

    # Ajouter un seul point dans la direction du petit axe (perpendiculaire au grand axe)
    dernier_point = points[-1]
    direction_segment = alpha + math.pi / 2

    dx = longueur_segment * math.cos(direction_segment)
    dy = longueur_segment * math.sin(direction_segment)

    lat = dernier_point[0] + dy / constante_latitude
    lon = dernier_point[1] + dx / constante_longitude
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
                                 relative_alt_m=altitude_actuelle)
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

def determiner_position_relative(centre, position, direction_degres):
    """
    Détermine si une position se trouve à droite ou à gauche d'une direction donnée.

    Args:
        centre: Position GPS du centre (doit avoir lat_deg et lon_deg)
        position: Position GPS à tester (doit avoir lat_deg et lon_deg)
        direction_degres: Direction en degrés (0° = Nord, 90° = Est)

    Returns:
        int: 1 (droite), -1 (gauche), 0 (sur la ligne)
    """
    # Coordonnées relatives (approximation valable à petite échelle)
    offset_x = position.lon_deg - centre.lon_deg  # Est-Ouest
    offset_y = position.lat_deg - centre.lat_deg  # Nord-Sud

    # Vecteur direction normalisé à partir de l'angle donné (0° = Nord)
    # Converti en repère mathématique (0° = Est, 90° = Nord)
    angle_rad = math.radians(90 - direction_degres)
    vecteur_x = math.cos(angle_rad)
    vecteur_y = math.sin(angle_rad)

    # Produit vectoriel 2D pour déterminer la position relative
    produit_vectoriel = vecteur_x * offset_y - vecteur_y * offset_x

    # Seuil de tolérance
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



    # Génération des coordonnées de l'ellipse
    coordonnees_ellipse = generer_coordonnees_ellipse_complete(rayon_ellipse, angle_orientation, nb_points_ellipse,facteur_petit_axe, facteur_grand_axe)

    print(f"Exécution de la trajectoire ellipse à {altitude_actuelle}m")
    position_actuelle = utils.get_position(master)

    

    # ... reste de la fonction inchangé ...
    position_depart = position_actuelle
    distance_min, index_proche = trouver_point_plus_proche(coordonnees_ellipse, position_actuelle)
    coordonnees_ordonnees = organiser_points_ellipse(coordonnees_ellipse, index_proche, -1)

    # Conversion en objets Position
    points_ellipse = []
    for lat, lon in coordonnees_ordonnees:
        points_ellipse.append(Position(lat_deg=lat, lon_deg=lon, relative_alt_m=altitude_actuelle))

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
                        Position(lat_deg=latitudes_gps[i, j], lon_deg=longitudes_gps[i, j], relative_alt_m=altitude_actuelle)
                    )
                except:
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
    seuil_affichage=500000
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
        utils.goto(master, nouvelle_lat, nouvelle_lon, entree_nuage.relative_alt_m)

        # Mise à jour de la position de départ
        entree_nuage = utils.get_position(master)
        dernier_bord_nuage = entree_nuage
        position_depart_cartographie = entree_nuage  # Mémorisation du point de départ
        temps_depart_cartographie = time.time()  # Mémorisation du temps de départ
        print("Nuage détecté !")
        #angle = calculer_cap_gps(origine_drone, centre_nuage) % 360
        angle = angle_vent % 360
        break

# Suivi de frontière
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

        if tour:
            # Ajustement des paramètres de l'ellipse
            print("--- Ajustement des paramètres de l'ellipse ---")
            #a_inc += (facteur_petit_axe + a_inc) * 0.3
            #b_inc += (facteur_grand_axe + b_inc) * 0.3
            ang_inc -= 60
            tour = False
            continue
        
        if tour_nuage_complet:
            # Réinitialiser l'indicateur de tour complet
            tour_nuage_complet = False
            
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

################## Fin de mission ##################
print("Passage en mode RTL, arrêt du programme dans 10 secondes ...")
utils.set_mode(master, "RTL")
time.sleep(10)
print("Fin du programme.")
