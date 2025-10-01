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
from enum import Enum
import logging

import pymap3d as pm  # Bibliothèque pour la conversion GPS / ENU


##################### MACHINE À ÉTATS ###########################################

class EtatMission(Enum):
    """États de la mission de cartographie"""
    INITIALISATION = "initialisation"
    DECOLLAGE = "decollage"
    RECHERCHE_NUAGE = "recherche_nuage"
    SUIVI_FRONTIERE = "suivi_frontiere"
    AJUSTEMENT_ELLIPSE = "ajustement_ellipse"
    SORTIE_NUAGE = "sortie_nuage"
    CRITIQUE = "critique"
    BATTERIE_FAIBLE = "batterie_faible"
    RETOUR_BASE = "retour_base"
    TERMINE = "termine"


class GestionnaireMission:
    """Gestionnaire d'état de la mission"""

    def __init__(self):
        self.etat_actuel = EtatMission.INITIALISATION
        self.historique = []
        self.compteur_tours = 0
        self.temps_debut_etat = time.time()
        self.timeout_etat = 300  # 5 minutes max par état

    def changer_etat(self, nouvel_etat, raison=""):
        """Change l'état avec logging"""
        ancien_etat = self.etat_actuel
        self.historique.append((ancien_etat, time.time()))
        self.etat_actuel = nouvel_etat
        self.temps_debut_etat = time.time()

        print(f"[ÉTAT] {ancien_etat.value} -> {nouvel_etat.value}")
        if raison:
            print(f"[RAISON] {raison}")

    def est_timeout(self):
        """Vérifie si l'état actuel a dépassé son timeout"""
        return (time.time() - self.temps_debut_etat) > self.timeout_etat

    def peut_continuer(self):
        """Vérifie si la mission peut continuer"""
        return self.etat_actuel not in [EtatMission.CRITIQUE, EtatMission.BATTERIE_FAIBLE,
                                        EtatMission.RETOUR_BASE, EtatMission.TERMINE]


##################### GESTION DES COORDONNÉES UNIFIÉE #######################

class GestionnaireCoordonnees:
    """Gestionnaire unifié pour toutes les conversions de coordonnées"""

    def __init__(self, origine_gps):
        """
        Initialise avec une origine GPS pour le repère ENU local

        Args:
            origine_gps: Position GPS d'origine (lat, lon, alt)
        """
        self.origine_lat = origine_gps.lat_deg
        self.origine_lon = origine_gps.lon_deg
        self.origine_alt = origine_gps.relative_alt_m

    def gps_to_enu(self, lat, lon, alt):
        """Convertit GPS vers ENU"""
        return pm.geodetic2enu(lat, lon, alt,
                               self.origine_lat, self.origine_lon, self.origine_alt)

    def enu_to_gps(self, east, north, up):
        """Convertit ENU vers GPS"""
        return pm.enu2geodetic(east, north, up,
                               self.origine_lat, self.origine_lon, self.origine_alt)

    def distance_gps(self, pos1, pos2):
        """Calcule la distance entre deux positions GPS (méthode précise)"""
        e1, n1, u1 = self.gps_to_enu(pos1.lat_deg, pos1.lon_deg, pos1.relative_alt_m)
        e2, n2, u2 = self.gps_to_enu(pos2.lat_deg, pos2.lon_deg, pos2.relative_alt_m)
        return math.sqrt((e2 - e1) ** 2 + (n2 - n1) ** 2 + (u2 - u1) ** 2)

    def generer_cercle_enu(self, centre_enu, rayon, nb_points, facteur_a=1, facteur_b=1, angle_rotation=0):
        """
        Génère un cercle/ellipse en coordonnées ENU

        Args:
            centre_enu: Centre en ENU (east, north, up)
            rayon: Rayon en mètres
            nb_points: Nombre de points
            facteur_a: Facteur d'allongement axe est-ouest
            facteur_b: Facteur d'allongement axe nord-sud
            angle_rotation: Angle de rotation en degrés
        """
        points_enu = []
        angle_rad = math.radians(angle_rotation)

        for i in range(nb_points):
            theta = (i / nb_points) * 2 * math.pi

            # Point sur l'ellipse
            x = rayon * facteur_a * math.cos(theta)
            y = rayon * facteur_b * math.sin(theta)

            # Rotation
            x_rot = x * math.cos(angle_rad) - y * math.sin(angle_rad)
            y_rot = x * math.sin(angle_rad) + y * math.cos(angle_rad)

            # Translation vers le centre
            east = centre_enu[0] + x_rot
            north = centre_enu[1] + y_rot
            up = centre_enu[2]

            points_enu.append((east, north, up))

        return points_enu

    def convertir_points_enu_vers_gps(self, points_enu):
        """Convertit une liste de points ENU vers GPS"""
        points_gps = []
        for east, north, up in points_enu:
            lat, lon, alt = self.enu_to_gps(east, north, up)
            points_gps.append(Position(lat_deg=lat, lon_deg=lon, relative_alt_m=alt))
        return points_gps


##################### LOGIQUE DE SUIVI DE FRONTIÈRE AMÉLIORÉE ###############

class SuiviFrontiere:
    """Gestion du suivi de frontière avec logique d'état claire"""

    def __init__(self, gestionnaire_coords, seuil_entree, seuil_sortie, seuil_critique):
        self.coords = gestionnaire_coords
        self.seuil_entree = seuil_entree
        self.seuil_sortie = seuil_sortie
        self.seuil_critique = seuil_critique

        # États du suivi
        self.dans_nuage = False
        self.nb_detections_consecutives = 0
        self.nb_sorties_consecutives = 0
        self.seuil_confirmation = 3  # Nb de mesures consécutives pour confirmer

        # Historique des mesures
        self.historique_mesures = []
        self.taille_historique = 10

    def analyser_mesure(self, position_drone, GPSLat, GPSLon, SensorValue):
        """
        Analyse une mesure et retourne l'état de détection

        Returns:
            tuple: (etat, valeur_mesuree, distance_min)
            etat: 'entree', 'dans_nuage', 'sortie', 'critique', 'hors_nuage'
        """
        # Trouver la valeur la plus proche
        valeur, distance = self._trouver_valeur_plus_proche(position_drone, GPSLat, GPSLon, SensorValue)

        # Ajouter à l'historique
        self.historique_mesures.append((valeur, distance, time.time()))
        if len(self.historique_mesures) > self.taille_historique:
            self.historique_mesures.pop(0)

        # Vérification critique (priorité absolue)
        if valeur > self.seuil_critique and distance < 15:
            return 'critique', valeur, distance

        # Analyse avec confirmation
        if valeur > self.seuil_entree and distance < 10:
            self.nb_detections_consecutives += 1
            self.nb_sorties_consecutives = 0

            if not self.dans_nuage and self.nb_detections_consecutives >= self.seuil_confirmation:
                self.dans_nuage = True
                return 'entree', valeur, distance
            elif self.dans_nuage:
                return 'dans_nuage', valeur, distance

        elif valeur < self.seuil_sortie and distance < 10:
            self.nb_sorties_consecutives += 1
            self.nb_detections_consecutives = 0

            if self.dans_nuage and self.nb_sorties_consecutives >= self.seuil_confirmation:
                self.dans_nuage = False
                return 'sortie', valeur, distance

        # Réinitialiser les compteurs si pas de détection claire
        if distance > 10:
            self.nb_detections_consecutives = 0
            self.nb_sorties_consecutives = 0

        return 'hors_nuage' if not self.dans_nuage else 'dans_nuage', valeur, distance

    def _trouver_valeur_plus_proche(self, position_drone, GPSLat, GPSLon, SensorValue):
        """Trouve la valeur de capteur la plus proche avec moyennage local"""
        distances = np.full_like(GPSLat, np.inf)

        # Calcul des distances
        for i in range(GPSLat.shape[0]):
            for j in range(GPSLat.shape[1]):
                try:
                    pos_capteur = Position(lat_deg=GPSLat[i, j], lon_deg=GPSLon[i, j],
                                           relative_alt_m=position_drone.relative_alt_m)
                    distances[i, j] = self.coords.distance_gps(position_drone, pos_capteur)
                except:
                    continue

        # Trouve le point le plus proche
        idx_min = np.unravel_index(np.argmin(distances), distances.shape)
        distance_min = distances[idx_min]

        # Moyennage local dans un rayon de 5m
        valeurs_locales = []
        for i in range(max(0, idx_min[0] - 2), min(GPSLat.shape[0], idx_min[0] + 3)):
            for j in range(max(0, idx_min[1] - 2), min(GPSLat.shape[1], idx_min[1] + 3)):
                if distances[i, j] < distance_min + 5:  # Dans un rayon de 5m
                    valeurs_locales.append(SensorValue[i, j])

        valeur_moyenne = np.mean(valeurs_locales) if valeurs_locales else SensorValue[idx_min]

        return valeur_moyenne, distance_min

    def reinitialiser(self):
        """Réinitialise l'état du suivi"""
        self.dans_nuage = False
        self.nb_detections_consecutives = 0
        self.nb_sorties_consecutives = 0
        self.historique_mesures = []


##################### GÉNÉRATEUR DE TRAJECTOIRE AMÉLIORÉ ####################

class GenerateurTrajectoire:
    """Génère et gère les trajectoires d'ellipse avec ajustement adaptatif"""

    def __init__(self, gestionnaire_coords, centre_gps, params_initiaux):
        self.coords = gestionnaire_coords
        self.centre_gps = centre_gps
        self.centre_enu = self.coords.gps_to_enu(centre_gps.lat_deg, centre_gps.lon_deg, centre_gps.relative_alt_m)

        # Paramètres de base
        self.rayon_base = params_initiaux['rayon']
        self.nb_points = params_initiaux['nb_points']
        self.facteur_a_base = params_initiaux['facteur_a']
        self.facteur_b_base = params_initiaux['facteur_b']
        self.angle_base = params_initiaux['angle']

        # Paramètres actuels (modifiés par ajustement)
        self.rayon_actuel = self.rayon_base
        self.facteur_a_actuel = self.facteur_a_base
        self.facteur_b_actuel = self.facteur_b_base
        self.angle_actuel = self.angle_base

        # État de la trajectoire
        self.points_actuels = []
        self.indice_point_actuel = 0
        self.nb_ajustements = 0

    def generer_trajectoire(self, position_drone):
        """Génère une nouvelle trajectoire en fonction de la position du drone"""
        # Générer les points en ENU
        points_enu = self.coords.generer_cercle_enu(
            self.centre_enu, self.rayon_actuel, self.nb_points,
            self.facteur_a_actuel, self.facteur_b_actuel, self.angle_actuel
        )

        # Convertir en GPS
        self.points_actuels = self.coords.convertir_points_enu_vers_gps(points_enu)

        # Trouver le point le plus proche du drone
        self.indice_point_actuel = self._trouver_point_plus_proche(position_drone)

        # Réorganiser pour commencer par le point le plus proche
        self.points_actuels = (self.points_actuels[self.indice_point_actuel:] +
                               self.points_actuels[:self.indice_point_actuel])
        self.indice_point_actuel = 0

        return self.points_actuels

    def point_suivant(self):
        """Retourne le prochain point de la trajectoire"""
        if not self.points_actuels:
            return None

        point = self.points_actuels[self.indice_point_actuel]
        self.indice_point_actuel = (self.indice_point_actuel + 1) % len(self.points_actuels)
        return point

    def ajuster_parametres(self):
        """Ajuste les paramètres de l'ellipse (agrandissement et rotation)"""
        self.nb_ajustements += 1

        # Agrandissement progressif
        facteur_agrandissement = 1 + (0.3 * self.nb_ajustements)
        self.rayon_actuel = self.rayon_base * facteur_agrandissement
        self.facteur_a_actuel = self.facteur_a_base * facteur_agrandissement
        self.facteur_b_actuel = self.facteur_b_base * facteur_agrandissement

        # Rotation progressive
        self.angle_actuel = (self.angle_base + 40 * self.nb_ajustements) % 360

        print(f"[AJUSTEMENT] Rayon: {self.rayon_actuel:.1f}m, Angle: {self.angle_actuel:.1f}°")

    def reinitialiser(self):
        """Remet les paramètres à leurs valeurs initiales"""
        self.rayon_actuel = self.rayon_base
        self.facteur_a_actuel = self.facteur_a_base
        self.facteur_b_actuel = self.facteur_b_base
        self.angle_actuel = self.angle_base
        self.nb_ajustements = 0
        self.points_actuels = []
        self.indice_point_actuel = 0

    def _trouver_point_plus_proche(self, position_drone):
        """Trouve l'indice du point le plus proche du drone"""
        distance_min = float('inf')
        indice_min = 0

        for i, point in enumerate(self.points_actuels):
            distance = self.coords.distance_gps(position_drone, point)
            if distance < distance_min:
                distance_min = distance
                indice_min = i

        return indice_min


##################### FONCTION PRINCIPALE AVEC MACHINE À ÉTATS ##############

def mission_principale():
    """Fonction principale avec gestion d'état claire"""

    # Initialisation
    gestionnaire_mission = GestionnaireMission()

    # Vos paramètres existants...
    # (gardez votre code d'arguments et de paramètres)

    try:
        while gestionnaire_mission.peut_continuer():
            etat = gestionnaire_mission.etat_actuel

            # Vérifications de sécurité globales
            if get_battery_cap() < batt_min:
                gestionnaire_mission.changer_etat(EtatMission.BATTERIE_FAIBLE, "Batterie faible")
                continue

            if gestionnaire_mission.est_timeout():
                gestionnaire_mission.changer_etat(EtatMission.RETOUR_BASE, "Timeout état")
                continue

            # Machine à états
            if etat == EtatMission.INITIALISATION:
                executer_initialisation(gestionnaire_mission)

            elif etat == EtatMission.DECOLLAGE:
                executer_decollage(gestionnaire_mission)

            elif etat == EtatMission.RECHERCHE_NUAGE:
                executer_recherche_nuage(gestionnaire_mission)

            elif etat == EtatMission.SUIVI_FRONTIERE:
                executer_suivi_frontiere(gestionnaire_mission)

            elif etat == EtatMission.AJUSTEMENT_ELLIPSE:
                executer_ajustement_ellipse(gestionnaire_mission)

            elif etat == EtatMission.CRITIQUE:
                executer_procedure_critique(gestionnaire_mission)

            elif etat == EtatMission.BATTERIE_FAIBLE:
                executer_batterie_faible(gestionnaire_mission)

            elif etat == EtatMission.RETOUR_BASE:
                executer_retour_base(gestionnaire_mission)

            time.sleep(0.1)  # Éviter la surcharge CPU

    except KeyboardInterrupt:
        print("[ARRÊT] Interruption manuelle")
        gestionnaire_mission.changer_etat(EtatMission.RETOUR_BASE, "Arrêt manuel")

    except Exception as e:
        print(f"[ERREUR] Exception: {e}")
        gestionnaire_mission.changer_etat(EtatMission.RETOUR_BASE, f"Erreur: {e}")

    finally:
        # Nettoyage final
        set_mode("RTL")
        print("[FIN] Mission terminée")


# Fonctions d'exécution pour chaque état
def executer_initialisation(gestionnaire_mission):
    """Initialise tous les composants"""
    global coords_manager, suivi_frontiere, generateur_traj

    # Initialiser les gestionnaires
    origine = get_position()
    coords_manager = GestionnaireCoordonnees(origine)

    suivi_frontiere = SuiviFrontiere(coords_manager, Seuil_entree, Seuil_sortie, Seuil_Critique)

    params_traj = {
        'rayon': RayonCercle,
        'nb_points': NbPoints,
        'facteur_a': a,
        'facteur_b': b,
        'angle': ang_ellipse
    }
    generateur_traj = GenerateurTrajectoire(coords_manager, centre, params_traj)

    gestionnaire_mission.changer_etat(EtatMission.DECOLLAGE, "Initialisation terminée")


def executer_decollage(gestionnaire_mission):
    """Gère le décollage"""
    if get_mode() == "AUTO":
        arm_and_takeoff(alt)
        gestionnaire_mission.changer_etat(EtatMission.RECHERCHE_NUAGE, "Décollage terminé")


def executer_recherche_nuage(gestionnaire_mission):
    """Recherche le nuage"""
    simple_goto(centre.lat_deg, centre.lon_deg, centre.relative_alt_m)

    if ToutDroitStop(GPSLat, GPSLon, SensorValue, Seuil_entree_debut, alt):
        gestionnaire_mission.changer_etat(EtatMission.SUIVI_FRONTIERE, "Nuage détecté")


def executer_suivi_frontiere(gestionnaire_mission):
    """Exécute le suivi de frontière"""
    position = get_position()
    etat_detection, valeur, distance = suivi_frontiere.analyser_mesure(position, GPSLat, GPSLon, SensorValue)

    if etat_detection == 'critique':
        gestionnaire_mission.changer_etat(EtatMission.CRITIQUE, f"Seuil critique: {valeur}")
    elif etat_detection == 'sortie':
        gestionnaire_mission.changer_etat(EtatMission.AJUSTEMENT_ELLIPSE, "Sortie du nuage")


# Continuer le suivi normal...

def executer_ajustement_ellipse(gestionnaire_mission):
    """Ajuste les paramètres de l'ellipse"""
    generateur_traj.ajuster_parametres()
    gestionnaire_mission.changer_etat(EtatMission.SUIVI_FRONTIERE, "Paramètres ajustés")


def executer_procedure_critique(gestionnaire_mission):
    """Procédure d'urgence pour seuil critique"""
    set_mode("LOITER")
    time.sleep(2)
    gestionnaire_mission.changer_etat(EtatMission.RETOUR_BASE, "Procédure critique terminée")


def executer_batterie_faible(gestionnaire_mission):
    """Gère la batterie faible"""
    gestionnaire_mission.changer_etat(EtatMission.RETOUR_BASE, "Batterie critique")


def executer_retour_base(gestionnaire_mission):
    """Retour à la base"""
    set_mode("RTL")
    gestionnaire_mission.changer_etat(EtatMission.TERMINE, "Retour base activé")


# Lancement de la mission
if __name__ == "__main__":
    mission_principale()