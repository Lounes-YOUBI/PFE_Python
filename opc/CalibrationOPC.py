#!/usr/bin/env python3
"""
Logger PM2.5 continu pour capteur OPC-N3 avec sauvegarde CSV horodatée
Basé sur le code fonctionnel fourni avec opcng
Conçu pour fonctionner sur Raspberry Pi avec arrêt sécurisé
"""

from time import sleep
import spidev
import math
import opcng as opc
import csv
import signal
import sys
from datetime import datetime
import os
import atexit

class PM25Logger:
    def __init__(self, csv_filename='pm25_data.csv', read_interval=2):
        self.csv_filename = csv_filename
        self.read_interval = read_interval
        self.running = True
        self.spi = None
        self.dev = None
        self.measurement_count = 0
        
        # Configuration des gestionnaires pour arrêt propre
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        atexit.register(self.cleanup)
        
        # Créer le fichier CSV avec en-têtes s'il n'existe pas
        self.init_csv_file()
    
    def signal_handler(self, signum, frame):
        """Gestionnaire pour arrêt propre du programme"""
        print(f"\nSignal {signum} reçu. Arrêt en cours...")
        self.running = False
    
    def init_csv_file(self):
        """Initialise le fichier CSV avec les en-têtes si nécessaire"""
        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['timestamp', 'pm25_value', 'status', 'measurement_id'])
            print(f"Fichier CSV créé: {self.csv_filename}")
        else:
            print(f"Fichier CSV existant utilisé: {self.csv_filename}")
    
    def init_sensor(self):
        """Initialise le capteur OPC-N3"""
        try:
            # Configuration SPI identique à votre code
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)
            self.spi.mode = 1
            self.spi.max_speed_hz = 500000
            self.spi.lsbfirst = False
            
            # Détection du capteur
            self.dev = opc.detect(self.spi)
            
            print(f'Informations capteur: {self.dev.info()}')
            print(f'Numéro de série: {self.dev.serial()}')
            
            # Tenter d'afficher la version firmware si disponible
            try:
                print(f'Version firmware: {self.dev.firmware()}')
            except AttributeError:
                print('Version firmware: Non disponible pour ce modèle')
            
            # Allumer le ventilateur et le laser
            self.dev.on()
            print("Capteur allumé - Ventilateur et laser activés")
            
            # Période de stabilisation
            print("Stabilisation du capteur en cours (30 secondes)...")
            sleep(30)
            
            return True
            
        except Exception as e:
            print(f"Erreur lors de l'initialisation du capteur: {e}")
            return False
    
    def read_pm25_value(self):
        """
        Lit une valeur PM2.5 depuis le capteur OPC-N3
        Retourne la valeur ou None en cas d'erreur
        """
        try:
            if self.dev:
                pm_data = self.dev.pm()
                pm25_value = pm_data["PM2.5"]
                
                # Vérification de validité identique à votre code
                if isinstance(pm25_value, (int, float)) and not math.isnan(pm25_value):
                    return pm25_value
                else:
                    print("Valeur PM2.5 invalide ignorée.")
                    return None
            
            return None
            
        except Exception as e:
            print(f"Erreur lors de la lecture PM2.5: {e}")
            return None
    
    def save_to_csv(self, pm25_value, status="OK"):
        """
        Sauvegarde une mesure dans le fichier CSV avec horodatage
        Utilise le mode 'append' et flush pour éviter la perte de données
        """
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.measurement_count += 1
            
            # Ouvrir en mode append pour éviter la perte de données
            with open(self.csv_filename, 'a', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([timestamp, pm25_value, status, self.measurement_count])
                
                # Forcer l'écriture immédiate sur le disque
                csvfile.flush()
                os.fsync(csvfile.fileno())
            
            return True
            
        except Exception as e:
            print(f"Erreur lors de la sauvegarde CSV: {e}")
            return False
    
    def run(self):
        """
        Boucle principale de lecture et sauvegarde continue
        """
        print("=" * 50)
        print("Démarrage du logger PM2.5 OPC-N3")
        print(f"Fichier de sortie: {self.csv_filename}")
        print(f"Intervalle de lecture: {self.read_interval} secondes")
        print("Appuyez sur Ctrl+C pour arrêter proprement")
        print("=" * 50)
        
        # Initialisation du capteur
        if not self.init_sensor():
            print("Impossible d'initialiser le capteur. Arrêt du programme.")
            return
        
        try:
            print("Début de l'acquisition des données...")
            
            while self.running:
                # Lire la valeur PM2.5
                pm25_value = self.read_pm25_value()
                
                if pm25_value is not None:
                    # Sauvegarder la mesure
                    if self.save_to_csv(pm25_value):
                        print(f"Mesure #{self.measurement_count}: PM2.5 = {pm25_value:.2f} μg/m³ - {datetime.now().strftime('%H:%M:%S')}")
                    else:
                        print(f"Erreur de sauvegarde pour la valeur: {pm25_value}")
                        # Sauvegarder avec status d'erreur
                        self.save_to_csv(pm25_value, "SAVE_ERROR")
                else:
                    # Données invalides
                    print(f"Données invalides - {datetime.now().strftime('%H:%M:%S')}")
                    self.save_to_csv("INVALID", "INVALID_DATA")
                
                # Attendre avant la prochaine lecture
                sleep(self.read_interval)
                
        except KeyboardInterrupt:
            print("\nInterruption clavier détectée...")
        except Exception as e:
            print(f"Erreur inattendue dans la boucle principale: {e}")
            # Sauvegarder l'erreur
            self.save_to_csv("ERROR", f"EXCEPTION: {str(e)}")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Nettoyage des ressources"""
        print("\nNettoyage en cours...")
        
        try:
            # Éteindre le capteur (ventilateur et laser)
            if self.dev:
                self.dev.off()
                print("Capteur éteint - Ventilateur et laser désactivés")
        except Exception as e:
            print(f"Erreur lors de l'extinction du capteur: {e}")
        
        try:
            # Fermer la connexion SPI
            if self.spi:
                self.spi.close()
                print("Connexion SPI fermée")
        except Exception as e:
            print(f"Erreur lors de la fermeture SPI: {e}")
        
        print(f"Logger arrêté. Total: {self.measurement_count} mesures dans {self.csv_filename}")
        print("Toutes les données ont été sauvegardées de manière sécurisée.")

def main():
    """Fonction principale"""
    # Configuration
    CSV_FILENAME = 'pm25_data.csv'  # Nom du fichier de sortie
    READ_INTERVAL = 2  # Intervalle entre les lectures en secondes
    
    # Créer et lancer le logger
    logger = PM25Logger(csv_filename=CSV_FILENAME, read_interval=READ_INTERVAL)
    
    try:
        logger.run()
    except Exception as e:
        print(f"Erreur fatale: {e}")
        logger.cleanup()

if __name__ == "__main__":
    main()
