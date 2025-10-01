from pymavlink import mavutil
import time

# Liste des modes ArduCopter disponibles
ARDUCOPTER_MODES = {
    "STABILIZE": 0,
    "ACRO": 1,
    "ALT_HOLD": 2,
    "AUTO": 3,
    "GUIDED": 4,
    "LOITER": 5,
    "RTL": 6,
    "LAND": 9,
    "POSHOLD": 16,
    "BRAKE": 17,
    "THROW": 18,
    "AVOID_ADSB": 19,
    "GUIDED_NOGPS": 20,
    "SMART_RTL": 21,
    "FLOWHOLD": 22,
    "FOLLOW": 23,
    "ZIGZAG": 24,
    "SYSTEMID": 25,
    "AUTOTUNE": 26,
    "FLIP": 27,
    "AUTOROTATE": 28
}

def list_modes():
    print("\nModes disponibles :")
    for i, (mode, num) in enumerate(ARDUCOPTER_MODES.items()):
        print(f"{i+1}. {mode} ({num})")
    print()

def get_mode_selection():
    while True:
        try:
            selection = int(input("Sélectionne un mode par numéro : ")) - 1
            mode_list = list(ARDUCOPTER_MODES.items())
            if 0 <= selection < len(mode_list):
                return mode_list[selection]
            else:
                print("Numéro invalide. Essaie encore.")
        except ValueError:
            print("Entrée non valide. Entre un chiffre.")

def set_mode(master, mode_str, mode_id):
    print(f"\nTentative de passage en mode {mode_str} ({mode_id})...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    # Attente de confirmation du changement
    timeout = 10
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        current_mode = msg.custom_mode
        if current_mode == mode_id:
            print(f"✅ Mode changé avec succès en {mode_str}")
            return
        print("⏳ En attente du changement de mode...")
        time.sleep(1)

    print(f"❌ Échec du changement de mode en {mode_str}.")

def main():
    print("Connexion au drone...")
    master = mavutil.mavlink_connection('/dev/serial/by-id/usb-ArduPilot_Pixhawk4_390037000C50564241343220-if02')  # adapte si besoin
    master.wait_heartbeat()
    print(f"Connecté au système {master.target_system}\n")

    while True:
        list_modes()
        mode_str, mode_id = get_mode_selection()
        set_mode(master, mode_str, mode_id)

        again = input("\nSouhaites-tu tester un autre mode ? (o/n) : ").lower()
        if again != 'o':
            print("Fin du test.")
            break

if __name__ == "__main__":
    main()
