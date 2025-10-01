from pymavlink import mavutil
import time

# Connexion au drone
mav_connection = mavutil.mavlink_connection('/dev/serial/by-id/usb-ArduPilot_Pixhawk4_390037000C50564241343220-if02')

# Attente du heartbeat pour synchronisation
mav_connection.wait_heartbeat()
print(f"Now connected to SYSID {mav_connection.target_system}")

# Fonction pour vérifier si le mode est correctement changé
def wait_for_mode(mode_id, timeout=10):
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True)
        if msg and msg.custom_mode == mode_id:
            print(f"Vehicle is now in {mode_id} mode.")
            return True
        time.sleep(1)
    print(f"Failed to switch to {mode_id} mode.")
    return False

# Passage en mode GUIDED
print("Changing mode to GUIDED...")
mode_id = mav_connection.mode_mapping()['GUIDED']

# Envoi de la commande de changement de mode
mav_connection.mav.set_mode_send(
    mav_connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

# Vérification du mode GUIDED
if wait_for_mode(mode_id):
    # Armer le drone
    print("Arming drone...")
    mav_connection.mav.command_long_send(
        mav_connection.target_system,
        mav_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # 1 pour armer, 0 pour désarmer
        0, 0, 0, 0, 0, 0
    )

    # Attente que le drone soit armé
    armed = False
    while not armed:
        msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
        if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            armed = True
            print("Drone armed.")
        time.sleep(1)

    # Tenter de décoller
    print("Initiating takeoff...")
    mav_connection.mav.command_long_send(
        mav_connection.target_system,
        mav_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, 10  # Altitude de décollage = 10 mètres
    )

    print("Takeoff command sent.")

else:
    print("Failed to pass the drone into GUIDED mode. Aborting...")
