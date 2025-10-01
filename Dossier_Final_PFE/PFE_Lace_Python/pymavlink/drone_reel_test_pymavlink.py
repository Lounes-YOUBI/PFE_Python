# takeoff.py
import argparse
from pymavlink import mavutil
import time
from utilities.connect_to_sysid import connect_to_sysid
from utilities.wait_for_position_aiding import wait_until_position_aiding
from utilities.get_autopilot_info import get_autopilot_info


def takeoff(mav_connection, takeoff_altitude: float, tgt_sys_id: int = 1, tgt_comp_id=1):

    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(
        mav_connection.target_system,
        mav_connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        1000000,  # 1Hz
        0, 0, 0, 0, 0
    )

    #wait_until_position_aiding(mav_connection)

    autopilot_info = get_autopilot_info(mav_connection, tgt_sys_id)
    
    print("Autopilote utilisé : ", autopilot_info["autopilot"])


    if autopilot_info["autopilot"] == "ardupilotmega":
        print("Connected to ArduPilot autopilot")
        mode_id = mav_connection.mode_mapping()["GUIDED"]
        #takeoff_params = [0, 0, 0, 0, 0, 0, takeoff_altitude]
        takeoff_params = [0, 5, 0, 0, 0, 0,10]  # Décollage à une vitesse verticale de 5 m/s, atteignant 10 m d'altitude


    elif autopilot_info["autopilot"] == "px4":
        print("Connected to PX4 autopilot")
        print(mav_connection.mode_mapping())
        mode_id = mav_connection.mode_mapping()["TAKEOFF"][1]
        print(mode_id)
        msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        starting_alt = msg.alt / 1000
        takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + takeoff_altitude]

    else:
        raise ValueError("Autopilot not supported")


    # Change mode to guided (Ardupilot) or takeoff (PX4)
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

    # Arm the UAS
    #set_mode(mav_connection, 'MANUAL')
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Arm ACK:  {arm_msg}")

    # Command Takeoff
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4], takeoff_params[5], takeoff_params[6])

    takeoff_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Takeoff ACK:  {takeoff_msg}")

    return takeoff_msg.result

def check_preflight_status(mav_connection):
    mav_connection.mav.command_long_send(
        mav_connection.target_system,
        mav_connection.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0,  # Confirmation
        1,  # Command Parameter 1 (Gyro Calibration)
        1,  # Command Parameter 2 (Mag Calibration)
        1,  # Command Parameter 3 (Accel Calibration)
        0,  # Remaining params unused
        0,
        0,
        0
    )
    print("Sent preflight calibration request")
# Fonction pour changer de mode
def set_mode(mav_connection,mode_name):
	""" Change le mode de vol du drone """
	mode_id = mav_connection.mode_mapping().get(mode_name)
	if mode_id is None:
		print(f"Mode {mode_name} inconnu !")
		return

	# Envoi du message MAVLink pour changer de mode
	mav_connection.mav.set_mode_send(
		mav_connection.target_system,
		mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
		mode_id
	)
	print(f"Demande de passage en mode {mode_name} envoyée.")

	# Attendre la confirmation
	while True:
		msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True)
		if msg.custom_mode == mode_id:
			print(f"Mode {mode_name} activé !")
			break


def get_mode(mav_connection):
	""" Récupère et affiche le mode de vol actuel """
	msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True)
	if not msg:
		print("Impossible de récupérer le mode.")
		return None

	mode_id = msg.custom_mode  # ID du mode actuel
	mode_name = None

	# Mapper l'ID avec un nom de mode
	for name, mode in mav_connection.mode_mapping().items():
		if mode == mode_id:
			mode_name = name
			break

	if mode_name:
		print(f"Mode actuel : {mode_name}")
	else:
		print(f"Mode inconnu (ID: {mode_id})")

	return mode_name

def check_health(master):
    """Vérification de la santé des capteurs avant l'armement"""
    print("Vérification de la santé du drone...")

    # Envoyer une requête pour l'état de santé
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        0, 0, 0, 0, 0, 0
    )

    # Attendre et lire le message SYS_STATUS
    while True:
        msg = master.recv_match(type="SYS_STATUS", blocking=True)
        if msg:
            sensors_health = msg.onboard_control_sensors_health
            battery_voltage = msg.voltage_battery / 1000.0
            print(f"Batterie : {battery_voltage:.2f}V")
            
            # Vérifier les conditions
            if sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO and \
               sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL and \
               sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
                if battery_voltage > 10.5:  # Exemple : tension minimale de 10.5V
                    print("Tous les systèmes sont sains!")
                    return True
                else:
                    print("Batterie faible! Rechargez avant de continuer.")
            else:
                print("Capteurs défectueux ou non calibrés.")
        time.sleep(1)
        
def main():
    parser = argparse.ArgumentParser(description="A simple script to command a UAV to takeoff.")
    parser.add_argument("--altitude", type=int, help="Altitude for the UAV to reach upon takeoff.", default=10)
    parser.add_argument("--sysid", type=int, help="System ID of the UAV to command.", default=1)


    args = parser.parse_args()
    mav_connection = mavutil.mavlink_connection('/dev/serial/by-id/usb-ArduPilot_Pixhawk4_390037000C50564241343220-if02')
    #mav_connection = connect_to_sysid('/dev/ttyACM0', args.sysid)
    #mav_connection = connect_to_sysid('/dev/serial/by-id/usb-ArduPilot_Pixhawk4_390037000C50564241343220-if02',args.sysid)

    #check_preflight_status(mav_connection)
        
    #check_health(mav_connection)
        
    takeoff(mav_connection, args.altitude)

    print(get_mode(mav_connection))
    time.sleep(3)
    set_mode(mav_connection, 'AUTO')
    time.sleep(3)
    print(get_mode(mav_connection))
    time.sleep(3)
if __name__ == "__main__":
    main()