from pymavlink import mavutil
import time

# Connexion au drone
mav_connection = mavutil.mavlink_connection('/dev/serial/by-id/usb-ArduPilot_Pixhawk4_390037000C50564241343220-if02')
#mav_connection = mavutil.mavlink_connection('/dev/serial/by-id/usb-ArduPilot_Pixhawk4_390037000C50564241343220-if02',args.sysid)


# Attendre que le drone envoie des messages de type HEARTBEAT
mav_connection.wait_heartbeat()
print(f"Now connected to SYSID {mav_connection.target_system}")

# Passer en mode GUIDED
mav_connection.mav.command_long_send(
    mav_connection.target_system,
    mav_connection.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,  # confirmation
    4,  # mode GUIDED
    0, 0, 0, 0, 0, 0
)

# Vérifier que le mode est bien changé
time.sleep(1)  # Laisser le temps au mode de se changer
print("Vehicle in GUIDED mode")

# Armer le drone
mav_connection.mav.command_long_send(
    mav_connection.target_system,
    mav_connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,  # confirmation
    1,  # armement
    0, 0, 0, 0, 0, 0
)

# Attendre que le drone soit armé
while True:
    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Drone armed")
        break
    time.sleep(1)
