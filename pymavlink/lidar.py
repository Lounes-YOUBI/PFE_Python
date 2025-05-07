from pymavlink import mavutil

# Connexion au drone
master = mavutil.mavlink_connection('/dev/ttyACM0') # adapter selon votre config
master.wait_heartbeat()
print("Connecté au système")

while True:
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True)
    if msg:
        print(f"Distance: {msg.current_distance} cm | Min: {msg.min_distance} | Max: {msg.max_distance} | Type: {msg.type}")
