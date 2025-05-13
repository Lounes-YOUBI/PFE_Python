from time import sleep
import spidev
import math
import opcng as opc
import statistics

pm25_values = []

spi = spidev.SpiDev()
spi.open(0, 0)
spi.mode = 1
spi.max_speed_hz = 500000
spi.lsbfirst = False

dev = opc.detect(spi)

print(f'device information: {dev.info()}')
print(f'serial: {dev.serial()}')
print(f'firmware version: {dev.serial()}')

# power on fan and laser
dev.on()


for i in range(100):
    sleep(1)
    value = dev.pm()["PM2.5"]
    print(value)
    # Vérifie que la valeur est bien un nombre et pas NaN
    if isinstance(value, (int, float)) and not math.isnan(value):
        pm25_values.append(value)
    else:
        print("Valeur PM2.5 invalide ignorée.")

# power off fan and laser
dev.off()

# Vérifie qu’il reste des données valides avant de calculer les statistiques
if pm25_values:
    mean_pm25 = statistics.mean(pm25_values)
    stdev_pm25 = statistics.stdev(pm25_values) if len(pm25_values) > 1 else 0
    min_pm25 = min(pm25_values)
    max_pm25 = max(pm25_values)

    print(f"Moyenne PM2.5 : {mean_pm25:.2f}")
    print(f"Écart type PM2.5 : {stdev_pm25:.2f}")
    print(f"Minimum PM2.5 : {min_pm25:.2f}")
    print(f"Maximum PM2.5 : {max_pm25:.2f}")
else:
    print("Aucune donnée valide pour le calcul des statistiques.")


