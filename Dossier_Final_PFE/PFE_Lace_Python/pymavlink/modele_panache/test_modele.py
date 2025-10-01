import numpy as np

def charger_concentration(fichier):
    """Charge le fichier numpy de concentration 3D et extrait une coupe 2D à l'altitude centrale."""
    donnees_3d = np.load(fichier)
    index_altitude = donnees_3d.shape[2] // 2
    coupe_2d = donnees_3d[:, :, index_altitude]
    return coupe_2d

def ecart_max_connectivite4(array_2d):
    """Trouve la plus grande différence entre deux points adjacents (droite, gauche, devant, derrière) dans un tableau 2D."""
    max_ecart = 0
    pos1 = pos2 = (0, 0)
    rows, cols = array_2d.shape
    for i in range(rows):
        for j in range(cols):
            for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:  # devant, derrière, gauche, droite
                ni, nj = i+di, j+dj
                if 0 <= ni < rows and 0 <= nj < cols:
                    ecart = abs(array_2d[i, j] - array_2d[ni, nj])
                    if ecart > max_ecart:
                        max_ecart = ecart
                        pos1 = (i, j)
                        pos2 = (ni, nj)
    return max_ecart, pos1, pos2

if __name__ == "__main__":
    fichier_concentration = "concentration.npy"
    coupe_2d = charger_concentration(fichier_concentration)
    max_ecart, pos1, pos2 = ecart_max_connectivite4(coupe_2d)
    print(f"Plus grand écart entre deux points adjacents (droite, gauche, devant, derrière) : {max_ecart} entre {pos1} et {pos2}")