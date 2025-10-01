import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pyvista as pv

from coeffs_dispersion import calc_sigmas_briggs

################################### VARIABLES STANDARDS #####################################

P = 101325  # Pression atmosphérique standard en [Pa]
R = 287     # Constante spécifique de l'air en [J/kg.K]
c_p = 1.004 # capacité calorifique de l'air [kJ/kg.K] 
classe_stabilite = ['A', 'B', 'C', 'D', 'E', 'F']
choix_zone = ['urbaine', 'rurale']
k = 0.4     # Constante de Karman

################################## VARIABLES MODIFIABLES ######################################

Q_t = 100   # Puissance thermique globale dégagée par le foyer en [kW]
S = 20      # Surface en feu en [m²]
d = math.sqrt((4 * S) / math.pi)  # Diamètre de la source en [m]

# Ou alors, si on connaît le diamètre du feu :
# d = 10
# S = math.pi * (d/2)**2

T_amb = 20 + 273  # Température ambiante en [K] (20°C)
uref = 5          # Vitesse du vent à zref [m/s]
zref = 10         # Hauteur de référence [m]
h = 0             # Hauteur de la source du feu en [m]

# Rugosité pour une forêt ou un milieu urbain
r = 1             # en [m]

# Sélection de la classe de stabilité (1 pour 'A', ..., 6 pour 'F')
stab = 6
classe_selectionnee = classe_stabilite[stab - 1]  # (Python indexe à partir de 0)

# Sélection de la zone (1 pour 'urbaine', 2 pour 'rurale')
zone = 1
zone_selectionnee = choix_zone[zone - 1]


############################### CALCULS DES PARAMETRES SPECIFIQUES AUX INCENDIES ####################################

Q_c = (2/3) * Q_t  # Puissance convectée de l'incendie en [kW]
z_0 = 0            # Origine virtuelle en [m]
z_1 = z_0 + 0.166 * Q_c**(2/5) # Altitude correspondant à la hauteur moyenne des flammes [m]


# Fonction permettant de calculer le débit de fumées en [kg/s]
def debit_fumees(z, z_1, Q_c, z_0):
    if z >= z_1:
        m_dot = 0.071 * Q_c**(1/3) * (z - z_0)**(5/3) * (1 + 0.026 * Q_c**(2/3) * (z - z_0)**(-5/3))
    else:
        m_dot = (0.0054 * Q_c * z) / (0.166 * Q_c**(2/5) + z_0)
    return m_dot

# Calcul du débit massique à z_1
m_dotz1 = 0.071 * Q_c**(1/3) * (z_1 - z_0)**(5/3) * (1 + 0.026 * Q_c**(2/3) * (z_1 - z_0)**(-5/3))


T_moy = T_amb + Q_c / (m_dotz1 * c_p)  # Température moyenne au coeur du panache en [K]
T_moy_C = T_moy - 273  # Température en Celsius

T0_emis = T_amb + 2 * (T_moy - T_amb) # Température émise à la source [K]
r_smoke = 0.12 * (T0_emis / T_amb)**(1/2) * (z_1 - z_0) # Rayon du panache [m]

rho = P / (R * T_moy)  # Masse volumique de l'air à T_moy en [kg/m³]
u_s = m_dotz1 / (rho * 2 * math.pi * r_smoke**2)  # Vitesse ascensionnelle en [m/s]

# Affichage des résultats (optionnel)
print("Classe de stabilité sélectionnée:", classe_selectionnee)
print("Zone sélectionnée:", zone_selectionnee)
print("Température moyenne en Celsius:", T_moy_C)
print("Vitesse ascensionnelle:", u_s)

# Valeur du coefficient p présent dans la fonction puissance
p_coeff = np.array([
    0.141,  # Classe A
    0.176,  # Classe B
    0.174,  # Classe C
    0.209,  # Classe D
    0.277,  # Classe E
    0.414   # Classe F
])

p = p_coeff[stab - 1]  # Choix du coefficient p en fonction de la stabilité

# Calcul du profil de vitesse du vent
def calcul_vitesse(zref, uref, p, z):
    return uref * (z / zref)**p

################################################## CALCUL DE LA CONCENTRATION EN 3D  ####################################################

# Domaine spatial (en mètres)
x = np.linspace(0, 1000, 500)   # Direction du vent
y = np.linspace(-100, 100, 200)  # Direction latérale
z = np.linspace(0, 100, 200)     # Hauteur

# Maillage 3D - Utilisation correcte de meshgrid
X, Y, Z = np.meshgrid(x, y, z, indexing='ij')  # Dimensions (1000, 100, 200)

# Initialiser la concentration avec dimensions correspondantes
C = np.zeros((len(x), len(y), len(z)))  # Dimensions (1000, 100, 200)

epsilon = 1e-10  # Valeur très petite pour éviter les divisions par zéro

for i in range(len(x)):
    for j in range(len(y)):
        for k in range(len(z)):
            # Calcul des coefficients de dispersion
            sigma_y_val, sigma_z_val = calc_sigmas_briggs(
                classe_selectionnee, 
                zone_selectionnee, 
                X[i, j, k]  # X[i,j,k] est la valeur scalaire pour ce point
            )

            # Gestion des divisions par zéro
            sigma_y_val_safe = max(sigma_y_val, epsilon)
            sigma_z_val_safe = max(sigma_z_val, epsilon)
            
            # Calculer le débit
            m_dot = debit_fumees(Z[i, j, k], z_1, Q_c, z_0)
            
            # Calculer la vitesse du vent
            u = calcul_vitesse(zref, uref, p, Z[i, j, k])
           
            term_y = np.exp(-(Y[i, j, k]**2) / (2 * sigma_y_val_safe**2 + epsilon))
            
            diff_z = Z[i, j, k] - (z_1 + h)
            sum_z = Z[i, j, k] + (z_1 + h)
            
            term_z = (np.exp(-(diff_z**2) / (2 * sigma_z_val_safe**2 + epsilon)) +
                      np.exp(-(sum_z**2) / (2 * sigma_z_val_safe**2 + epsilon)))
            
            # Gestion du dénominateur
            denominator = 2 * np.pi * u * sigma_y_val_safe * sigma_z_val_safe
            denominator_safe = max(denominator, epsilon)
            
            C[i, j, k] = (m_dot / denominator_safe) * term_y * term_z


############################################# ENREGISTREMENT TABLEAU ############################################

# Enregistrement du tableau des concentrations
np.save('concentration.npy', C)

# Enregistrement du vecteur d'altitudes
np.save("altitudes.npy", z)

print("Tableau des valeurs de concentrations enregistré !")

################################################## FIGURES  ####################################################

# -------- Première figure : Coupe en Z = zslice --------
zslice = 7
idx_z = np.argmin(np.abs(z - zslice))

# Extraire la tranche 2D et créer un maillage 2D
C_slice_z = C[:, :, idx_z]  # Dimensions (1000, 100)
X_slice_z, Y_slice_z = np.meshgrid(x, y, indexing='ij')  # (1000, 100)

plt.figure(1)
plt.pcolormesh(X_slice_z, Y_slice_z, C_slice_z, shading='auto')
plt.colorbar()
plt.title(f'Concentration à Z={zslice}m')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.show()


# -------- Deuxième figure : Coupe en Y = yslice --------
yslice = 2
idx_y = np.argmin(np.abs(y - yslice))

# Extraire la tranche 2D et créer un maillage 2D
C_slice_y = C[:, idx_y, :]  # Dimensions (1000, 200)
X_slice_y, Z_slice_y = np.meshgrid(x, z, indexing='ij')  # (1000, 200)

plt.figure(2)
plt.pcolormesh(X_slice_y, Z_slice_y, C_slice_y, shading='auto')
plt.colorbar()
plt.title(f'Concentration à Y={yslice}m')
plt.xlabel('X (m)')
plt.ylabel('Z (m)')
plt.show()

# -------- Troisième figure : Surface isovolume à C = threshold --------

grid = pv.StructuredGrid(X, Y, Z)
grid['C'] = C.flatten(order='F')  # PyVista utilise l’ordre Fortran

# Définir le seuil
threshold = 0.0015

# Extraire l'isovolume (surface à valeur seuil)
contour = grid.contour(isosurfaces=[threshold], scalars='C')


# Affichage avec configuration complète des axes
plotter = pv.Plotter()
plotter.add_mesh(contour, color='gray', show_edges=False)

# Affiche les axes 3D avec graduations et étiquettes personnalisées
plotter.show_bounds(
    grid='back',           # Où afficher les axes (arrière, front, all)
    location='outer',      # Emplacement autour du volume
    all_edges=True,        # Montre toutes les arêtes
    xlabel='X (m)',
    ylabel='Y (m)',
    zlabel='Z (m)',
    font_size=14
)

# Caméra et titre
plotter.view_vector([-1, 1, 0.5])
plotter.add_title(f'Isosurface de concentration à {threshold} kg/m³', font_size=14)

# Affichage
plotter.show()