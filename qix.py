from fltk import *
import matplotlib.path as mpath
import random
import numpy as np
import time
import math


# Constantes pour le jeu
LARGEUR_FENETRE = 900
HAUTEUR_FENETRE = 700
LARGEUR_MAP = 700
HAUTEUR_MAP = 500
X_MAP = (LARGEUR_FENETRE - LARGEUR_MAP) // 2
Y_MAP = (HAUTEUR_FENETRE - HAUTEUR_MAP) // 2
RAYON_JOUEUR = 10
VITESSE_JOUEUR = 3
VITESSE_QIX = 3
VITESSE_SPARX = 3
RAYON_QIX = 50
TAILLE_SPARX = 15
POURCENTAGE_COUVERTURE_OBJECTIF = 75
NOMBRE_VIES = 4
NOMBRE_VIES2 = 999
SEUIL_BORD = 10
FACTEUR_POINTS = 15
directions_sparx1 = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # droite, bas, gauche, haut
directions_sparx2 = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # gauche, bas, droite, haut
qix_directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
qix_direction = random.choice(qix_directions)
QIX_MOVE_PROBABILITY = 0.25  # Probabilité de changer de direction à chaque itération (25%)
QIX_MOVE_DISTANCE = 5
NOMBRE_POMMES = 5
RAYON_POMME = 5
positions_pommes = [(random.randint(X_MAP, X_MAP + LARGEUR_MAP), random.randint(Y_MAP, Y_MAP + HAUTEUR_MAP)) for _ in range(NOMBRE_POMMES)]
temps_invincible = 0
temps_invincible2 = 0
VITESSE_BOOST_JOUEUR = 7


# Variables globales
x_joueur, y_joueur = X_MAP + LARGEUR_MAP // 2, Y_MAP + HAUTEUR_MAP
direction_joueur = (0, 0)
x_joueur2, y_joueur2 = X_MAP + LARGEUR_MAP // 2, Y_MAP + HAUTEUR_MAP // 2
direction_joueur2 = (0, 0)
x_sparx1, y_sparx1, direction_sparx1 = X_MAP, Y_MAP, (1, 0)  # Commence dans le coin supérieur gauche, se déplace vers la droite
x_sparx2, y_sparx2, direction_sparx2 = X_MAP + LARGEUR_MAP, Y_MAP, (-1, 0)  # Commence dans le coin supérieur droit, se déplace vers la gauche
sur_la_bordure = True
dessin_en_cours = False
victoire = False
boost_actif = False
boost_actif2 = False
mode_deux_joueurs = False
points_chemin = []
zones_coloriees = []
zones_coloriees2 = []  
indice_direction_sparx1 = 0

indice_direction_sparx2 = 0
x_qix, y_qix = X_MAP + LARGEUR_MAP // 2, Y_MAP + HAUTEUR_MAP // 2

qix_directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
qix_direction = random.choice(qix_directions)
x_titre = 20
y_titre = 20

# Fonctions pour le jeu
def dessiner_map():
    rectangle(0, 0, 900, 700, couleur="black", remplissage="black")
    rectangle(X_MAP, Y_MAP, X_MAP + LARGEUR_MAP, Y_MAP + HAUTEUR_MAP, couleur="white", remplissage="black")


def dessiner_joueur():
    cercle(x_joueur, y_joueur, RAYON_JOUEUR, remplissage='red')
    
    
def dessiner_joueur2():
    cercle(x_joueur2, y_joueur2, RAYON_JOUEUR, remplissage='blue')
    
    
def dessiner_losange(x, y, taille):
    points = [
        (x, y - taille),  # Haut
        (x + taille, y),  # Droite
        (x, y + taille),  # Bas
        (x - taille, y)   # Gauche
    ]
    polygone(points, remplissage='green')
    
    
def dessiner_ennemis():
    dessiner_losange(x_sparx1, y_sparx1, TAILLE_SPARX)
    dessiner_losange(x_sparx2, y_sparx2, TAILLE_SPARX)
    cercle(x_qix, y_qix, RAYON_QIX, couleur="pink",remplissage='pink')
    

def dessiner_pommes():
    for x, y in positions_pommes:
        cercle(x, y, RAYON_POMME, remplissage='yellow')

def avale_pomme(x_joueur, y_joueur):
    global temps_invincible
    for pomme in positions_pommes:
        if ((x_joueur - pomme[0]) ** 2 + (y_joueur - pomme[1]) ** 2) ** 0.5 < RAYON_JOUEUR + RAYON_POMME:
            positions_pommes.remove(pomme)
            temps_invincible = time.time() + 5  # 5 secondes d'invincibilité
            break
    

class Obstacle:
    def __init__(self, x, y, taille, nombre_cotes):
        self.x = x
        self.y = y
        self.taille = taille
        self.nombre_cotes = nombre_cotes
        self.points = self.generer_points()

    def generer_points(self):
        points = []
        angle = 2 * math.pi / self.nombre_cotes
        for i in range(self.nombre_cotes):
            x_point = self.x + self.taille * math.cos(i * angle)
            y_point = self.y + self.taille * math.sin(i * angle)
            points.append((x_point, y_point))
        return points

def creer_obstacles(n, largeur_map, hauteur_map):
    obstacles = []
    for _ in range(n):
        x = random.randint(X_MAP, X_MAP + largeur_map)
        y = random.randint(Y_MAP, Y_MAP + hauteur_map)
        taille = random.randint(20, 50)  # Taille aléatoire
        nombre_cotes = random.randint(3, 6)  # Nombre de côtés aléatoire
        obstacles.append(Obstacle(x, y, taille, nombre_cotes))
    return obstacles


def dessiner_obstacles(obstacles):
    for obstacle in obstacles:
        polygone(obstacle.points, remplissage='gray')
        
        
def collision_avec_obstacle(x_joueur, y_joueur, obstacles):
    for obstacle in obstacles:
        if point_dans_polygone(x_joueur, y_joueur, obstacle.points):
            return True
    return False
        
        
def changer_direction_sparx(x, y, direction):
    # Vérifier les bords de la map
    if x <= X_MAP or x >= X_MAP + LARGEUR_MAP:
        return (0, direction[1] * -1)  # Changement de direction en Y
    elif y <= Y_MAP or y >= Y_MAP + HAUTEUR_MAP:
        return (direction[0] * -1, 0)  # Changement de direction en X

    # Vérifier les zones coloriées
    #if est_sur_un_bord_de_zone_coloriee(x, y, SEUIL_BORD):
        # Logique pour changer la direction sur les zones coloriées
        # Cette partie doit être implémentée en fonction de votre jeu

    #return direction  # Pas de changement de direction
    
    
def est_sur_un_bord(x, y):
    return x in [X_MAP, X_MAP + LARGEUR_MAP] or y in [Y_MAP, Y_MAP + HAUTEUR_MAP]


def est_dans_map(x, y):
    return X_MAP <= x <= X_MAP + LARGEUR_MAP and Y_MAP <= y <= Y_MAP + HAUTEUR_MAP


def est_sur_le_meme_bord(point1, point2):
    # Vérifie si les deux points sont sur le même bord horizontal ou vertical
    bord_horizontal = (point1[1] in [Y_MAP, Y_MAP + HAUTEUR_MAP] and
                       point2[1] in [Y_MAP, Y_MAP + HAUTEUR_MAP])
    bord_vertical = (point1[0] in [X_MAP, X_MAP + LARGEUR_MAP] and
                     point2[0] in [X_MAP, X_MAP + LARGEUR_MAP])

    return bord_horizontal or bord_vertical


def deplacer_joueur(obstacles):
    global x_joueur, y_joueur, sur_la_bordure, dessin_en_cours, direction_joueur, points_chemin, boost_actif
    dx, dy = direction_joueur

    vitesse_actuelle = VITESSE_BOOST_JOUEUR if boost_actif else VITESSE_JOUEUR
    nouveau_x = x_joueur + dx * vitesse_actuelle
    nouveau_y = y_joueur + dy * vitesse_actuelle
    
    if collision_avec_obstacle(nouveau_x, nouveau_y, obstacles):
        return

    # Ajout de la vérification pour rester dans les limites de la map
    if not (X_MAP <= nouveau_x <= X_MAP + LARGEUR_MAP and Y_MAP <= nouveau_y <= Y_MAP + HAUTEUR_MAP):
        return

    if dessin_en_cours:
        # Gestion du dessin en cours
        if est_dans_map(nouveau_x, nouveau_y) and not est_dans_zone_coloriee(nouveau_x, nouveau_y):
            x_joueur = nouveau_x
            y_joueur = nouveau_y
            points_chemin.append((x_joueur, y_joueur))
            if est_sur_un_bord(nouveau_x, nouveau_y) and not est_sur_le_meme_bord(points_chemin[0], (x_joueur, y_joueur)):
                fermer_et_traiter_chemin()  # Automatiser le coloriage
    else:
        # Gestion du déplacement hors du mode dessin
        if est_sur_un_bord(nouveau_x, nouveau_y) or est_sur_un_bord_de_zone_coloriee(nouveau_x, nouveau_y, SEUIL_BORD):
            x_joueur = nouveau_x
            y_joueur = nouveau_y
            
            
def deplacer_joueur2(obstacles):
    global x_joueur2, y_joueur2, direction_joueur2, boost_actif2, sur_la_bordure, dessin_en_cours, points_chemin
    dx, dy = direction_joueur2
    
    vitesse_actuelle = VITESSE_BOOST_JOUEUR if boost_actif else VITESSE_JOUEUR
    nouveau_x2 = x_joueur2 + dx * vitesse_actuelle
    nouveau_y2 = y_joueur2 + dy * vitesse_actuelle
    
    if collision_avec_obstacle(nouveau_x2, nouveau_y2, obstacles):
        return

    # Ajout de la vérification pour rester dans les limites de la map
    if not (X_MAP <= nouveau_x2 <= X_MAP + LARGEUR_MAP and Y_MAP <= nouveau_y2 <= Y_MAP + HAUTEUR_MAP):
        return

    if dessin_en_cours:
        # Gestion du dessin en cours
        if est_dans_map(nouveau_x2, nouveau_y2) and not est_dans_zone_coloriee(nouveau_x2, nouveau_y2):
            x_joueur2 = nouveau_x2
            y_joueur2 = nouveau_y2
            points_chemin.append((x_joueur2, y_joueur2))
            if est_sur_un_bord(nouveau_x2, nouveau_y2) and not est_sur_le_meme_bord(points_chemin[0], (x_joueur2, y_joueur2)):
                fermer_et_traiter_chemin()  # Automatiser le coloriage
    else:
        # Gestion du déplacement hors du mode dessin
        if est_sur_un_bord(nouveau_x2, nouveau_y2) or est_sur_un_bord_de_zone_coloriee(nouveau_x2, nouveau_y2, SEUIL_BORD):
            x_joueur2 = nouveau_x2
            y_joueur2 = nouveau_y2
    


def deplacer_sparx(x, y, directions, indice_direction):
    dx, dy = directions[indice_direction]
    x += dx * VITESSE_SPARX
    y += dy * VITESSE_SPARX

    # Vérifier les collisions avec les bords de la map
    if x <= X_MAP or x >= X_MAP + LARGEUR_MAP:
        indice_direction = (indice_direction + 1) % len(directions)
    elif y <= Y_MAP or y >= Y_MAP + HAUTEUR_MAP:
        indice_direction = (indice_direction + 1) % len(directions)

    return x, y, indice_direction        


def deplacer_sparx1():
    global x_sparx1, y_sparx1, indice_direction_sparx1
    x_sparx1, y_sparx1, indice_direction_sparx1 = deplacer_sparx(x_sparx1, y_sparx1, directions_sparx1, indice_direction_sparx1)


def deplacer_sparx2():
    global x_sparx2, y_sparx2, indice_direction_sparx2
    x_sparx2, y_sparx2, indice_direction_sparx2 = deplacer_sparx(x_sparx2, y_sparx2, directions_sparx2, indice_direction_sparx2)
    
    
def move_qix():
    global x_qix, y_qix, qix_direction
    if random.random() < QIX_MOVE_PROBABILITY:
        direction = random.choice(["Left", "Right", "Up", "Down"])
        if direction == "Left":
            x_qix = max(X_MAP, x_qix - QIX_MOVE_DISTANCE)
        elif direction == "Right":
            x_qix = min(X_MAP + LARGEUR_MAP - RAYON_QIX, x_qix + QIX_MOVE_DISTANCE)
        elif direction == "Up":
            y_qix = max(Y_MAP, y_qix - QIX_MOVE_DISTANCE)
        elif direction == "Down":
            y_qix = min(Y_MAP + HAUTEUR_MAP - RAYON_QIX, y_qix + QIX_MOVE_DISTANCE)
           
                
def est_sur_un_bord_de_zone_coloriee(x, y, seuil_bord):
    for zone in zones_coloriees:
        path = mpath.Path(zone)
        if path.contains_point((x, y)):
            # Calculer la distance du point (x, y) à chaque segment de la zone coloriée
            distances = [distance_point_segment((x, y), (zone[i], zone[(i + 1) % len(zone)])) for i in range(len(zone))]
            if min(distances) < seuil_bord:
                return True
    return False


def distance_point_segment(p, segment):
    """ Calcule la distance entre un point p et un segment de ligne. """
    p1, p2 = segment
    p1, p2, p = np.array(p1), np.array(p2), np.array(p)
    if all(p1 == p2):
        return np.linalg.norm(p - p1)
    # Projection scalaire
    t = np.dot(p - p1, p2 - p1) / np.linalg.norm(p2 - p1)**2
    t = max(0, min(1, t))
    projection = p1 + t * (p2 - p1)
    return np.linalg.norm(p - projection)


def chemin_se_croise(nouveau_x, nouveau_y):
    if len(points_chemin) < 3:
        return False
    for i in range(1, len(points_chemin) - 1):
        if segments_se_croisent((points_chemin[i - 1], points_chemin[i]), ((x_joueur, y_joueur), (nouveau_x, nouveau_y))):
            return True
    return False


def orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        return 0  # colinéaire
    return 1 if val > 0 else 2  # horaire ou antihoraire


def segments_se_croisent(segment1, segment2):
    p1, q1 = segment1
    p2, q2 = segment2

    # Trouver les quatre orientations nécessaires pour les conditions générales et particulières
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # Condition générale
    if o1 != o2 and o3 != o4:
        return True

    # Cas particuliers
    # p1, q1 et p2 sont colinéaires et p2 se trouve sur le segment p1q1
    if o1 == 0 and est_sur_segment(p1, p2, q1):
        return True

    # p1, q1 et q2 sont colinéaires et q2 se trouve sur le segment p1q1
    if o2 == 0 and est_sur_segment(p1, q2, q1):
        return True

    # p2, q2 et p1 sont colinéaires et p1 se trouve sur le segment p2q2
    if o3 == 0 and est_sur_segment(p2, p1, q2):
        return True

    # p2, q2 et q1 sont colinéaires et q1 se trouve sur le segment p2q2
    if o4 == 0 and est_sur_segment(p2, q1, q2):
        return True

    return False  # Dans tous les autres cas, ils ne se croisent pas

def est_sur_segment(p, q, r):
    if (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
        min(p[1], r[1]) <= q[1] <= max(p[1], r[1])):
        return True
    return False


def est_dans_zone_coloriee(x, y):
    for zone in zones_coloriees:
        if point_dans_polygone(x, y, zone):
            return True
    return False


def point_dans_polygone(x, y, polygone):
    path = mpath.Path(polygone)
    return path.contains_point((x, y))


def calculer_aire_et_points(zones_coloriees):
    aire_totale_map = LARGEUR_MAP * HAUTEUR_MAP
    aire_totale_coloriee = 0

    for zone in zones_coloriees:
        aire_totale_coloriee += calculer_aire_polygone(zone)

    pourcentage_couverture = (aire_totale_coloriee / aire_totale_map) * 100
    points = pourcentage_couverture * FACTEUR_POINTS  

    return pourcentage_couverture, points

def calculer_aire_polygone(polygone):
    """Calcule l'aire d'un polygone donné."""
    n = len(polygone)
    aire = 0.0
    for i in range(n):
        j = (i + 1) % n
        aire += polygone[i][0] * polygone[j][1]
        aire -= polygone[j][0] * polygone[i][1]
    aire = abs(aire) / 2.0
    return aire


def gerer_entree_clavier(ev):
    global direction_joueur, dessin_en_cours, direction_joueur2, sur_la_bordure, points_chemin, boost_actif, boost_actif2
    touche_pressee = touche(ev)

    if touche_pressee == 'Left':
        direction_joueur = (-1, 0)
    elif touche_pressee == 'Right':
        direction_joueur = (1, 0)
    elif touche_pressee == 'Up':
        direction_joueur = (0, -1)
    elif touche_pressee == 'Down':
        direction_joueur = (0, 1)
    elif touche_pressee == 'Return':
        dessin_en_cours = not dessin_en_cours
        sur_la_bordure = not dessin_en_cours
        if dessin_en_cours:
            points_chemin.append((x_joueur, y_joueur))
        else:
            points_chemin.append((x_joueur, y_joueur))
            fermer_et_traiter_chemin()
    elif touche_pressee == 'Shift_R':
        boost_actif = not boost_actif
        
    if mode_deux_joueurs :
        if touche_pressee == 'q':
            direction_joueur2 = (-1, 0)
        elif touche_pressee == 'd':
            direction_joueur2 = (1, 0)
        elif touche_pressee == 'z':
            direction_joueur2 = (0, -1)
        elif touche_pressee == 's':
            direction_joueur2 = (0, 1)
        elif touche_pressee == 'Shift_L':
            boost_actif2 = not boost_actif2
        


def dessiner_chemin():
    if len(points_chemin) > 1:
        for i in range(1, len(points_chemin)):
            ligne(points_chemin[i - 1][0], points_chemin[i - 1][1], 
                  points_chemin[i][0], points_chemin[i][1], couleur="white")



def fermer_et_traiter_chemin():
    global points_chemin, sur_la_bordure, dessin_en_cours
    if chemin_ferme():
        colorier_zone()
    dessin_en_cours = False
    sur_la_bordure = True
    points_chemin = []
    
    # Réinitialisation des variables pour le prochain chemin
    dessin_en_cours = False
    sur_la_bordure = True
    points_chemin = []


def chemin_ferme():
    # Vérifie si le chemin forme une boucle fermée.
    # Pour le moment, cela peut être aussi simple que de vérifier si le chemin commence et se termine sur la bordure.
    # Cette logique peut être améliorée pour une vérification plus robuste.
    return len(points_chemin) > 2 and points_chemin[0] != points_chemin[-1]


def colorier_zone():
    global points_chemin, zones_coloriees

    if len(points_chemin) < 3:
        return  # Pas assez de points pour former une zone

    # Compléter le chemin en ajoutant des points le long des bords si nécessaire
    complete_path = complete_chemin(points_chemin)

    # Construire une liste de points pour le polygone
    polygone_points = [(p[0], p[1]) for p in complete_path]

    # Ajouter le polygone à la liste des zones coloriées
    zones_coloriees.append(polygone_points)


def complete_chemin(points_chemin):
    if len(points_chemin) < 2:
        return points_chemin

    start_point = points_chemin[0]
    end_point = points_chemin[-1]
    completed_path = points_chemin.copy()

    # Points des quatre coins de la map
    corners = [(X_MAP, Y_MAP), (X_MAP + LARGEUR_MAP, Y_MAP), 
               (X_MAP + LARGEUR_MAP, Y_MAP + HAUTEUR_MAP), (X_MAP, Y_MAP + HAUTEUR_MAP)]

    # Trouver le point de coin le plus proche du point de fin
    nearest_corner = min(corners, key=lambda p: distance_between_points(p, end_point))

    # Ajouter le point de coin le plus proche au chemin
    completed_path.append(nearest_corner)

    # Ajouter le point de départ si nécessaire pour fermer le chemin
    if nearest_corner != start_point:
        # Identifier le coin qui est le plus proche du point de départ
        start_nearest_corner = min(corners, key=lambda p: distance_between_points(p, start_point))
        completed_path.append(start_nearest_corner)

    return completed_path

def distance_between_points(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


def dessiner_zones_coloriees():
    for polygone_points in zones_coloriees:
        polygone(polygone_points, couleur="white", remplissage='orange', epaisseur=2.6)
        

def init_fenetre(hauteur, largeur):
    cree_fenetre(hauteur, largeur)
    
    
def afficher_vies(vies):
    texte(50, 680, f"Vies : {vies}", couleur='white', ancrage='nw', police="Courier", taille=14)

 
def collision_joueur_sparx(x_joueur, y_joueur, x_sparx, y_sparx, taille_sparx):
    if time.time() > temps_invincible:  # Vérifier si le joueur n'est plus invincible
        distance = ((x_joueur - x_sparx) ** 2 + (y_joueur - y_sparx) ** 2) ** 0.5
        return distance < RAYON_JOUEUR + taille_sparx
    return False


def collision_joueur_qix(x_joueur, y_joueur, x_qix, y_qix, rayon_qix):
    if time.time() > temps_invincible:  
        distance = ((x_joueur - x_qix) ** 2 + (y_joueur - y_qix) ** 2) ** 0.5
        return distance < RAYON_JOUEUR + rayon_qix
    return False


def collision_joueur2_sparx(x_joueur2, y_joueur2, x_sparx, y_sparx, taille_sparx):
    if time.time() > temps_invincible2:  # Utiliser une variable de temps d'invincibilité différente pour le joueur 2
        distance = ((x_joueur2 - x_sparx) ** 2 + (y_joueur2 - y_sparx) ** 2) ** 0.5
        return distance < RAYON_JOUEUR + taille_sparx
    return False


def collision_joueur2_qix(x_joueur2, y_joueur2, x_qix, y_qix, rayon_qix):
    if time.time() > temps_invincible2:  # Utiliser une variable de temps d'invincibilité différente pour le joueur 2
        distance = ((x_joueur2 - x_qix) ** 2 + (y_joueur2 - y_qix) ** 2) ** 0.5
        return distance < RAYON_JOUEUR + rayon_qix
    return False


def afficher_invincibilite():
    if time.time() < temps_invincible or time.time() < temps_invincible2 :
        texte(LARGEUR_FENETRE / 2, 20, "Invincible!", couleur='gold', ancrage='center', police="Courier", taille=20)

 
def jeu_un_joueur():
    
    global victoire, NOMBRE_VIES, x_joueur, y_joueur, x_titre, y_titre
    ferme_fenetre()
    init_fenetre(LARGEUR_FENETRE, HAUTEUR_FENETRE)
    score = 0
    pourcentage_conquis = 0
    vies = NOMBRE_VIES
    obstacles = creer_obstacles(8, LARGEUR_MAP, HAUTEUR_MAP)

    while True:
        ev = donne_ev()
        tev = type_ev(ev)
        if tev == 'Touche':
            gerer_entree_clavier(ev)
        elif tev == 'Quitte':
            break

        efface_tout()
        dessiner_map()
        dessiner_zones_coloriees()
        dessiner_joueur()
        deplacer_joueur(obstacles)
        dessiner_ennemis()
        deplacer_sparx1()
        deplacer_sparx2()
        move_qix()
        afficher_vies(vies)
        dessiner_pommes()
        avale_pomme(x_joueur, y_joueur)
        dessiner_obstacles(obstacles)
        
        if temps_invincible > time.time():
            afficher_invincibilite()

        if dessin_en_cours:
            dessiner_chemin()

        # Mise à jour du pourcentage de couverture et du score
        pourcentage_conquis, score = calculer_aire_et_points(zones_coloriees)

        texte(x_titre, y_titre, "Qix", couleur='red', ancrage='nw', police="Arial", taille=50)
         
        # Vérifier la victoire
        if pourcentage_conquis >= POURCENTAGE_COUVERTURE_OBJECTIF:
            victoire = True

        # Affichage du pourcentage, du score et de l'animation de victoire si nécessaire
        afficher_statistiques(pourcentage_conquis, score)
        if victoire:
            afficher_animation_victoire()
            
        if collision_joueur_sparx(x_joueur, y_joueur, x_sparx1, y_sparx1, TAILLE_SPARX) or \
           collision_joueur_sparx(x_joueur, y_joueur, x_sparx2, y_sparx2, TAILLE_SPARX) or \
           collision_joueur_qix(x_joueur, y_joueur, x_qix, y_qix, RAYON_QIX):
            vies -= 1
            if vies <= 0:
                afficher_animation_fin()
            else:
                # Réinitialiser la position du joueur et continuer
                x_joueur, y_joueur = X_MAP + LARGEUR_MAP // 2, Y_MAP + HAUTEUR_MAP

        mise_a_jour()

    ferme_fenetre()
    
def afficher_vies2(vies):
    texte(750, 680, f"Vies J2: {vies}", couleur='white', ancrage='nw', police="Courier", taille=14)
    
def jeu_deux_joueurs():
    global victoire, NOMBRE_VIES, x_titre, y_titre, x_joueur, y_joueur, x_joueur2, y_joueur2, boost_actif, boost_actif2
    ferme_fenetre()
    init_fenetre(LARGEUR_FENETRE, HAUTEUR_FENETRE)
    
    score1 = score2 = 0
    pourcentage_conquis1 = pourcentage_conquis2 = 0
    vies1 = NOMBRE_VIES
    vies2 = NOMBRE_VIES2
    obstacles = creer_obstacles(8, LARGEUR_MAP, HAUTEUR_MAP)

    x_joueur, y_joueur = X_MAP, Y_MAP + HAUTEUR_MAP
    x_joueur2, y_joueur2 = X_MAP + LARGEUR_MAP, Y_MAP + HAUTEUR_MAP

    while True:
        ev = donne_ev()
        tev = type_ev(ev)
        if tev == 'Touche':
            gerer_entree_clavier(ev)  # Cette fonction doit maintenant gérer les entrées pour les deux joueurs
        elif tev == 'Quitte':
            break

        efface_tout()
        dessiner_map()
        dessiner_zones_coloriees()
        dessiner_joueur()
        deplacer_joueur(obstacles)  # Déplacer le joueur 1
        dessiner_joueur2()  # Fonction pour dessiner le joueur 2
        deplacer_joueur2(obstacles)  # Fonction pour déplacer le joueur 2
        dessiner_ennemis()
        deplacer_sparx1()
        deplacer_sparx2()
        move_qix()
        afficher_vies(vies1)  # Afficher les vies du joueur 1
        afficher_vies2(vies2)  # Fonction pour afficher les vies du joueur 2
        dessiner_pommes()
        avale_pomme(x_joueur, y_joueur)  # Joueur 1 avale une pomme
        avale_pomme(x_joueur2, y_joueur2)  # Joueur 2 avale une pomme
        dessiner_obstacles(obstacles)
        
        if temps_invincible > time.time():
            afficher_invincibilite()
        if temps_invincible2 > time.time():
            afficher_invincibilite()
        
        if dessin_en_cours:
            dessiner_chemin()

        # Mise à jour du pourcentage de couverture et du score pour chaque joueur
        pourcentage_conquis1, score1 = calculer_aire_et_points(zones_coloriees)
        pourcentage_conquis2, score2 = calculer_aire_et_points(zones_coloriees2)  # Zones coloriées par le joueur 2

        texte(x_titre, y_titre, "Qix", couleur='red', ancrage='nw', police="Arial", taille=50)
         
        # Vérifier la victoire pour chaque joueur
        if pourcentage_conquis1 >= POURCENTAGE_COUVERTURE_OBJECTIF or pourcentage_conquis2 >= POURCENTAGE_COUVERTURE_OBJECTIF:
            victoire = True

        # Affichage du pourcentage, du score et de l'animation de victoire si nécessaire
        afficher_statistiques(pourcentage_conquis1, score1)
        afficher_statistiques2(pourcentage_conquis2, score2)  # Fonction pour afficher les stats du joueur 2
        if victoire:
            afficher_animation_victoire()
            
        if collision_joueur_sparx(x_joueur, y_joueur, x_sparx1, y_sparx1, TAILLE_SPARX) or \
        collision_joueur_sparx(x_joueur, y_joueur, x_sparx2, y_sparx2, TAILLE_SPARX) or \
        collision_joueur_qix(x_joueur, y_joueur, x_qix, y_qix, RAYON_QIX):
            
            vies1 -= 1
            if vies1 <= 0:
                afficher_animation_fin()
            else:
                # Réinitialiser la position du joueur 1 et continuer
                x_joueur, y_joueur = X_MAP, Y_MAP + HAUTEUR_MAP

        if collision_joueur2_sparx(x_joueur2, y_joueur2, x_sparx1, y_sparx1, TAILLE_SPARX) or \
        collision_joueur2_sparx(x_joueur2, y_joueur2, x_sparx2, y_sparx2, TAILLE_SPARX) or \
        collision_joueur2_qix(x_joueur2, y_joueur2, x_qix, y_qix, RAYON_QIX):
            vies2 -= 1
            if vies2 <= 0:
                afficher_animation_fin()
            else:
                # Réinitialiser la position du joueur 2 et continuer
                x_joueur2, y_joueur2 = X_MAP + LARGEUR_MAP, Y_MAP + HAUTEUR_MAP
                
        mise_a_jour()

    ferme_fenetre()


def affiche_menu_fin(chemin_image):
    cree_fenetre(LARGEUR_FENETRE, HAUTEUR_FENETRE)
    image(LARGEUR_FENETRE / 2, 150, "QixImage.png")

    # Fonction pour créer un bouton avec coins arrondis
    def bouton_arrondi(x1, y1, x2, y2, texte_bouton):
        couleur_bouton = 'grey'
        rayon_arrondi = 20
        police_retro = "Courier"
        rectangle(x1 + rayon_arrondi, y1, x2 - rayon_arrondi, y2, remplissage=couleur_bouton)
        rectangle(x1, y1 + rayon_arrondi, x2, y2 - rayon_arrondi, remplissage=couleur_bouton)
        cercle(x1 + rayon_arrondi, y1 + rayon_arrondi, rayon_arrondi, remplissage=couleur_bouton)
        cercle(x2 - rayon_arrondi, y1 + rayon_arrondi, rayon_arrondi, remplissage=couleur_bouton)
        cercle(x1 + rayon_arrondi, y2 - rayon_arrondi, rayon_arrondi, remplissage=couleur_bouton)
        cercle(x2 - rayon_arrondi, y2 - rayon_arrondi, rayon_arrondi, remplissage=couleur_bouton)
        texte((x1 + x2) / 2, (y1 + y2) / 2, texte_bouton, couleur='white', ancrage='center', police=police_retro, taille=20)

    bouton_arrondi(325, 300, 575, 380, 'Recommencer')
    bouton_arrondi(325, 420, 575, 500, 'Quitter')
    
def gere_clic_menu_fin(x, y):
    if 325 <= x <= 575:
        if 300 <= y <= 380:
            return "Recommencer"
        elif 420 <= y <= 500:
            return "Quitter"
    return None


def afficher_statistiques(pourcentage, score):
    texte(50, 630, f"Aire : {pourcentage:.2f}%", couleur='white', ancrage='nw', police="Courier", taille=14)
    texte(50, 650, f"Score : {score}", couleur='white', ancrage='nw', police="Courier", taille=14)
    
    
def afficher_statistiques2(pourcentage, score):
    # Modifier la position pour ne pas chevaucher avec les statistiques du joueur 1
    texte(750, 630, f"Aire J2: {pourcentage:.2f}%", couleur='white', ancrage='nw', police="Courier", taille=14)
    texte(750, 650, f"Score J2: {score}", couleur='white', ancrage='nw', police="Courier", taille=14)

    
def afficher_animation_victoire():
    # Animation basique : texte clignotant
    if int(time.time() * 2) % 2:  # Clignote toutes les secondes
        texte(LARGEUR_FENETRE / 2, HAUTEUR_FENETRE / 2, "Victoire !", couleur='green', ancrage='center', police="Arial", taille=40)


def afficher_animation_fin():
    for _ in range(5):  # Clignote 5 fois
        efface_tout()
        texte(LARGEUR_FENETRE / 2, HAUTEUR_FENETRE / 2, "Game Over", couleur='red', ancrage='center', police="Courier", taille=40)
        mise_a_jour()
        time.sleep(0.5)  # Reste affiché pendant 0.5 seconde

        efface_tout()
        mise_a_jour()
        time.sleep(0.5)
         

def affiche_image(x, y, chemin_image):
    image(x, y, chemin_image, ancrage='center')
    
    
def affiche_menu_principal():
    init_fenetre(900, 700)
    rectangle(0, 0, 900, 700, remplissage='black')
    affiche_image(450, 150, "QixImage.png")

    # Fonction pour créer un bouton avec coins arrondis
    def bouton_arrondi(x1, y1, x2, y2, texte_bouton):
        couleur_bouton = 'grey'
        rayon_arrondi = 20
        police_retro = "Courier"
        rectangle(x1 + rayon_arrondi, y1, x2 - rayon_arrondi, y2, remplissage=couleur_bouton)
        rectangle(x1, y1 + rayon_arrondi, x2, y2 - rayon_arrondi, remplissage=couleur_bouton)
        cercle(x1 + rayon_arrondi, y1 + rayon_arrondi, rayon_arrondi, remplissage=couleur_bouton)
        cercle(x2 - rayon_arrondi, y1 + rayon_arrondi, rayon_arrondi, remplissage=couleur_bouton)
        cercle(x1 + rayon_arrondi, y2 - rayon_arrondi, rayon_arrondi, remplissage=couleur_bouton)
        cercle(x2 - rayon_arrondi, y2 - rayon_arrondi, rayon_arrondi, remplissage=couleur_bouton)
        texte((x1 + x2) / 2, (y1 + y2) / 2, texte_bouton, couleur='white', ancrage='center', police=police_retro, taille=20)

    # Position des boutons ajustée pour être en dessous des lettres QIX
    bouton_arrondi(325, 300, 575, 380, '1 Joueur')
    bouton_arrondi(325, 420, 575, 500, '2 Joueurs')
    bouton_arrondi(325, 540, 575, 620, 'Quitter')
    
    texte(450, 670, "By Irimbola and Yevhen", couleur='seagreen', ancrage='center', police="Courier", taille=20)


affiche_menu_principal()


# Gestion des clics sur les boutons
def gere_clic(x, y):
    global mode_deux_joueurs
    
    if 325 <= x <= 575:
        if 300 <= y <= 380:
            mode_deux_joueurs = False
            return "1 Joueur"
        elif 420 <= y <= 500:
            mode_deux_joueurs = True
            return "2 Joueurs"
        elif 540 <= y <= 620:
            return "Quitter"
    return None

while True:
    ev = donne_ev()
    tev = type_ev(ev)
    if tev == "ClicGauche":
        choix = gere_clic(abscisse(ev), ordonnee(ev))
        if choix == "1 Joueur":
            jeu_un_joueur() 
        if choix == "2 Joueurs":
            jeu_deux_joueurs()
        elif choix == "Quitter":
            break
    if tev == "Quitte":
        break
    
    mise_a_jour()
    
ferme_fenetre()