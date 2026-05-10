#include <stdio.h>      // pour printf (afficher les collisions)
#include <stdlib.h>     // pour malloc, free, rand
#include <math.h>       // pour les maths

#define NB_DRONES 10000          // nombre total de drones (impose par le projet)
#define SEUIL_COLLISION 5.0f     // distance en metres en dessous de laquelle on dit collision

struct Drone {                   // structure imposee par le cahier des charges
    int id;                      // identifiant unique du drone (de 0 a 9999)
    float x;                     // coordonnee X dans l'espace
    float y;                     // coordonnee Y dans l'espace
    float z;                     // coordonnee Z dans l'espace
};

#define TAILLE_DRONE sizeof(struct Drone)   // taille d'un drone en octets (environ 16)

#define OFFSET_ID 0                         // decalage du champ id (c'est le premier)
#define OFFSET_X (OFFSET_ID + sizeof(int))  // decalage du champ x (apres id)
#define OFFSET_Y (OFFSET_X + sizeof(float)) // decalage du champ y (apres x)
#define OFFSET_Z (OFFSET_Y + sizeof(float)) // decalage du champ z (apres y)

static int get_id(char *base, int idx)      // fonction pour lire l'id du drone numero idx
{
    return *(int *)(base + idx * TAILLE_DRONE + OFFSET_ID); // formule d'acces sans crochet
}

static void set_id(char *base, int idx, int val) // fonction pour ecrire l'id du drone idx
{
    *(int *)(base + idx * TAILLE_DRONE + OFFSET_ID) = val; // ecriture par arithmetique
}

static float get_x(char *base, int idx)      // fonction pour lire la coordonnee X
{
    return *(float *)(base + idx * TAILLE_DRONE + OFFSET_X); // acces par offset
}

static void set_x(char *base, int idx, float val) // fonction pour ecrire la coordonnee X
{
    *(float *)(base + idx * TAILLE_DRONE + OFFSET_X) = val; // ecriture par pointeur
}

static float get_y(char *base, int idx)      // fonction pour lire la coordonnee Y
{
    return *(float *)(base + idx * TAILLE_DRONE + OFFSET_Y); // acces par offset
}

static void set_y(char *base, int idx, float val) // fonction pour ecrire la coordonnee Y
{
    *(float *)(base + idx * TAILLE_DRONE + OFFSET_Y) = val; // ecriture par pointeur
}

static float get_z(char *base, int idx)      // fonction pour lire la coordonnee Z
{
    return *(float *)(base + idx * TAILLE_DRONE + OFFSET_Z); // acces par offset
}

static void set_z(char *base, int idx, float val) // fonction pour ecrire la coordonnee Z
{
    *(float *)(base + idx * TAILLE_DRONE + OFFSET_Z) = val; // ecriture par pointeur
}

static float distance_carree(char *base, int i, int j) // calcule le carre de la distance
{
    float dx = get_x(base, i) - get_x(base, j);   // difference des X
    float dy = get_y(base, i) - get_y(base, j);   // difference des Y
    float dz = get_z(base, i) - get_z(base, j);   // difference des Z
    return dx*dx + dy*dy + dz*dz;                 // retourne X² + Y² + Z² (pas de sqrt)
}

static void echanger(char *base, int i, int j)   // echange deux drones dans le tableau
{
    char *pi = base + i * TAILLE_DRONE;          // pointeur vers le drone i
    char *pj = base + j * TAILLE_DRONE;          // pointeur vers le drone j
    char tmp[TAILLE_DRONE];                     // tampon temporaire pour stocker un drone
    int k;                                       // compteur pour les boucles
    for (k = 0; k < TAILLE_DRONE; k++) {         // boucle sur tous les octets
        tmp[k] = *(pi + k);                      // copie drone i vers tampon
    }
    for (k = 0; k < TAILLE_DRONE; k++) {         // boucle sur tous les octets
        *(pi + k) = *(pj + k);                   // copie drone j vers i
    }
    for (k = 0; k < TAILLE_DRONE; k++) {         // boucle sur tous les octets
        *(pj + k) = tmp[k];                      // copie tampon vers j
    }
}

static void partitionner(char *base, int bas, int haut, int *pivot_idx) // partition pour tri
{
    float pivot_x = get_x(base, haut);           // valeur du pivot (dernier element)
    int i = bas - 1;                             // index de la zone des petits elements
    int j;                                       // compteur pour la boucle
    for (j = bas; j < haut; j++) {               // parcourt tous les elements sauf pivot
        if (get_x(base, j) <= pivot_x) {         // si element plus petit ou egal au pivot
            i++;                                 // on agrandit la zone des petits
            echanger(base, i, j);                // on place l'element dans la zone
        }
    }
    echanger(base, i + 1, haut);                 // on place le pivot a sa place finale
    *pivot_idx = i + 1;                          // on retourne l'index du pivot
}

static void tri_rapide(char *base, int bas, int haut) // tri recursif selon X
{
    if (bas < haut) {                            // si la zone a au moins 2 elements
        int pivot;                               // index du pivot
        partitionner(base, bas, haut, &pivot);   // partitionne la zone
        tri_rapide(base, bas, pivot - 1);        // trie recursivement la partie gauche
        tri_rapide(base, pivot + 1, haut);       // trie recursivement la partie droite
    }
}

void detecter_collisions(char *base, int n)      // detection principale des collisions
{
    tri_rapide(base, 0, n - 1);                  // ETAPE 1 : tri des drones selon X
    float seuil_carre = SEUIL_COLLISION * SEUIL_COLLISION; // seuil au carre (evite sqrt)
    int i, j;                                    // compteurs pour les boucles
    int compteur = 0;                            // compteur pour limiter l'affichage

    for (i = 0; i < n; i++) {                    // pour chaque drone de l'essaim
        float xi = get_x(base, i);               // on lit sa coordonnee X
        for (j = i + 1; j < n; j++) {            // on regarde les drones suivants
            float xj = get_x(base, j);           // on lit le X du drone suivant
            if (xj - xi > SEUIL_COLLISION) {     // si trop loin en X
                break;                           // on sort de la boucle (optimisation)
            }
            if (distance_carree(base, i, j) < seuil_carre) { // on verifie la vraie distance
                printf("COLLISION: %d <-> %d\n", get_id(base, i), get_id(base, j));
                compteur++;                      // on a trouve une collision
                if (compteur >= 10) {            // si on a deja 10 collisions
                    printf("\n--- Limite de 10 collisions atteinte ---\n");
                    return;                      // on arrete tout (pas besoin de plus)
                }
            }
        }
    }
}

int main(void)                                   // point d'entree du programme
{
    char *essaim = (char *)malloc(NB_DRONES * TAILLE_DRONE); // allocation unique (un seul bloc)
    if (!essaim) {                               // si l'allocation a echoue
        return 1;                                // on quitte avec erreur
    }
    int i;                                       // compteur pour la boucle
    for (i = 0; i < NB_DRONES; i++) {            // boucle pour initialiser les 10000 drones
        set_id(essaim, i, i);                    // id = i (de 0 a 9999)
        set_x(essaim, i, (float)(rand() % 20000) / 100.0f - 100.0f); // X entre -100 et 100
        set_y(essaim, i, (float)(rand() % 20000) / 100.0f - 100.0f); // Y entre -100 et 100
        set_z(essaim, i, (float)(rand() % 20000) / 100.0f - 100.0f); // Z entre -100 et 100
    }
    detecter_collisions(essaim, NB_DRONES);      // lance la detection des collisions
    free(essaim);                                // libere la memoire
    return 0;                                    // fin normale du programme
}