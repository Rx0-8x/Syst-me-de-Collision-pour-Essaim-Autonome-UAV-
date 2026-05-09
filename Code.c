#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

/* Structure heterogene representant un drone dans l'espace 3D */
struct Drone {
    int   id;   /* identifiant unique du drone */
    float x;    /* coordonnee spatiale X */
    float y;    /* coordonnee spatiale Y */
    float z;    /* coordonnee spatiale Z */
};

/* Comparateur pour qsort : ordonne les drones selon l'axe x (croissant) */
int comparer_x(const void* a, const void* b) {
    const struct Drone* d1 = (const struct Drone*)a;
    const struct Drone* d2 = (const struct Drone*)b;
    if (d1->x < d2->x) return -1;
    if (d1->x > d2->x) return  1;
    return 0;
}

/*
 * Module de securite principal.
 * Algorithme : tri selon x + elagage tri-axial.
 * Complexite : O(n log n) en moyenne.
 */
void executer_module_securite(struct Drone* essaim, int n) {
    if (n < 2 || essaim == NULL) return;

    /* Phase 1 : tri pour activer la condition d'arret precoce */
    qsort(essaim, n, sizeof(struct Drone), comparer_x);

    float min_dist_sq = FLT_MAX; /* meilleure distance au carre */
    struct Drone* drone_a = NULL;
    struct Drone* drone_b = NULL;

    /* Pointeur de fin calcule une seule fois hors des boucles */
    struct Drone* fin_essaim = essaim + n;

    /* Phase 2 : recherche avec elagage tri-axial */
    for (struct Drone* p1 = essaim; p1 < fin_essaim; p1++) {
        for (struct Drone* p2 = p1 + 1; p2 < fin_essaim; p2++) {

            float dx    = (p2->x) - (p1->x);
            float dx_sq = dx * dx;

            /* Elagage x : dx croissant apres tri -> break garanti correct */
            if (dx_sq >= min_dist_sq) break;

            float dy    = (p2->y) - (p1->y);
            float dy_sq = dy * dy;
            /* Elagage y : candidat trop eloigne, on passe au suivant */
            if (dy_sq >= min_dist_sq) continue;

            float dz    = (p2->z) - (p1->z);
            float dz_sq = dz * dz;
            /* Elagage z : idem */
            if (dz_sq >= min_dist_sq) continue;

            /* Calcul complet uniquement si les 3 elaguages sont passes */
            float dist_actuelle_sq = dx_sq + dy_sq + dz_sq;

            if (dist_actuelle_sq < min_dist_sq) {
                min_dist_sq = dist_actuelle_sq;
                drone_a = p1;
                drone_b = p2;
            }
        }
    }

    if (drone_a != NULL && drone_b != NULL) {
        printf("ALERTE : Collision imminente detectee.\n");
        printf("Drone Alpha: ID %d | Drone Beta: ID %d\n",
               drone_a->id, drone_b->id);
        /* sqrtf appele une seule fois, uniquement pour l'affichage */
        printf("Distance de securite: %f m\n", sqrtf(min_dist_sq));
    }
}

int main() {
    int N = 10000;

    /* Allocation dynamique d'un bloc contigu de N drones */
    struct Drone* essaim = (struct Drone*)malloc(N * sizeof(struct Drone));
    if (essaim == NULL) return 1; /* echec malloc */

    /* Initialisation par arithmetique de pointeurs (indexation interdite) */
    struct Drone* curseur = essaim;
    for (int i = 0; i < N; i++) {
        curseur->id = i + 1;
        curseur->x  = (float)(rand() % 5000) / 10.0f;
        curseur->y  = (float)(rand() % 5000) / 10.0f;
        curseur->z  = (float)(rand() % 5000) / 10.0f;
        curseur++; /* avancement par arithmetique pure */
    }

    executer_module_securite(essaim, N);

    free(essaim); /* liberation du tas */
    return 0;
}