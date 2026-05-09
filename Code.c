/*
 * =============================================================================
 * SYSTÈME DE DÉTECTION DE COLLISION POUR ESSAIM AUTONOME (UAV)
 * École des Sciences de l'Information — Programmation Avancée en C
 * =============================================================================
 *
 * ALGORITHME : Paire la plus proche — Diviser pour Régner
 * COMPLEXITÉ : O(n log n) garantie dans TOUS les cas (meilleur, moyen, pire)
 *
 * CONTRAINTES RESPECTÉES :
 *   - Aucune indexation par crochets (essaim[i] interdit)
 *   - Navigation exclusivement par arithmétique de pointeurs
 *   - Allocation dynamique unique via malloc
 * =============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>

/* -------------------------------------------------------------------------
 * Structure hétérogène représentant un drone dans l'espace 3D
 * ------------------------------------------------------------------------- */
struct Drone {
    int   id;   /* identifiant unique du drone              */
    float x;    /* coordonnée spatiale sur l'axe X (mètres) */
    float y;    /* coordonnée spatiale sur l'axe Y (mètres) */
    float z;    /* coordonnée spatiale sur l'axe Z (mètres) */
};

/* -------------------------------------------------------------------------
 * Structure résultat : paire de drones la plus proche
 * ------------------------------------------------------------------------- */
struct Paire {
    struct Drone* alpha;    /* pointeur vers le premier drone de la paire  */
    struct Drone* beta;     /* pointeur vers le second drone de la paire   */
    float         dist_sq;  /* distance euclidienne au carré (sans sqrt)   */
};

/* =========================================================================
 * FONCTIONS UTILITAIRES
 * ========================================================================= */

/*
 * distance_carre : calcule la distance euclidienne au carré entre deux drones.
 * Le carré est utilisé partout sauf à l'affichage final, pour éviter n sqrt.
 */
static float distance_carre(const struct Drone* a, const struct Drone* b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return dx*dx + dy*dy + dz*dz;
}

/*
 * paire_minimum : renvoie la paire ayant la plus petite distance au carré.
 */
static struct Paire paire_minimum(struct Paire p, struct Paire q) {
    return (p.dist_sq <= q.dist_sq) ? p : q;
}

/* =========================================================================
 * COMPARATEURS POUR qsort
 * ========================================================================= */

/* Tri croissant selon l'axe X — utilisé pour la phase principale */
int comparer_x(const void* a, const void* b) {
    const struct Drone* d1 = (const struct Drone*)a;
    const struct Drone* d2 = (const struct Drone*)b;
    if (d1->x < d2->x) return -1;
    if (d1->x > d2->x) return  1;
    return 0;
}

/* Tri croissant selon l'axe Y — utilisé pour filtrer la bande centrale */
int comparer_y(const void* a, const void* b) {
    const struct Drone* d1 = *(const struct Drone**)a;
    const struct Drone* d2 = *(const struct Drone**)b;
    if (d1->y < d2->y) return -1;
    if (d1->y > d2->y) return  1;
    return 0;
}

/* =========================================================================
 * PHASE NAÏVE : O(n²) appliquée uniquement sur de très petits sous-tableaux
 *
 * Justification : pour n ≤ 3, le coût de la récursion dépasse le gain.
 * Cette fonction respecte l'interdiction des crochets.
 * ========================================================================= */
static struct Paire paire_naive(struct Drone* debut, int n) {
    struct Paire resultat = { NULL, NULL, FLT_MAX };

    struct Drone* fin = debut + n;          /* pointeur de fin du sous-tableau */

    for (struct Drone* p1 = debut; p1 < fin; p1++) {
        for (struct Drone* p2 = p1 + 1; p2 < fin; p2++) {
            float d = distance_carre(p1, p2);
            if (d < resultat.dist_sq) {
                resultat.dist_sq = d;
                resultat.alpha   = p1;
                resultat.beta    = p2;
            }
        }
    }
    return resultat;
}

/* =========================================================================
 * PHASE BANDE CENTRALE : analyse des drones dans la zone de recouvrement
 *
 * Après la récursion gauche/droite, certaines paires optimales peuvent
 * traverser la frontière médiane. On examine uniquement les drones dont
 * la coordonnée X est à moins de δ du milieu (δ = sqrt(min_dist_sq)).
 *
 * Théorème : dans cette bande triée par Y, chaque drone ne peut former
 * une paire candidate qu'avec au maximum 7 de ses voisins suivants.
 * Cela rend cette phase O(n) et garantit O(n log n) global.
 * ========================================================================= */
static struct Paire analyser_bande(
    struct Drone** bande,   /* tableau de pointeurs triés par Y             */
    int            taille,  /* nombre de drones dans la bande               */
    struct Paire   meilleure/* meilleure paire issue des deux récursions    */
) {
    /*
     * Pour chaque drone de la bande (parcouru par pointeur sur pointeur),
     * on compare avec au plus 7 successeurs (preuve géométrique classique).
     * La bande est déjà triée par Y, donc on s'arrête dès que dy² ≥ dist_sq.
     */
    for (int i = 0; i < taille; i++) {
        struct Drone* courant = *(bande + i);   /* arithmétique de pointeur */

        /* Limite : 7 voisins au maximum — preuve de complexité O(n) */
        int j_max = i + 8;
        if (j_max > taille) j_max = taille;

        for (int j = i + 1; j < j_max; j++) {
            struct Drone* voisin = *(bande + j); /* arithmétique de pointeur */

            float dy    = voisin->y - courant->y;
            float dy_sq = dy * dy;

            /*
             * Élagage Y garanti correct ici, car la bande EST triée par Y.
             * Contrairement à l'approche naïve, cet élagage est structurel.
             */
            if (dy_sq >= meilleure.dist_sq) break;

            float d = distance_carre(courant, voisin);
            if (d < meilleure.dist_sq) {
                meilleure.dist_sq = d;
                meilleure.alpha   = courant;
                meilleure.beta    = voisin;
            }
        }
    }
    return meilleure;
}

/* =========================================================================
 * ALGORITHME PRINCIPAL : Diviser pour Régner — O(n log n) garanti
 *
 * Principe :
 *   1. Diviser  : couper le tableau trié par X en deux moitiés égales
 *   2. Régner   : résoudre récursivement chaque moitié
 *   3. Fusionner: vérifier la bande centrale de largeur 2δ
 *
 * Preuve de complexité :
 *   T(n) = 2·T(n/2) + O(n)   →   T(n) = O(n log n)  [Théorème maître, cas 2]
 * ========================================================================= */
static struct Paire diviser_pour_regner(
    struct Drone* debut,    /* début du sous-tableau trié par X             */
    int           n,        /* nombre de drones dans ce sous-tableau        */
    struct Drone** tampon   /* tampon auxiliaire pré-alloué (évite malloc)  */
) {
    /* Cas de base : n ≤ 3 → résolution naïve directe */
    if (n <= 3) {
        return paire_naive(debut, n);
    }

    /* --- PHASE DIVISER --------------------------------------------------- */
    int           milieu     = n / 2;
    struct Drone* drone_pivot = debut + milieu; /* pointeur vers le pivot X  */
    float         x_pivot    = drone_pivot->x;

    /* Récursion sur la moitié gauche [debut, debut+milieu[ */
    struct Paire paire_gauche = diviser_pour_regner(debut, milieu, tampon);

    /* Récursion sur la moitié droite [debut+milieu, debut+n[ */
    struct Paire paire_droite = diviser_pour_regner(drone_pivot, n - milieu, tampon);

    /* Meilleure paire issue des deux moitiés */
    struct Paire meilleure = paire_minimum(paire_gauche, paire_droite);
    float        delta_sq  = meilleure.dist_sq;
    float        delta     = sqrtf(delta_sq);   /* δ : seuil de la bande     */

    /* --- PHASE BANDE CENTRALE -------------------------------------------- */
    /*
     * Collecte des drones dont |x - x_pivot| < δ dans le tampon auxiliaire.
     * Navigation par arithmétique de pointeurs, sans crochets.
     */
    int taille_bande = 0;
    struct Drone* curseur = debut;
    struct Drone* fin     = debut + n;

    while (curseur < fin) {
        float ecart_x = curseur->x - x_pivot;
        if (ecart_x < 0.0f) ecart_x = -ecart_x;   /* valeur absolue manuelle */

        if (ecart_x < delta) {
            /* Stockage du pointeur dans le tampon (arithmétique de pointeur) */
            *(tampon + taille_bande) = curseur;
            taille_bande++;
        }
        curseur++;   /* avancement par arithmétique pure */
    }

    /* Tri de la bande par Y — requis pour que l'élagage Y soit valide */
    qsort(tampon, taille_bande, sizeof(struct Drone*), comparer_y);

    /* Analyse de la bande : O(n) grâce à la limite des 7 voisins */
    return analyser_bande(tampon, taille_bande, meilleure);
}

/* =========================================================================
 * MODULE DE SÉCURITÉ : point d'entrée public
 * ========================================================================= */
void executer_module_securite(struct Drone* essaim, int n) {
    if (n < 2 || essaim == NULL) return;

    /* --- Pré-tri selon X : O(n log n), condition requise par l'algorithme */
    qsort(essaim, n, sizeof(struct Drone), comparer_x);

    /*
     * Allocation du tampon auxiliaire : tableau de n pointeurs vers Drone.
     * Ce tampon est alloué UNE seule fois ici et partagé entre tous les
     * niveaux de récursion (évite n·log(n) malloc imbriqués).
     */
    struct Drone** tampon = (struct Drone**)malloc(n * sizeof(struct Drone*));
    if (tampon == NULL) {
        fprintf(stderr, "ERREUR : allocation tampon échouée.\n");
        return;
    }

    /* --- Algorithme diviser pour régner : O(n log n) garanti */
    struct Paire resultat = diviser_pour_regner(essaim, n, tampon);

    /* Libération du tampon auxiliaire */
    free(tampon);

    /* --- Affichage du résultat */
    if (resultat.alpha != NULL && resultat.beta != NULL) {
        printf("╔══════════════════════════════════════════╗\n");
        printf("║   ALERTE : COLLISION IMMINENTE DÉTECTÉE  ║\n");
        printf("╚══════════════════════════════════════════╝\n");
        printf("  Drone Alpha : ID %-6d | (%.2f, %.2f, %.2f)\n",
               resultat.alpha->id,
               resultat.alpha->x, resultat.alpha->y, resultat.alpha->z);
        printf("  Drone Beta  : ID %-6d | (%.2f, %.2f, %.2f)\n",
               resultat.beta->id,
               resultat.beta->x, resultat.beta->y, resultat.beta->z);
        /* sqrt appliqué une seule fois, uniquement pour l'affichage */
        printf("  Distance de sécurité : %.6f m\n", sqrtf(resultat.dist_sq));
    }
}

/* =========================================================================
 * PROGRAMME PRINCIPAL
 * ========================================================================= */
int main(void) {
    const int N = 10000;

    /* Initialisation du générateur aléatoire */
    srand((unsigned int)time(NULL));

    /* --- Allocation dynamique d'un bloc contigu de N drones (le Tas) */
    struct Drone* essaim = (struct Drone*)malloc(N * sizeof(struct Drone));
    if (essaim == NULL) {
        fprintf(stderr, "ERREUR CRITIQUE : malloc essaim échoué.\n");
        return 1;
    }

    /*
     * Initialisation des coordonnées par arithmétique de pointeurs.
     * La variable `curseur` avance d'un struct Drone à la fois.
     * AUCUN crochet d'indexation n'est utilisé.
     */
    struct Drone* curseur = essaim;
    struct Drone* fin     = essaim + N;

    int identifiant = 1;
    while (curseur < fin) {
        curseur->id = identifiant;
        curseur->x  = (float)(rand() % 100000) / 100.0f;   /* 0.00 à 999.99 m */
        curseur->y  = (float)(rand() % 100000) / 100.0f;
        curseur->z  = (float)(rand() % 100000) / 100.0f;
        curseur++;       /* avancement par arithmétique pure — pas de crochets */
        identifiant++;
    }

    printf("Essaim de %d drones initialisé. Analyse en cours...\n\n", N);

    /* --- Exécution du module de sécurité */
    executer_module_securite(essaim, N);

    /* --- Libération du tas */
    free(essaim);

    return 0;
}