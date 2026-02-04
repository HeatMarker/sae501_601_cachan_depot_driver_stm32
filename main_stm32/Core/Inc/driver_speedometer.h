/**
 * @file    driver_speedometer.h
 * @brief   Fichier d'en-tête pour le capteur de vitesse (Tachymètre).
 * @details Contient les paramètres physiques du véhicule (diamètre roue),
 * les constantes de calibration du capteur (ticks par tour) et la définition
 * de la structure objet.
 */

#ifndef DRIVER_SPEEDOMETER_H
#define DRIVER_SPEEDOMETER_H

#include "main.h"
#include <math.h>

/** @brief Diamètre de la roue du véhicule en millimètres. */
#define WHEEL_DIAMETER_MM	68.0f

/** @brief Nombre de tours de roue effectués lors de la calibration. */
#define NB_TOURS_TEST		10.0f

/** @brief Nombre de ticks capteur relevés pour le nombre de tours de test. */
#define VALEUR_COMPTEUR_LUE	52.0f

/** @brief Nombre de ticks capteur correspondant à un tour complet de roue. */
#define TICKS_PER_WHEEL_TURN	(VALEUR_COMPTEUR_LUE / NB_TOURS_TEST)

/** @brief Périmètre de la roue en mètres (Distance pour un tour). */
#define PERIMETER_M             ((WHEEL_DIAMETER_MM * 3.14159f) / 1000.0f)

/**
 * @brief Structure de gestion du tachymètre.
 * @details Stocke l'état précédent du compteur et du temps pour calculer
 * la vitesse différentielle.
 */
typedef struct{
    TIM_HandleTypeDef *htim;        ///< Pointeur vers le Timer utilisé en mode compteur.
    uint16_t last_counter_val;      ///< Valeur du compteur lors de la dernière lecture.
    uint32_t last_process_time;     ///< Timestamp (ms) de la dernière lecture.
    float current_speed_ms;         ///< Vitesse actuelle calculée en m/s.
} Speedometer_Handle_t;

/**
 * @brief  Initialise l'objet tachymètre.
 * @param  hSpeedo Pointeur vers la structure à initialiser.
 * @param  htim    Pointeur vers le handle du Timer matériel.
 */
void speedometer_init(Speedometer_Handle_t *hSpeedo, TIM_HandleTypeDef *htim);

/**
 * @brief  Calcule la vitesse actuelle basée sur les impulsions reçues.
 * @param  hSpeedo Pointeur vers la structure du tachymètre.
 * @return Vitesse en m/s.
 */
float speedometer_solve_speed(Speedometer_Handle_t *hSpeedo);

#endif
