/**
 * @file    driver_servo.h
 * @brief   Fichier d'en-tête pour le pilote du servomoteur.
 * @details Définit la structure de configuration (Handle) et les prototypes
 * des fonctions de pilotage (Angle, Pourcentage, Valeur absolue).
 */

#ifndef INC_DRIVER_SERVO_H_
#define INC_DRIVER_SERVO_H_

#include "tim.h"

/**
 * @brief Structure de configuration et de gestion du Servo.
 * @details Cette structure lie le matériel (Timer/Channel) aux bornes physiques
 * du servomoteur (valeurs PWM min/max).
 */
typedef struct {
    TIM_HandleTypeDef *htim;   ///< Pointeur vers le handle du Timer (HAL).
    uint32_t channel;          ///< Canal du Timer (ex: TIM_CHANNEL_1).
    uint16_t min_pulse_ticks;  ///< Valeur registre CCR pour la position min (ex: 3200).
    uint16_t max_pulse_ticks;  ///< Valeur registre CCR pour la position max (ex: 6400).
} Servo_Handle_t;

/**
 * @brief  Initialise le servo-moteur.
 * @param  hservo Pointeur vers le handle du servo.
 */
void servo_initialisation(Servo_Handle_t *hservo);

/**
 * @brief  Commande le servo en pourcentage (0-100%).
 * @param  hservo  Pointeur vers le handle du servo.
 * @param  percent Position cible en pourcentage.
 */
void servo_pwm_percent(Servo_Handle_t *hservo, uint8_t percent);

/**
 * @brief  Commande le servo en degrés.
 * @param  hservo Pointeur vers le handle du servo.
 * @param  angle  Angle cible en degrés (sera borné par le driver).
 */
void servo_pwm_angle_degree(Servo_Handle_t *hservo, int8_t angle);

/**
 * @brief  Commande le servo via une valeur absolue (Haute résolution).
 * @param  hservo    Pointeur vers le handle du servo.
 * @param  abs_value Valeur absolue normalisée (0 à 65535).
 */
void servo_pwm_angle_abs_value(Servo_Handle_t *hservo, uint16_t abs_value);

/**
 * @brief  Configure l'application de test (Fonction Debug).
 */
void app_test_config(void);

/**
 * @brief  Boucle principale de l'application de test (Fonction Debug).
 */
void app_test_loop(void);

#endif
