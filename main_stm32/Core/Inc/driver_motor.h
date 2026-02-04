/**
 * @file    driver_motor.h
 * @brief   Fichier d'en-tête pour le pilote moteur (ESC).
 * @details Contient les définitions des états de la machine à états (FSM),
 * les structures de configuration et les prototypes des fonctions de contrôle.
 */

#ifndef INC_DRIVER_MOTOR_H_
#define INC_DRIVER_MOTOR_H_

#include "tim.h"
#include <stdbool.h>

/**
 * @brief Énumération des états de la machine à états du moteur.
 * @details Gère les transitions complexes comme le freinage actif et les pauses au neutre.
 */
typedef enum{
    MOTOR_STATE_NEUTRAL = 0,                ///< Moteur au point mort (arrêt).
    MOTOR_STATE_FORWARD_HOLD,               ///< Marche avant stable.
    MOTOR_STATE_FWD_BRAKE_TAP,              ///< Séquence de freinage (depuis l'avant).
    MOTOR_STATE_FWD_NEUTRAL_GAP,            ///< Pause au neutre après freinage avant.
    MOTOR_STATE_REVERSE_HOLD,               ///< Marche arrière stable.
    MOTOR_STATE_REV_BRAKE_TAP,              ///< Séquence de freinage (depuis l'arrière).
    MOTOR_STATE_REV_NEUTRAL_GAP,            ///< Pause au neutre après freinage arrière.
    MOTOR_STATE_NEUTRAL_TO_REVERSE_TAP,     ///< Coup de frein pour enclencher la marche arrière (spécifique ESC).
    MOTOR_STATE_NEUTRAL_TO_REVERSE_GAP      ///< Pause avant d'enclencher la marche arrière.
} MotorState_t;

/**
 * @brief Structure de contexte interne pour la gestion des transitions.
 */
typedef struct{
    int16_t  target_speed_mms;     ///< Vitesse cible demandée en mm/s.
    uint8_t  target_pwm;           ///< Pourcentage PWM cible calculé (0-100).
    bool     target_forward;       ///< Direction cible (true=avant, false=arrière).
    uint32_t deadline_ms;          ///< Echéance temporelle pour les états temporisés.
} Motor_Context_t;

/**
 * @brief Handle principal de l'objet Moteur.
 * @details Contient la configuration matérielle, les limites physiques et l'état courant.
 */
typedef struct{
    TIM_HandleTypeDef *htim;       ///< Pointeur vers le handle du Timer PWM (ex: &htim2).
    uint32_t channel;              ///< Canal du Timer utilisé (ex: TIM_CHANNEL_1).
    uint16_t min_pulse_ticks;      ///< Valeur du registre CCR pour 0% de PWM.
    uint16_t max_pulse_ticks;      ///< Valeur du registre CCR pour 100% de PWM.

    int16_t  max_speed_pos_mms;    ///< Vitesse physique maximale en marche avant (mm/s).
    int16_t  max_speed_neg_mms;    ///< Vitesse physique maximale en marche arrière (mm/s, valeur négative).

    MotorState_t    state;         ///< État actuel de la machine à états.
    bool            go_forward;    ///< Direction actuelle appliquée.
    Motor_Context_t ctx;           ///< Contexte de transition (cible).
} Motor_Handle_t;

/**
 * @brief  Initialise le driver moteur.
 * @param  hmotor Pointeur vers la structure de configuration.
 */
void motor_init(Motor_Handle_t *hmotor);

/**
 * @brief  Applique directement un pourcentage de PWM (Bypass FSM).
 * @param  hmotor  Pointeur vers le handle moteur.
 * @param  percent Pourcentage du cycle (0 à 100).
 */
void motor_pwm_percent(Motor_Handle_t *hmotor, uint8_t percent);

/**
 * @brief  Définit la consigne de vitesse en mm/s.
 * @param  hmotor    Pointeur vers le handle moteur.
 * @param  speed_mms Vitesse souhaitée (Positive=Avant, Négative=Arrière).
 */
void motor_set_speed_mms(Motor_Handle_t *hmotor, int16_t speed_mms);

/**
 * @brief  Fonction de traitement cyclique (Machine à états).
 * @param  hmotor Pointeur vers le handle moteur.
 * @param  now_ms Timestamp actuel en ms.
 */
void motor_process_1ms(Motor_Handle_t *hmotor, uint32_t now_ms);

/**
 * @brief  Configuration globale de l'application (Callback ou Init).
 */
void app_config(void);

#endif
