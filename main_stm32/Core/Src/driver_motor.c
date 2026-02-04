/**
 * @file    driver_motor.c
 * @brief   Pilote de moteur non-bloquant pour ESC (Electronic Speed Controller).
 * @details Implémente une machine à états finis pour gérer les transitions de sécurité
 * du moteur (Accélération, Freinage, Passage au neutre, Inversion de sens)
 * sans bloquer le processeur.
 */

#include "driver_motor.h"
#include "tim.h"

// 64Mhz - PSC=19 - ARR=63999 soit PWM{50Hz, duty=50%}

/** @brief Durée de l'état de freinage actif lors d'une inversion de sens (ms). */
#define T_BRAKE_MS          120
/** @brief Durée de pause au point mort après un freinage (ms). */
#define T_NEUTRAL_GAP_MS    120
/** @brief Rapport cyclique (%) correspondant au point mort (arrêt). */
#define PWM_NEUTRAL         50u
/** @brief Rapport cyclique (%) pour le freinage en marche arrière. */
#define PWM_BRAKE_REV       40u
/** @brief Rapport cyclique (%) pour le freinage en marche avant. */
#define PWM_BRAKE_FWD       60u

static uint8_t motor_speed_mms_to_pwm_percent(Motor_Handle_t *hmotor, int16_t value);

/**
 * @brief  Écrit la valeur brute dans le registre de comparaison du Timer PWM.
 * @param  hmotor Pointeur vers le handle du moteur.
 * @param  value  Valeur en ticks d'horloge à appliquer.
 */
static inline void pwm_pulse(Motor_Handle_t *hmotor, uint16_t value){
    if (hmotor && hmotor->htim) {
        __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->channel, value);
    }
}

/**
 * @brief  Mappe un pourcentage (0-100%) vers la plage de ticks du Timer.
 * @param  hmotor  Pointeur vers le handle du moteur (contient min/max ticks).
 * @param  percent Pourcentage cible.
 * @return Valeur correspondante en ticks Timer.
 */
static inline uint16_t motor_map_percent(Motor_Handle_t *hmotor, int16_t percent){
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;

    const uint16_t min = hmotor->min_pulse_ticks;
    const uint16_t max = hmotor->max_pulse_ticks;

    return (uint16_t)(min + ((uint32_t)(max - min) * (uint32_t)percent) / 100u);
}

/**
 * @brief  Convertit une vitesse linéaire (mm/s) en pourcentage PWM.
 * @note   Gère l'asymétrie des vitesses maximales avant et arrière.
 * @param  hmotor Pointeur vers le handle du moteur.
 * @param  value  Vitesse cible en mm/s.
 * @return Pourcentage PWM calculé (0 à 100).
 */
static uint8_t motor_speed_mms_to_pwm_percent(Motor_Handle_t *hmotor, int16_t value){
    if (value >= hmotor->max_speed_pos_mms) return 100;
    if (value <= hmotor->max_speed_neg_mms) return 0;

    if (value >= 0)
        return (uint8_t)(50 + ((int32_t)value * 50) / hmotor->max_speed_pos_mms);
    else
        return (uint8_t)(50 + ((int32_t)value * 50) / (-hmotor->max_speed_neg_mms));
}

/**
 * @brief  Vérifie si une échéance temporelle est dépassée.
 * @note   Gère le débordement (overflow) du compteur système 32 bits.
 * @param  now      Temps actuel (ms).
 * @param  deadline Temps cible (ms).
 * @return true si le temps est écoulé, false sinon.
 */
static inline bool time_reached(uint32_t now, uint32_t deadline){
    return (int32_t)(now - deadline) >= 0;
}

/**
 * @brief  Initialise le moteur et sa machine à états.
 * @details Place le moteur au neutre, réinitialise le contexte de commande
 * et démarre la génération PWM hardware.
 * @param  hmotor Pointeur vers le handle du moteur à initialiser.
 */
void motor_init(Motor_Handle_t *hmotor){
    if(hmotor && hmotor->htim){
        hmotor->state = MOTOR_STATE_NEUTRAL;
        hmotor->go_forward = true;
        hmotor->ctx.target_speed_mms = 0;
        hmotor->ctx.target_pwm = PWM_NEUTRAL;
        hmotor->ctx.target_forward = true;
        hmotor->ctx.deadline_ms = 0;

        HAL_TIM_PWM_Start(hmotor->htim, hmotor->channel);
    }
}

/**
 * @brief  Force une commande PWM directe en pourcentage.
 * @note   Utilisé principalement par la machine à états pour appliquer
 * les commandes de freinage ou de neutre.
 * @param  hmotor  Pointeur vers le handle du moteur.
 * @param  percent Pourcentage PWM cible (0-100).
 */
void motor_pwm_percent(Motor_Handle_t *hmotor, uint8_t percent){
    uint16_t value = motor_map_percent(hmotor, (int16_t)percent);
    pwm_pulse(hmotor, value);
}

/**
 * @brief  Définit la nouvelle consigne de vitesse cible.
 * @details Met à jour le contexte cible. La machine à états (motor_process_1ms)
 * se chargera d'atteindre cette cible en respectant les séquences de freinage.
 * @param  hmotor    Pointeur vers le handle du moteur.
 * @param  speed_mms Vitesse cible en mm/s (Positif = Avant, Négatif = Arrière).
 */
void motor_set_speed_mms(Motor_Handle_t *hmotor, int16_t speed_mms){
    hmotor->ctx.target_speed_mms = speed_mms;

    if(speed_mms == 0){
        hmotor->ctx.target_pwm = PWM_NEUTRAL;
    }
    else{
        hmotor->ctx.target_pwm = motor_speed_mms_to_pwm_percent(hmotor, speed_mms);
        hmotor->ctx.target_forward = (speed_mms > 0);
    }
}

/**
 * @brief  Machine à états principale de gestion du moteur.
 * @details Doit être appelée périodiquement (ex: 1kHz). Gère la logique séquentielle :
 * - Marche Avant -> Neutre -> Marche Arrière : Passage direct (géré par ESC en général) ou via frein.
 * - Inversion brusque : Application d'une séquence Frein -> Pause -> Nouveau sens.
 * @param  hmotor Pointeur vers le handle du moteur.
 * @param  now_ms Temps système actuel en millisecondes.
 */
void motor_process_1ms(Motor_Handle_t *hmotor, uint32_t now_ms){
    if (!hmotor) return;

    const bool want_neutral = (hmotor->ctx.target_speed_mms == 0);

    switch (hmotor->state){
        case MOTOR_STATE_NEUTRAL:
            motor_pwm_percent(hmotor, PWM_NEUTRAL);

            if(!want_neutral){
                if(hmotor->ctx.target_forward){
                    motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                    hmotor->go_forward = true;
                    hmotor->state = MOTOR_STATE_FORWARD_HOLD;
                }
                else{
                    motor_pwm_percent(hmotor, PWM_BRAKE_REV);
                    hmotor->ctx.deadline_ms = now_ms + T_BRAKE_MS;
                    hmotor->state = MOTOR_STATE_NEUTRAL_TO_REVERSE_TAP;
                }
            }
            break;

        case MOTOR_STATE_NEUTRAL_TO_REVERSE_TAP:
            if (!time_reached(now_ms, hmotor->ctx.deadline_ms)) break;
            motor_pwm_percent(hmotor, PWM_NEUTRAL);
            hmotor->ctx.deadline_ms = now_ms + T_NEUTRAL_GAP_MS;
            hmotor->state = MOTOR_STATE_NEUTRAL_TO_REVERSE_GAP;
            break;

        case MOTOR_STATE_NEUTRAL_TO_REVERSE_GAP:
            if(!time_reached(now_ms, hmotor->ctx.deadline_ms)) break;
            if(want_neutral){
                motor_pwm_percent(hmotor, PWM_NEUTRAL);
                hmotor->state = MOTOR_STATE_NEUTRAL;
            }
            else if (hmotor->ctx.target_forward){
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                hmotor->go_forward = true;
                hmotor->state = MOTOR_STATE_FORWARD_HOLD;
            }
            else{
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                hmotor->go_forward = false;
                hmotor->state = MOTOR_STATE_REVERSE_HOLD;
            }
            break;

        case MOTOR_STATE_FORWARD_HOLD:
            if(want_neutral){
                motor_pwm_percent(hmotor, PWM_NEUTRAL);
                hmotor->state = MOTOR_STATE_NEUTRAL;
                break;
            }
            if(hmotor->ctx.target_forward){
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
            }
            else{
                motor_pwm_percent(hmotor, PWM_BRAKE_REV);
                hmotor->ctx.deadline_ms = now_ms + T_BRAKE_MS;
                hmotor->state = MOTOR_STATE_FWD_BRAKE_TAP;
            }
            break;

        case MOTOR_STATE_FWD_BRAKE_TAP:
            if(!time_reached(now_ms, hmotor->ctx.deadline_ms)) break;
            motor_pwm_percent(hmotor, PWM_NEUTRAL);
            hmotor->ctx.deadline_ms = now_ms + T_NEUTRAL_GAP_MS;
            hmotor->state = MOTOR_STATE_FWD_NEUTRAL_GAP;
            break;

        case MOTOR_STATE_FWD_NEUTRAL_GAP:
            if(!time_reached(now_ms, hmotor->ctx.deadline_ms)) break;
            if(want_neutral){
                motor_pwm_percent(hmotor, PWM_NEUTRAL);
                hmotor->state = MOTOR_STATE_NEUTRAL;
            }
            else if(hmotor->ctx.target_forward){
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                hmotor->go_forward = true;
                hmotor->state = MOTOR_STATE_FORWARD_HOLD;
            }
            else{
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                hmotor->go_forward = false;
                hmotor->state = MOTOR_STATE_REVERSE_HOLD;
            }
            break;

        case MOTOR_STATE_REVERSE_HOLD:
            if(want_neutral){
                motor_pwm_percent(hmotor, PWM_NEUTRAL);
                hmotor->state = MOTOR_STATE_NEUTRAL;
                break;
            }
            if(!hmotor->ctx.target_forward){
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
            }
            else{
                motor_pwm_percent(hmotor, PWM_BRAKE_FWD);
                hmotor->ctx.deadline_ms = now_ms + T_BRAKE_MS;
                hmotor->state = MOTOR_STATE_REV_BRAKE_TAP;
            }
            break;

        case MOTOR_STATE_REV_BRAKE_TAP:
            if(!time_reached(now_ms, hmotor->ctx.deadline_ms)) break;
            motor_pwm_percent(hmotor, PWM_NEUTRAL);
            hmotor->ctx.deadline_ms = now_ms + T_NEUTRAL_GAP_MS;
            hmotor->state = MOTOR_STATE_REV_NEUTRAL_GAP;
            break;

        case MOTOR_STATE_REV_NEUTRAL_GAP:
            if(!time_reached(now_ms, hmotor->ctx.deadline_ms)) break;
            if(want_neutral){
                motor_pwm_percent(hmotor, PWM_NEUTRAL);
                hmotor->state = MOTOR_STATE_NEUTRAL;
            }
            else if(!hmotor->ctx.target_forward){
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                hmotor->go_forward = false;
                hmotor->state = MOTOR_STATE_REVERSE_HOLD;
            }
            else{
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                hmotor->go_forward = true;
                hmotor->state = MOTOR_STATE_FORWARD_HOLD;
            }
            break;
    }
}
