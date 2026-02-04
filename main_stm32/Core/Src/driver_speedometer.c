/**
 * @file    driver_speedometer.c
 * @brief   Driver de calcul de vitesse basé sur un capteur à effet Hall.
 * @details Utilise un timer en mode compteur pour mesurer le nombre d'impulsions
 * générées par la rotation de la roue et déduit la vitesse linéaire.
 */

#include "driver_speedometer.h"
#include "tim.h"

/**
 * @brief  Calcule la vitesse instantanée en m/s.
 * @details Cette fonction doit être appelée périodiquement. Elle calcule la différence
 * de temps et de nombre d'impulsions (ticks) depuis le dernier appel.
 * @note   Gère implicitement le débordement (overflow) du compteur 16 bits via
 * l'arithmétique non signée.
 * @param  hSpeedo Pointeur vers la structure de gestion du tachymètre.
 * @return Vitesse calculée en mètres par seconde (m/s).
 */
float speedometer_solve_speed(Speedometer_Handle_t *hSpeedo){
    uint32_t now = HAL_GetTick();
    uint32_t time_diff_ms = now - hSpeedo->last_process_time;

    if(time_diff_ms == 0){
        return hSpeedo->current_speed_ms;
    }

    uint16_t current_counter = (uint16_t)__HAL_TIM_GET_COUNTER(hSpeedo->htim);
    uint16_t pulses = current_counter - hSpeedo->last_counter_val;
    float wheel_turns = (float)pulses / TICKS_PER_WHEEL_TURN;
    float distance_m = wheel_turns * PERIMETER_M;
    float speed_ms = distance_m / ((float)time_diff_ms / 1000.0f);

    hSpeedo->last_counter_val = current_counter;
    hSpeedo->last_process_time = now;
    hSpeedo->current_speed_ms = speed_ms;

    return speed_ms;
}

/**
 * @brief  Initialise le driver tachymètre.
 * @details Associe le timer matériel à la structure, initialise les variables
 * d'état (temps et compteur) et démarre le timer.
 * @param  hSpeedo Pointeur vers la structure de gestion à initialiser.
 * @param  htim    Pointeur vers le handle du Timer STM32 (HAL) utilisé.
 */
void speedometer_init(Speedometer_Handle_t *hSpeedo, TIM_HandleTypeDef *htim){
    hSpeedo->htim = htim;
    hSpeedo->last_counter_val = (uint16_t)__HAL_TIM_GET_COUNTER(hSpeedo->htim);
    hSpeedo->last_process_time = HAL_GetTick();
    hSpeedo->current_speed_ms = 0.0f;

    HAL_TIM_Base_Start(hSpeedo->htim);
}
