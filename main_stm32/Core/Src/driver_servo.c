/**
 * @file    driver_servo.c
 * @brief   Pilote PWM pour le servomoteur de direction.
 * @details Fournit les fonctions de conversion (Degrés/Pourcentage/Absolu vers PWM)
 * et de gestion des limites mécaniques pour le servo.
 */

#include "tim.h"
#include "driver_servo.h"

/** @brief Décalage (offset) en pourcentage appliqué à la commande (Trim). */
#define SERVO_OFFSET_PERCENT  5
/** @brief Angle minimum autorisé en degrés (Borne mécanique logicielle). */
#define SERVO_CLAMP_MIN       -20
/** @brief Angle maximum autorisé en degrés (Borne mécanique logicielle). */
#define SERVO_CLAMP_MAX       20

//static inline void pwm_pulse(Servo_Handle_t *hservo, uint16_t value);
static int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

/**
 * @brief  Applique la valeur de comparaison (CCR) au registre du Timer.
 * @param  hservo Pointeur vers le handle du servo.
 * @param  value  Valeur brute en ticks d'horloge.
 */
static inline void pwm_pulse(Servo_Handle_t *hservo, uint16_t value){
    if (hservo && hservo->htim) {
        __HAL_TIM_SET_COMPARE(hservo->htim, hservo->channel, value);
    }
}

/**
 * @brief  Fonction utilitaire de mappage linéaire (Règle de trois).
 * @param  x       Valeur d'entrée à mapper.
 * @param  in_min  Borne inférieure d'entrée.
 * @param  in_max  Borne supérieure d'entrée.
 * @param  out_min Borne inférieure de sortie.
 * @param  out_max Borne supérieure de sortie.
 * @return Valeur mappée dans la plage de sortie.
 */
static int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief  Convertit un pourcentage théorique en ticks Timer avec gestion d'offset.
 * @note   Applique SERVO_OFFSET_PERCENT et borne le résultat entre 0 et 100 avant calcul.
 * @param  hservo  Pointeur vers le handle du servo.
 * @param  percent Pourcentage d'entrée (peut être négatif avant correction offset).
 * @return Valeur en ticks Timer correspondant au pourcentage corrigé.
 */
static inline uint16_t servo_map_percent(Servo_Handle_t *hservo, int16_t percent){
    int16_t corrected = percent + SERVO_OFFSET_PERCENT;

    if (corrected < 0)   corrected = 0;
    if (corrected > 100) corrected = 100;

    uint16_t min = hservo->min_pulse_ticks;
    uint16_t max = hservo->max_pulse_ticks;
    return min + ((max - min) * corrected) / 100;
}

/**
 * @brief  Commande le servo via un pourcentage (0 à 100%).
 * @param  hservo  Pointeur vers le handle du servo.
 * @param  percent Position cible en pourcentage.
 */
void servo_pwm_percent(Servo_Handle_t *hservo, uint8_t percent){
    uint16_t value = servo_map_percent(hservo, percent);
    pwm_pulse(hservo, value);
}

/**
 * @brief  Commande le servo via un angle en degrés.
 * @details Applique un bornage de sécurité (CLAMP_MIN / CLAMP_MAX) puis convertit
 * l'angle en pourcentage pour le PWM.
 * @param  hservo Pointeur vers le handle du servo.
 * @param  angle  Angle cible en degrés.
 */
void servo_pwm_angle_degree(Servo_Handle_t *hservo, int8_t angle){
    if (angle < SERVO_CLAMP_MIN) angle = SERVO_CLAMP_MIN;
    if (angle > SERVO_CLAMP_MAX) angle = SERVO_CLAMP_MAX;

    int16_t percent = ((angle + 35) * 100) / 70;
    uint16_t value = servo_map_percent(hservo, percent);

    pwm_pulse(hservo, value);
}

/**
 * @brief  Commande le servo via une valeur absolue haute résolution (0-65535).
 * @details Effectue une double conversion : Entrée -> Centi-degrés -> Ticks PWM.
 * Gère également l'offset (Trim) et les limites physiques du timer.
 * @param  hservo    Pointeur vers le handle du servo.
 * @param  abs_value Valeur absolue normalisée (0 à 65535).
 */
void servo_pwm_angle_abs_value(Servo_Handle_t *hservo, uint16_t abs_value){
    int32_t angle_centi = map(abs_value, 0, 65535, -4500, 4500);

    if (angle_centi < -2000) angle_centi = -2000;
    if (angle_centi >  2000) angle_centi =  2000;

    int32_t pwm_ticks = map(angle_centi, -3500, 3500, hservo->min_pulse_ticks, hservo->max_pulse_ticks);

    int32_t range = hservo->max_pulse_ticks - hservo->min_pulse_ticks;
    int32_t offset_ticks = (range * SERVO_OFFSET_PERCENT) / 100;

    pwm_ticks += offset_ticks;

    // Sécurité bornes hardware
    if (pwm_ticks > hservo->max_pulse_ticks) pwm_ticks = hservo->max_pulse_ticks;
    if (pwm_ticks < hservo->min_pulse_ticks) pwm_ticks = hservo->min_pulse_ticks;

    pwm_pulse(hservo, (uint16_t)pwm_ticks);
}

/**
 * @brief  Initialise le driver Servo.
 * @details Positionne le servo à 0 degrés (neutre) et active le canal PWM.
 * @param  hservo Pointeur vers le handle du servo.
 */
void servo_initialisation(Servo_Handle_t *hservo){
    if(hservo && hservo->htim){
        servo_pwm_angle_degree(hservo, 0);
        HAL_TIM_PWM_Start(hservo->htim, hservo->channel);
    }
}
