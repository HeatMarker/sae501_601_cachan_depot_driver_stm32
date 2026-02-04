/**
 * @file    app_main.c
 * @brief   Point d'entrée principal de l'application (Main Loop & Scheduler).
 * @details Ce fichier contient la boucle principale, l'initialisation du système,
 * et l'ordonnanceur coopératif pour les tâches périodiques :
 * - Gestion Moteur (1kHz)
 * - Télémétrie (100Hz)
 * - Lecture Vitesse (10Hz)
 * - Traitement des commandes et Sécurité Failsafe.
 */

#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "app_main.h"
#include "driver_ins.h"
#include "bmi08_defs.h"
#include "bmi08.h"
#include "bmi08x.h"
#include "driver_servo.h"
#include "driver_motor.h"
#include "dma.h"
#include "serial.h"
#include "serial_cmd.h"
#include "driver_speedometer.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/** @brief Période d'exécution de la tâche moteur (1 ms). */
#define TASK_MOTOR_US       1000
/** @brief Période d'envoi de la télémétrie (10 ms). */
#define TASK_TELEMETRY_US   10000
/** @brief Période de calcul de la vitesse (100 ms). */
#define TASK_SPEED_US		100000
/** @brief Délai d'inactivité avant déclenchement du Failsafe (arrêt d'urgence). */
#define FAILSAFE_TIMEOUT_MS 500

/** @brief Durée d'impulsion pour 1ms (référence PWM). */
#define t_1_ms 3200
/** @brief Durée d'impulsion pour 2ms (référence PWM). */
#define t_2_ms 6400

/** @brief Borne PWM minimale pour l'ESC. */
#define PWM_MIN_ESC 3200
/** @brief Borne PWM maximale pour l'ESC. */
#define PWM_MAX_ESC 6400

/** @brief Instance du servomoteur de direction. */
static Servo_Handle_t hServo1 = {
    .htim = &htim1,           		   // Instance du Timer
    .channel = TIM_CHANNEL_1, 	       // Canal PWM
    .min_pulse_ticks = t_1_ms,         // Ticks pour 0% (ex: 1ms)
    .max_pulse_ticks = t_2_ms          // Ticks pour 100% (ex: 2ms)
};

/** @brief Instance du moteur de propulsion (ESC). */
static Motor_Handle_t hMotor1 = {
    .htim = &htim2,					   // Instance du Timer
    .channel = TIM_CHANNEL_1,		   // Canal PWM
    .min_pulse_ticks = PWM_MIN_ESC,	   //
    .max_pulse_ticks = PWM_MAX_ESC,    //
    .max_speed_pos_mms = 1000,         // Limit max frwd
    .max_speed_neg_mms = -500		   // Limit max bkwd
};

/** @brief Instance du capteur de vitesse (Tachymètre). */
Speedometer_Handle_t hSpeedo;

/** @brief Timestamp de la dernière commande valide reçue (pour le Failsafe). */
static uint32_t last_cmd_time_ms = 0;
/** @brief Timestamp de la dernière exécution de la tâche moteur. */
static uint32_t last_motor_us = 0;
/** @brief Timestamp de la dernière exécution de la tâche télémétrie. */
static uint32_t last_telemetry_us = 0;
/** @brief Timestamp de la dernière exécution de la tâche vitesse. */
static uint32_t last_speed_us = 0;

/** @brief Variable globale stockant la vitesse actuelle (partagée avec serial_cmd). */
float speed_speedo_data = 0.0f;

/** @brief Compteur de débordements pour le Timer 3 (Extension 16-bit vers 32-bit). */
volatile uint32_t tim3_overflow_cnt = 0;

static uint32_t GetMicrosTotal(void);
static void process_incoming_commands(void);
static void check_failsafe_security(void);
static void task_motor_update(uint32_t now_us);
static void task_telemetry_update(uint32_t now_us);
static void task_get_speed(uint32_t now_us);

/**
 * @brief  Récupère un temps système précis en microsecondes.
 * @details Utilise le Timer 3 (16 bits) combiné à un compteur d'overflow logiciel
 * pour générer un timestamp 32 bits continu.
 * @return Temps écoulé en microsecondes.
 */
static uint32_t GetMicrosTotal(void){
    uint32_t m_overflow;
    uint16_t m_counter;

    do{
        m_overflow = tim3_overflow_cnt;
        m_counter = LL_TIM_GetCounter(TIM3);
    }
    while(m_overflow != tim3_overflow_cnt);

    return (m_overflow << 16) + m_counter;
}

/**
 * @brief  Applique les commandes reçues via le port série.
 * @details Vérifie l'état du parseur (`parser_state`). Si une commande est prête,
 * met à jour le servo ou le moteur avec les valeurs "shadow".
 * Réinitialise le watchdog de sécurité (`last_cmd_time_ms`).
 */
static void process_incoming_commands(void){
    if(parser_state == PARSER_IDLE){
        return;
    }

    last_cmd_time_ms = HAL_GetTick();

    switch(parser_state){
        case PARSER_SERVO_CMD:
            servo_pwm_angle_degree(&hServo1, shadow_servo_cmd);
            break;

        case PARSER_MOTOR_CMD:
            motor_set_speed_mms(&hMotor1, shadow_motor_cmd);
            break;

        default:
            break;
    }

    parser_state = PARSER_IDLE;
}

/**
 * @brief  Sécurité active (Dead Man's Switch).
 * @details Vérifie si le temps écoulé depuis la dernière commande valide dépasse
 * `FAILSAFE_TIMEOUT_MS`. Si oui, coupe le moteur.
 */
static void check_failsafe_security(void){
    if((HAL_GetTick() - last_cmd_time_ms) > FAILSAFE_TIMEOUT_MS){
        motor_set_speed_mms(&hMotor1, 0);
    }
}

/**
 * @brief  Tâche périodique : Mise à jour du Moteur.
 * @details Appelle la machine à états du moteur toutes les 1 ms.
 * @param  now_us Timestamp actuel en microsecondes.
 */
static void task_motor_update(uint32_t now_us){
    if((uint32_t)(now_us - last_motor_us) >= TASK_MOTOR_US){
        last_motor_us = now_us;
        motor_process_1ms(&hMotor1, HAL_GetTick());
    }
}

/**
 * @brief  Tâche périodique : Envoi de la Télémétrie.
 * @details Déclenche l'envoi de la trame IMU+Vitesse toutes les 10 ms.
 * @param  now_us Timestamp actuel en microsecondes.
 */
static void task_telemetry_update(uint32_t now_us){
    if((uint32_t)(now_us - last_telemetry_us) >= TASK_TELEMETRY_US){
        last_telemetry_us = now_us;
        serial_send_data_frame();
    }
}

/**
 * @brief  Tâche périodique : Calcul de la vitesse.
 * @details Met à jour la variable globale de vitesse toutes les 100 ms.
 * @param  now_us Timestamp actuel en microsecondes.
 */
static void task_get_speed(uint32_t now_us){
    if((uint32_t)(now_us - last_speed_us) >= TASK_SPEED_US){
        last_speed_us = now_us;
        speed_speedo_data = speedometer_solve_speed(&hSpeedo);
    }
}

/**
 * @brief  Initialisation générale de l'application.
 * @details Configure les Timers, active les interruptions nécessaires,
 * initialise les drivers (Série, BMI088, Servo, Moteur, Speedo) et cale les horloges.
 */
void app_config(void){
	LL_TIM_EnableCounter(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3);
	HAL_TIM_Base_Start(&htim4);

	serial_init();
	BMI088_Init(&hspi1);
	servo_initialisation(&hServo1);
	motor_init(&hMotor1);
	motor_pwm_percent(&hMotor1, 50);

	last_cmd_time_ms  = HAL_GetTick();
	last_motor_us     = GetMicrosTotal();
	last_telemetry_us = GetMicrosTotal();
	last_speed_us     = GetMicrosTotal();

	speedometer_init(&hSpeedo, &htim4);
}

/**
 * @brief  Boucle principale de l'application (Super Loop).
 * @details Exécute séquentiellement :
 * 1. La lecture des données série (Polling).
 * 2. Le traitement des commandes (si disponibles).
 * 3. La vérification de sécurité.
 * 4. L'ordonnancement des tâches périodiques basées sur `now_us`.
 * @note   Le compteur `now_us` boucle après environ 71 minutes.
 */
void app_loop(void){
	uint32_t now_us = GetMicrosTotal(); //70 minutes max

    serial_cmd_reader();

    process_incoming_commands();
    check_failsafe_security();

    task_motor_update(now_us);
    task_get_speed(now_us);
    task_telemetry_update(now_us);
}
