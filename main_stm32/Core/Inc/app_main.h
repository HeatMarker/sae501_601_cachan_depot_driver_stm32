/**
 * @file    app_main.h
 * @brief   En-tête de l'application principale.
 * @details Expose les points d'entrée (Setup/Loop) et les variables globales
 * partagées nécessaires à la gestion du temps et de la vitesse.
 */

#ifndef INC_APP_MAIN_H_
#define INC_APP_MAIN_H_

/**
 * @brief  Configure l'ensemble de l'application (Hardware + Drivers).
 */
void app_config(void);

/**
 * @brief  Exécute la boucle principale (Scheduler).
 */
void app_loop(void);

/** @brief Variable globale de vitesse (m/s) partagée entre le Speedometer et la Télémétrie. */
extern float speed_speedo_data;

/** @brief Compteur d'overflow pour l'extension 32-bits du Timer microseconde. */
extern volatile uint32_t tim3_overflow_cnt;

#endif
