/**
 * @file    driver_ins.h
 * @brief   Définitions et prototypes pour le pilote BMI088.
 * @details Contient les macros de configuration matérielle, les constantes de
 * conversion physique et les structures de données publiques.
 */

#ifndef BMI088_DRIVER_H
#define BMI088_DRIVER_H

#include "stm32g0xx_hal.h"
#include "bmi08_defs.h"
#include "bmi08.h"
#include "bmi08x.h"

/** @brief Port GPIO du Chip Select Accéléromètre (PA4/PB13 selon config). */
#define BMI088_CS_ACC_GPIO_Port     GPIOB
/** @brief Pin GPIO du Chip Select Accéléromètre. */
#define BMI088_CS_ACC_Pin           GPIO_PIN_13
/** @brief Port GPIO du Chip Select Gyroscope (PA1/PB14 selon config). */
#define BMI088_CS_GYRO_GPIO_Port    GPIOB
/** @brief Pin GPIO du Chip Select Gyroscope. */
#define BMI088_CS_GYRO_Pin          GPIO_PIN_14

/** @brief Facteur d'échelle LSB/g pour la gamme +/- 3g. */
#define ACCEL_RANGE_3G_LSB 			10922.67f
/** @brief Facteur d'échelle LSB/g pour la gamme +/- 6g. */
#define ACCEL_RANGE_6G_LSB          5461.33f
/** @brief Facteur d'échelle LSB/g pour la gamme +/- 12g. */
#define ACCEL_RANGE_12G_LSB 		2730.67f
/** @brief Facteur d'échelle LSB/g pour la gamme +/- 24g. */
#define ACCEL_RANGE_24G_LSB 		1365.33f
/** @brief Constante de conversion de gravité g vers mm/s². */
#define G_TO_MM_S2                  9806.65f

/** @brief Facteur d'échelle LSB/dps pour la gamme +/- 125 dps. */
#define GYRO_RANGE_125DPS_LSB 		262.4f
/** @brief Facteur d'échelle LSB/dps pour la gamme +/- 250 dps. */
#define GYRO_RANGE_250DPS_LSB 		131.2f
/** @brief Facteur d'échelle LSB/dps pour la gamme +/- 500 dps. */
#define GYRO_RANGE_500DPS_LSB 		65.6f
/** @brief Facteur d'échelle LSB/dps pour la gamme +/- 1000 dps. */
#define GYRO_RANGE_1000DPS_LSB      32.768f
/** @brief Facteur d'échelle LSB/dps pour la gamme +/- 2000 dps. */
#define GYRO_RANGE_2000DPS_LSB 		16.4f
/** @brief Constante de conversion degrés vers radians. */
#define DEG_TO_RAD                  0.017453292519943295f

/**
 * @brief Structure de définition d'un Chip Select SPI.
 */
typedef struct {
    GPIO_TypeDef *port;     ///< Port GPIO STM32.
    uint16_t pin;           ///< Numéro de Pin GPIO.
} bmi088_cs_t;

/**
 * @brief Structure de données physiques unifiées pour l'application.
 */
typedef struct {
    float accel_x_mms2;     ///< Accélération X (mm/s²).
    float accel_y_mms2;     ///< Accélération Y (mm/s²).
    float accel_z_mms2;     ///< Accélération Z (mm/s²).

    float gyro_x_rads;      ///< Vitesse angulaire X (rad/s).
    float gyro_y_rads;      ///< Vitesse angulaire Y (rad/s).
    float gyro_z_rads;      ///< Vitesse angulaire Z (rad/s).

    uint32_t timestamp_ms;  ///< Date de la mesure (ms depuis le démarrage).
} bmi088_data_t;

/**
 * @brief  Initialise le driver BMI088.
 * @param  hspi Pointeur vers le handle SPI utilisé.
 * @return BMI08_OK ou code d'erreur.
 */
int8_t BMI088_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief  Lit les registres bruts de l'accéléromètre.
 * @param  accel_data Structure de sortie pour les données brutes.
 * @return BMI08_OK ou code d'erreur.
 */
int8_t BMI088_Read_Accel_Raw(struct bmi08_sensor_data *accel_data);

/**
 * @brief  Lit les registres bruts du gyroscope.
 * @param  gyro_data Structure de sortie pour les données brutes.
 * @return BMI08_OK ou code d'erreur.
 */
int8_t BMI088_Read_Gyro_Raw(struct bmi08_sensor_data *gyro_data);

/**
 * @brief  Effectue une lecture complète et convertit les données.
 * @param  data Structure de sortie pour les données physiques.
 * @return BMI08_OK ou code d'erreur.
 */
int8_t BMI088_Read_All(bmi088_data_t *data);

/**
 * @brief  Convertit un jeu de données brutes accéléromètre en mm/s².
 * @param  accel_raw Données d'entrée brutes.
 * @param  accel_mms2 Tableau de sortie [X, Y, Z].
 */
void BMI088_Convert_Accel(struct bmi08_sensor_data *accel_raw, float *accel_mms2);

/**
 * @brief  Convertit un jeu de données brutes gyroscope en rad/s.
 * @param  gyro_raw Données d'entrée brutes.
 * @param  gyro_rads Tableau de sortie [X, Y, Z].
 */
void BMI088_Convert_Gyro(struct bmi08_sensor_data *gyro_raw, float *gyro_rads);

/**
 * @brief  Vérifie la présence des capteurs sur le bus SPI.
 * @param  print_result Active l'affichage debug via printf.
 * @return 1 (Succès) ou 0 (Échec).
 */
uint8_t BMI088_Test_Communication(uint8_t print_result);

/**
 * @brief  Redémarre les capteurs via commande logicielle.
 * @return BMI08_OK ou code d'erreur.
 */
int8_t BMI088_Soft_Reset(void);

#endif /* BMI088_DRIVER_H */
