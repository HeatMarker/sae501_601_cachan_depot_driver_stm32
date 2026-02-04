/**
 * @file    driver_ins.c
 * @brief   Implémentation du pilote pour l'IMU BMI088 (Accéléromètre + Gyroscope).
 * @details Gère l'initialisation, la communication SPI et la conversion des données
 * brutes en unités physiques via l'API Bosch SensorTec.
 */

#include "stm32g0xx_hal.h"
#include "driver_ins.h"
#include <stdio.h>
#include <string.h>

/** @brief Instance de la structure de périphérique Bosch BMI08. */
static struct bmi08_dev bmi088_dev;

/** @brief Pointeur vers le gestionnaire SPI STM32 (HAL). */
static SPI_HandleTypeDef *bmi088_hspi;

/** @brief Configuration du Chip Select pour l'accéléromètre. */
static bmi088_cs_t cs_accel = {
    .port = BMI088_CS_ACC_GPIO_Port,
    .pin  = BMI088_CS_ACC_Pin
};

/** @brief Configuration du Chip Select pour le gyroscope. */
static bmi088_cs_t cs_gyro = {
    .port = BMI088_CS_GYRO_GPIO_Port,
    .pin  = BMI088_CS_GYRO_Pin
};

/**
 * @brief  Fonction de lecture SPI bas niveau conforme à l'interface Bosch.
 * @note   Gère la différence de protocole entre l'accéléromètre (byte vide requis)
 * et le gyroscope (pas de byte vide).
 * @param  reg_addr Adresse du registre à lire.
 * @param  reg_data Pointeur vers le buffer de réception des données.
 * @param  len      Nombre d'octets à lire.
 * @param  intf_ptr Pointeur vers l'interface matérielle (struct bmi088_cs_t).
 * @return BMI08_OK en cas de succès, ou code d'erreur (ex: BMI08_E_COM_FAIL).
 */
static int8_t bmi088_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){
    if (reg_data == NULL || intf_ptr == NULL) return BMI08_E_NULL_PTR;
    if (len == 0 || len > BMI08_MAX_LEN) return BMI08_E_RD_WR_LENGTH_INVALID;

    bmi088_cs_t *cs = (bmi088_cs_t*)intf_ptr;
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(cs->port, cs->pin, GPIO_PIN_RESET);

    if(cs->pin == BMI088_CS_ACC_Pin){
        uint8_t tx_buf[256] = {0};
        uint8_t rx_buf[256] = {0};

        tx_buf[0] = reg_addr | 0x80;
        tx_buf[1] = 0x00;

        status = HAL_SPI_TransmitReceive(bmi088_hspi, tx_buf, rx_buf, len + 2, HAL_MAX_DELAY);

        /*
        for(uint32_t i = 0; i < len + 2; i++){
            printf("REG RX i : %d : %02X\r\n", i, rx_buf[i]);
        }
        */

        HAL_GPIO_WritePin(cs->port, cs->pin, GPIO_PIN_SET);

        if(status != HAL_OK) return BMI08_E_COM_FAIL;

        for(uint32_t i = 0; i < len; i++){
            reg_data[i] = rx_buf[i + 1];
        }
    }
    else{
        uint8_t tx_buf[256] = {0};
        uint8_t rx_buf[256] = {0};

        tx_buf[0] = reg_addr | 0x80;

        status = HAL_SPI_TransmitReceive(bmi088_hspi, tx_buf, rx_buf, len + 1, HAL_MAX_DELAY);

        HAL_GPIO_WritePin(cs->port, cs->pin, GPIO_PIN_SET);

        if(status != HAL_OK) return BMI08_E_COM_FAIL;

        for(uint32_t i = 0; i < len; i++){
            reg_data[i] = rx_buf[i + 1];
        }
    }

    return BMI08_OK;
}

/**
 * @brief  Fonction d'écriture SPI bas niveau conforme à l'interface Bosch.
 * @param  reg_addr Adresse du registre à écrire.
 * @param  reg_data Pointeur vers les données à écrire.
 * @param  len      Nombre d'octets à écrire.
 * @param  intf_ptr Pointeur vers l'interface matérielle (struct bmi088_cs_t).
 * @return BMI08_OK en cas de succès.
 */
static int8_t bmi088_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){
    bmi088_cs_t *cs = (bmi088_cs_t*)intf_ptr;

    HAL_GPIO_WritePin(cs->port, cs->pin, GPIO_PIN_RESET);

    uint8_t addr = reg_addr & 0x7F;
    HAL_SPI_Transmit(bmi088_hspi, &addr, 1, HAL_MAX_DELAY);

    HAL_SPI_Transmit(bmi088_hspi, (uint8_t*)reg_data, len, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(cs->port, cs->pin, GPIO_PIN_SET);

    return BMI08_OK;
}

/**
 * @brief  Fonction de délai microseconde pour l'interface Bosch.
 * @note   Utilise HAL_Delay (millisecondes), avec un minimum de 1ms.
 * @param  period   Durée du délai en microsecondes.
 * @param  intf_ptr Pointeur d'interface (inutilisé).
 */
static void bmi088_delay_us(uint32_t period, void *intf_ptr){
    uint32_t delay_ms = period / 1000;
    if(delay_ms == 0) delay_ms = 1;
    HAL_Delay(delay_ms);
}

/**
 * @brief  Initialise le module BMI088 (Accéléromètre et Gyroscope).
 * @param  hspi Pointeur vers le handle SPI STM32.
 * @return BMI08_OK si l'initialisation réussit, code d'erreur sinon.
 */
int8_t BMI088_Init(SPI_HandleTypeDef *hspi){
    if(hspi == NULL){
        return BMI08_E_NULL_PTR;
    }

    bmi088_hspi = hspi;

    HAL_GPIO_WritePin(BMI088_CS_ACC_GPIO_Port, BMI088_CS_ACC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_CS_GYRO_GPIO_Port, BMI088_CS_GYRO_Pin, GPIO_PIN_SET);

    bmi088_dev.intf = BMI08_SPI_INTF;
    bmi088_dev.read = bmi088_spi_read;
    bmi088_dev.write = bmi088_spi_write;
    bmi088_dev.delay_us = bmi088_delay_us;
    bmi088_dev.intf_ptr_accel = &cs_accel;
    bmi088_dev.intf_ptr_gyro = &cs_gyro;

    int8_t rslt_accel = bmi08a_init(&bmi088_dev);

#ifdef DEBUG
    //printf("BMI088 Init Accel: %d (0=OK)\r\n", rslt_accel);
    //printf("Accel Chip ID: 0x%02X (attendu: 0x1E)\r\n", bmi088_dev.accel_chip_id);
#endif

    int8_t rslt_gyro = bmi08g_init(&bmi088_dev);

#ifdef DEBUG
    //printf("BMI088 Init Gyro: %d (0=OK)\r\n", rslt_gyro);
    //printf("Gyro Chip ID: 0x%02X (attendu: 0x0F)\r\n", bmi088_dev.gyro_chip_id);
#endif

    if(rslt_accel != BMI08_OK || rslt_gyro != BMI08_OK){
        return BMI08_E_DEV_NOT_FOUND;
    }

    bmi088_dev.accel_cfg.odr   = BMI08_ACCEL_ODR_100_HZ;
    bmi088_dev.accel_cfg.range = BMI088_ACCEL_RANGE_6G;
    bmi088_dev.accel_cfg.bw    = BMI08_ACCEL_BW_NORMAL;
    bmi088_dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;

    rslt_accel  = bmi08a_set_power_mode(&bmi088_dev);
    rslt_accel |= bmi08a_set_meas_conf(&bmi088_dev);

    bmi088_dev.gyro_cfg.odr   = BMI08_GYRO_BW_23_ODR_200_HZ;
    bmi088_dev.gyro_cfg.range = BMI08_GYRO_RANGE_1000_DPS;
    bmi088_dev.gyro_cfg.bw    = BMI08_GYRO_BW_23_ODR_200_HZ;
    bmi088_dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

    rslt_gyro  = bmi08g_set_power_mode(&bmi088_dev);
    rslt_gyro |= bmi08g_set_meas_conf(&bmi088_dev);

    if(rslt_accel != BMI08_OK || rslt_gyro != BMI08_OK){
        return BMI08_E_COM_FAIL;
    }

#ifdef DEBUG
    //printf("BMI088 initialisé avec succès!\r\n");
#endif

    return BMI08_OK;
}

/**
 * @brief  Lit les données brutes de l'accéléromètre.
 * @param  accel_data Pointeur vers la structure de destination des données brutes.
 * @return BMI08_OK en cas de succès, ou code d'erreur.
 */
int8_t BMI088_Read_Accel_Raw(struct bmi08_sensor_data *accel_data){
    if(accel_data == NULL){
        return BMI08_E_NULL_PTR;
    }

    return bmi08a_get_data(accel_data, &bmi088_dev);
}

/**
 * @brief  Lit les données brutes du gyroscope.
 * @param  gyro_data Pointeur vers la structure de destination des données brutes.
 * @return BMI08_OK en cas de succès, ou code d'erreur.
 */
int8_t BMI088_Read_Gyro_Raw(struct bmi08_sensor_data *gyro_data){
    if(gyro_data == NULL){
        return BMI08_E_NULL_PTR;
    }

    return bmi08g_get_data(gyro_data, &bmi088_dev);
}

/**
 * @brief  Lit et convertit l'ensemble des données IMU (Accel + Gyro).
 * @param  data Pointeur vers la structure de données utilisateur (unités physiques).
 * @return BMI08_OK en cas de succès, ou code d'erreur.
 */
int8_t BMI088_Read_All(bmi088_data_t *data){
    if(data == NULL){
        return BMI08_E_NULL_PTR;
    }

    struct bmi08_sensor_data accel_raw, gyro_raw;

    int8_t rslt = BMI088_Read_Accel_Raw(&accel_raw);
    if(rslt != BMI08_OK){
        return rslt;
    }

    rslt = BMI088_Read_Gyro_Raw(&gyro_raw);
    if(rslt != BMI08_OK){
        return rslt;
    }

    data->accel_x_mms2 = ((float)accel_raw.x / ACCEL_RANGE_6G_LSB) * G_TO_MM_S2;
    data->accel_y_mms2 = ((float)accel_raw.y / ACCEL_RANGE_6G_LSB) * G_TO_MM_S2;
    data->accel_z_mms2 = ((float)accel_raw.z / ACCEL_RANGE_6G_LSB) * G_TO_MM_S2;

    data->gyro_x_rads = ((float)gyro_raw.x / GYRO_RANGE_1000DPS_LSB) * DEG_TO_RAD;
    data->gyro_y_rads = ((float)gyro_raw.y / GYRO_RANGE_1000DPS_LSB) * DEG_TO_RAD;
    data->gyro_z_rads = ((float)gyro_raw.z / GYRO_RANGE_1000DPS_LSB) * DEG_TO_RAD;

    data->timestamp_ms = HAL_GetTick();

    return BMI08_OK;
}

/**
 * @brief  Convertit les données brutes d'accélération en unités physiques.
 * @param  accel_raw  Pointeur vers les données brutes d'entrée.
 * @param  accel_mms2 Pointeur vers le tableau de sortie (x, y, z) en mm/s².
 */
void BMI088_Convert_Accel(struct bmi08_sensor_data *accel_raw, float *accel_mms2){
    if(accel_raw == NULL || accel_mms2 == NULL){
        return;
    }

    accel_mms2[0] = ((float)accel_raw->x / ACCEL_RANGE_6G_LSB) * G_TO_MM_S2;
    accel_mms2[1] = ((float)accel_raw->y / ACCEL_RANGE_6G_LSB) * G_TO_MM_S2;
    accel_mms2[2] = ((float)accel_raw->z / ACCEL_RANGE_6G_LSB) * G_TO_MM_S2;
}

/**
 * @brief  Convertit les données brutes gyroscopiques en unités physiques.
 * @param  gyro_raw  Pointeur vers les données brutes d'entrée.
 * @param  gyro_rads Pointeur vers le tableau de sortie (x, y, z) en rad/s.
 */
void BMI088_Convert_Gyro(struct bmi08_sensor_data *gyro_raw, float *gyro_rads){
    if(gyro_raw == NULL || gyro_rads == NULL){
        return;
    }

    gyro_rads[0] = ((float)gyro_raw->x / GYRO_RANGE_1000DPS_LSB) * DEG_TO_RAD;
    gyro_rads[1] = ((float)gyro_raw->y / GYRO_RANGE_1000DPS_LSB) * DEG_TO_RAD;
    gyro_rads[2] = ((float)gyro_raw->z / GYRO_RANGE_1000DPS_LSB) * DEG_TO_RAD;
}

/**
 * @brief  Teste la communication SPI en vérifiant les IDs des puces.
 * @param  print_result Si non nul, affiche le résultat sur la console de debug.
 * @return 1 si la communication est établie avec succès, 0 sinon.
 */
uint8_t BMI088_Test_Communication(uint8_t print_result){
    uint8_t accel_ok = (bmi088_dev.accel_chip_id == BMI088_ACCEL_CHIP_ID);
    uint8_t gyro_ok = (bmi088_dev.gyro_chip_id == BMI08_GYRO_CHIP_ID);

    if(print_result){

    	/*
    	printf("=== BMI088 Communication Test ===\r\n");
        printf("Accelerometer: %s (ID: 0x%02X)\r\n",
               accel_ok ? "OK" : "FAIL", bmi088_dev.accel_chip_id);
        printf("Gyroscope: %s (ID: 0x%02X)\r\n",
               gyro_ok ? "OK" : "FAIL", bmi088_dev.gyro_chip_id);
        printf("================================\r\n");
    	*/
    }

    return (accel_ok && gyro_ok);
}

/**
 * @brief  Effectue une réinitialisation logicielle (Soft Reset) des capteurs.
 * @return BMI08_OK en cas de succès, ou code d'erreur SPI.
 */
int8_t BMI088_Soft_Reset(void){
    uint8_t soft_reset_cmd = BMI08_SOFT_RESET_CMD;
    int8_t rslt = bmi088_spi_write(BMI08_REG_ACCEL_SOFTRESET, &soft_reset_cmd, 1, &cs_accel);

    HAL_Delay(50);

    rslt |= bmi088_spi_write(BMI08_REG_GYRO_SOFTRESET, &soft_reset_cmd, 1, &cs_gyro);

    HAL_Delay(50);

    return rslt;
}
