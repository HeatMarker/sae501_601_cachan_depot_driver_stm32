/**
 * @file    serial_cmd.h
 * @brief   Définitions des commandes et structures du protocole applicatif.
 * @details Contient les adresses des registres virtuels, la définition de la trame
 * de télémétrie binaire et les états de notification du parseur.
 */

#ifndef INC_SERIAL_CMD_H_
#define INC_SERIAL_CMD_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/** @brief Adresse du registre virtuel pour la commande Servo (0-100%). */
#define REG_SERVO_CMD 0x00
/** @brief Adresse du registre virtuel pour la commande Moteur (mm/s). */
#define REG_MOTOR_CMD 0x01
/** @brief Adresse du registre virtuel pour les commandes BMI088 (réservé). */
#define REG_BMI       0x02

/**
 * @brief Structure de la trame de télémétrie envoyée vers la Pi5.
 * @note  Structure "packed" pour éviter le padding et garantir l'alignement binaire.
 * Format total : 4 (Header/Meta) + 4 (Time) + 12 (Accel) + 12 (Gyro) + 4 (Speed) + 1 (CRC) = 37 octets.
 */
typedef struct __attribute__((packed)) {
    uint8_t head1;      ///< Octet de synchronisation 1 (0xAA).
    uint8_t head2;      ///< Octet de synchronisation 2 (0x55).
    uint8_t type;       ///< Type de packet (0x01 pour Télémétrie).
    uint8_t len;        ///< Longueur du payload (32 octets).
    uint32_t timestamp; ///< Timestamp système (HAL_GetTick).
    float accel[3];     ///< Données Accéléromètre [X, Y, Z] en mm/s².
    float gyro[3];      ///< Données Gyroscope [X, Y, Z] en rad/s.
    float speed;		///< Vitesse linéaire du véhicule en m/s.
    uint8_t crc;        ///< Checksum CRC-8 pour validation de l'intégrité.
} SerialImuFrame_t;

/**
 * @brief Indicateur de résultat du parsing pour la boucle principale.
 */
typedef enum{
    PARSER_IDLE,        ///< Aucune nouvelle commande traitée.
    PARSER_SERVO_CMD,   ///< Une commande Servo a été reçue et validée.
    PARSER_MOTOR_CMD,   ///< Une commande Moteur a été reçue et validée.
    PARSER_BMI_CMD,     ///< Une commande BMI a été reçue.
    PARSER_OTHERS       ///< Une autre commande a été reçue.
} ParserSwitch;

/** @brief État global du parseur, lu par le main pour appliquer les consignes. */
extern ParserSwitch parser_state;

/** @brief Dernière consigne reçue pour le servo (Shadow Register). */
extern int8_t shadow_servo_cmd;

/** @brief Dernière consigne reçue pour le moteur (Shadow Register). */
extern int16_t shadow_motor_cmd;

/**
 * @brief  Fonction de lecture périodique des commandes entrantes.
 * @details Lit le buffer circulaire RX et alimente la machine à états.
 */
void serial_cmd_reader(void);

/**
 * @brief  Capture les capteurs et envoie la trame de télémétrie.
 * @details Lit l'IMU et le compteur de vitesse, remplit SerialImuFrame_t,
 * calcule le CRC et transmet via DMA.
 */
void serial_send_data_frame(void);

#endif
