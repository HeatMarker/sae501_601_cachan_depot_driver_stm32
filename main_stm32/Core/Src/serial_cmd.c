/**
 * @file    serial_cmd.c
 * @brief   Gestionnaire de commandes et télémétrie (Protocole Application).
 * @details Ce fichier gère :
 * 1. Le parsing des commandes entrantes (Pilotage Moteur/Servo) via une machine à états.
 * 2. L'envoi périodique de la télémétrie (IMU + Vitesse) vers l'ordinateur de bord (Pi5).
 */

#include "serial_cmd.h"
#include "serial.h"
#include "driver_motor.h"
#include "driver_servo.h"
#include "driver_ins.h"
#include "app_main.h"
#include <string.h>

/** @brief État courant du parseur, exposé à l'application principale pour déclencher les actions. */
ParserSwitch parser_state = PARSER_IDLE;

/**
 * @brief États de la machine à états de réception (Protocole 4 octets).
 */
typedef enum{
    S_HDR=0,    ///< Attente de l'entête (Header).
    S_D0,       ///< Attente de l'octet de donnée 0 (LSB).
    S_D1,       ///< Attente de l'octet de donnée 1 (MSB).
    S_CRC       ///< Attente du CRC8 de validation.
} ParseState;

/** @brief État interne courant de la machine à états. */
static ParseState st=S_HDR;
/** @brief Buffer temporaire pour l'entête. */
static uint8_t hdr = 0;
/** @brief Buffer temporaire pour la donnée 0. */
static uint8_t d0  = 0;
/** @brief Buffer temporaire pour la donnée 1. */
static uint8_t d1  = 0;

/** @brief Copie locale de la dernière commande servo reçue. */
int8_t  shadow_servo_cmd = 0;
/** @brief Copie locale de la dernière commande moteur reçue. */
int16_t shadow_motor_cmd = 0;

/** @brief Reconstruit un uint16_t à partir de deux octets. */
static inline uint16_t to_u16(uint8_t lo,uint8_t hi){return(uint16_t)lo|((uint16_t)hi<<8);}
/** @brief Reconstruit un int16_t à partir de deux octets. */
static inline int16_t to_i16(uint8_t lo,uint8_t hi){return(int16_t)to_u16(lo,hi);}

/**
 * @brief  Lit la valeur d'un registre virtuel.
 * @note   Utilisé pour répondre aux requêtes de lecture du protocole.
 * @param  addr Adresse du registre à lire.
 * @return Valeur actuelle du registre (commande shadow).
 */
static int16_t read_reg16(uint8_t addr){
    switch(addr){
        case REG_SERVO_CMD:return shadow_servo_cmd;
        case REG_MOTOR_CMD:return shadow_motor_cmd;
        case REG_BMI:return 0;
        default:return 0;
    }
}

/**
 * @brief  Traite une trame complète et validée par CRC.
 * @details Identifie si c'est une lecture ou une écriture.
 * - Écriture : Met à jour les variables shadow et l'état global du parseur.
 * - Lecture : Envoie immédiatement la réponse sur le port série.
 * @param  hdr_b Octet d'entête reçu.
 * @param  d0_b  Octet de donnée 0 reçu.
 * @param  d1_b  Octet de donnée 1 reçu.
 */
static void handle_frame(uint8_t hdr_b,uint8_t d0_b,uint8_t d1_b){
    const uint8_t  is_read = PROTO_IS_READ(hdr_b)?1u:0u;
    const uint8_t  addr    = PROTO_ADDR(hdr_b);
    const uint16_t udata16 = to_u16(d0_b,d1_b);
    const int16_t  data16  = to_i16(d0_b,d1_b);

    (void)udata16;

    if(is_read){
        uint8_t count=d0_b;
        for(uint8_t i=0; i<count; i++){
            uint8_t a = (uint8_t)((addr+i)&PROTO_HDR_ADDR_MASK);
            int16_t v = read_reg16(a);
            (void)proto_send_data16(a,v);
        }
        return;
    }

    switch(addr){
        case REG_SERVO_CMD:
            parser_state = PARSER_SERVO_CMD;
            shadow_servo_cmd = data16;
        break;

        case REG_MOTOR_CMD:
            parser_state = PARSER_MOTOR_CMD;
            shadow_motor_cmd = data16;
        break;

        case REG_BMI:
            parser_state = PARSER_BMI_CMD;
        break;

        default:
            parser_state = PARSER_OTHERS;
        break;
    }
}

/**
 * @brief  Injecte un octet dans la machine à états de réception.
 * @note   Vérifie le CRC à la fin de la séquence. Si OK, appelle handle_frame.
 * @param  b Octet entrant.
 */
static void parse_byte(uint8_t b){
    switch(st){
        case S_HDR:hdr=b;st=S_D0;break;
        case S_D0:d0=b;st=S_D1;break;
        case S_D1:d1=b;st=S_CRC;break;
        case S_CRC:{
            uint8_t buf[3]={hdr,d0,d1};
            uint8_t crc=serial_crc8_atm(buf,3);
            if(crc==b)handle_frame(hdr,d0,d1);
            st=S_HDR;
        }
        break;
        default:st=S_HDR;break;
    }
}

/**
 * @brief  Fonction principale de lecture (Polling).
 * @details Récupère les données brutes du buffer circulaire RX et les passe
 * octet par octet à la machine à états.
 */
void serial_cmd_reader(void){
    uint8_t tmp[64];
    size_t n = serial_read(tmp,sizeof(tmp));
    if(!n)return;
    for(size_t i=0;i<n;i++){
        parse_byte(tmp[i]);
    }
}

/**
 * @brief  Construit et envoie la trame de télémétrie complète.
 * @details
 * 1. Lit l'IMU (Accéléromètre + Gyroscope).
 * 2. Récupère la vitesse (Speedometer).
 * 3. Déduit le signe de la vitesse grâce à la commande moteur (Marche AR).
 * 4. Formate le paquet binaire (SerialImuFrame_t) avec CRC.
 * 5. Envoie le tout de manière non-bloquante via DMA.
 */
void serial_send_data_frame(void) {
    bmi088_data_t imu_data;

    if (BMI088_Read_All(&imu_data) != BMI08_OK) {
        return;
    }

    static SerialImuFrame_t frame;

    frame.head1 = 0xAA;
    frame.head2 = 0x55;
    frame.type  = 0x01;
    /* payload: timestamp(4) + accel(12) + gyro(12) + speed(4) = 32 */
    frame.len   = 32;
    frame.timestamp = HAL_GetTick();

    frame.accel[0] = imu_data.accel_x_mms2;
    frame.accel[1] = imu_data.accel_y_mms2;
    frame.accel[2] = imu_data.accel_z_mms2;

    frame.gyro[0]  = imu_data.gyro_x_rads;
    frame.gyro[1]  = imu_data.gyro_y_rads;
    frame.gyro[2]  = imu_data.gyro_z_rads;

    frame.speed = speed_speedo_data;
    frame.speed = (shadow_motor_cmd < 0) ? frame.speed * -1 : frame.speed; // Prise en compte de la commande pour le sens de rotation

    uint8_t *raw_bytes = (uint8_t*)&frame;

    frame.crc = serial_crc8_atm(raw_bytes, sizeof(SerialImuFrame_t) - 1);

    serial_write_all_nb(raw_bytes, sizeof(SerialImuFrame_t));
}
