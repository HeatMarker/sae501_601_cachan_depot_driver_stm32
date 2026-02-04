/**
 * @file    serial.h
 * @brief   Interface du driver Série (UART + DMA) et définitions du protocole.
 * @details Définit les tailles de buffers, les macros de manipulation du protocole
 * binaire 4 octets et les prototypes des fonctions d'E/S.
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---------- CONFIGURATION HARDWARE & BUFFERS ---------- */

/** @brief Taille du buffer circulaire logiciel de réception (Doit être une puissance de 2). */
#define SERIAL_RX_RING_SIZE   1024u

/** @brief Taille du buffer linéaire DMA pour la réception (Double buffer partiel). */
#define SERIAL_RX_CHUNK_SIZE  256u

/** @brief Taille maximale d'un transfert DMA unique en émission. */
#define SERIAL_TX_CHUNK_MAX   255u

/** @brief Instance UART HAL utilisée (liaison avec usart.h). */
#define SERIAL_UART           huart2

/* ---------- DÉFINITIONS DU PROTOCOLE (4 OCTETS) ---------- */
/* Format trame : [HDR | D0 | D1 | CRC8] */
/* HDR : bit7 = R(1)/W(0), bits6..0 = Adresse registre (0..127) */

/** @brief Masque pour extraire le bit R/W de l'entête. */
#define PROTO_HDR_RW_MASK     0x80u

/** @brief Masque pour extraire l'adresse du registre (0-127). */
#define PROTO_HDR_ADDR_MASK   0x7Fu

/** @brief Macro de construction de l'octet d'entête (Header). */
#define PROTO_MAKE_HDR(rw, addr)   ( (uint8_t)(((rw) ? 0x80u : 0x00u) | ((addr) & 0x7Fu)) )

/** @brief Macro vérifiant si l'entête correspond à une lecture. */
#define PROTO_IS_READ(hdr)         ( ((hdr) & 0x80u) != 0 )

/** @brief Macro extrayant l'adresse du registre depuis l'entête. */
#define PROTO_ADDR(hdr)            ( (hdr) & 0x7Fu )

/* ---------- FONCTIONS BAS NIVEAU (LINK LAYER) ---------- */

/**
 * @brief  Initialise la couche série (DMA + Buffers).
 */
void     serial_init(void);

/**
 * @brief  Écriture non-bloquante partielle.
 * @param  data Pointeur vers les données.
 * @param  len  Nombre d'octets souhaités.
 * @return Nombre d'octets écrits (peut être inférieur à len).
 */
int      serial_write_nb(const uint8_t *data, uint16_t len);

/**
 * @brief  Écriture non-bloquante atomique (Tout ou rien).
 * @param  data Pointeur vers les données.
 * @param  len  Nombre d'octets.
 * @return len si succès, code erreur négatif si buffer plein.
 */
int      serial_write_all_nb(const uint8_t *data, uint16_t len);

/**
 * @brief  Écriture bloquante (Wrapper).
 * @param  data Pointeur vers les données.
 * @param  len  Nombre d'octets.
 * @return 0 si succès, -1 si erreur.
 */
int      serial_write(const uint8_t *data, uint16_t len);

/**
 * @brief  Retourne le nombre d'octets disponibles dans le buffer RX.
 */
size_t   serial_available(void);

/**
 * @brief  Lit des données depuis le buffer RX.
 * @param  dst     Buffer de destination.
 * @param  max_len Nombre max d'octets à lire.
 * @return Nombre d'octets lus.
 */
size_t   serial_read(uint8_t *dst, size_t max_len);

/**
 * @brief  Lit des données jusqu'à un délimiteur spécifique.
 * @param  dst     Buffer de destination.
 * @param  max_len Taille max du buffer.
 * @param  delim   Caractère de fin (ex: '\\n').
 * @return Nombre d'octets lus.
 */
size_t   serial_read_until(uint8_t *dst, size_t max_len, uint8_t delim);

/* ---------- UTILITAIRES ---------- */

/**
 * @brief  Calcule le CRC8 (Polynôme ATM 0x07).
 * @param  data Pointeur vers les données.
 * @param  len  Longueur.
 * @return Checksum calculé.
 */
uint8_t  serial_crc8_atm(const uint8_t *data, uint16_t len);

/* ---------- FONCTIONS PROTOCOLE ---------- */

/**
 * @brief  Envoie une commande d'écriture 16 bits (Frame Write).
 * @details Trame : [HDR(0,addr) | VAL_LO | VAL_HI | CRC]
 * @param  addr  Adresse du registre.
 * @param  value Valeur à écrire.
 * @return Résultat de l'envoi série.
 */
int      proto_send_write16(uint8_t addr, int16_t value);

/**
 * @brief  Envoie une requête de lecture (Frame Read Request).
 * @details Trame : [HDR(1,addr) | COUNT | FLAGS | CRC]
 * @param  addr  Adresse de départ.
 * @param  count Nombre d'octets/registres à lire.
 * @param  flags Flags spécifiques.
 * @return Résultat de l'envoi série.
 */
int      proto_send_read_burst(uint8_t addr, uint8_t count, uint8_t flags);

/**
 * @brief  Envoie une donnée de télémétrie (Alias de Write).
 * @note   Utilisé par le STM32 pour répondre à une requête ou streamer des données.
 * @param  addr  Adresse du registre (ID de la donnée).
 * @param  value Valeur de la donnée.
 * @return Résultat de l'envoi série.
 */
static inline int proto_send_data16(uint8_t addr, int16_t value){
    return proto_send_write16(addr, value);
}

#endif
