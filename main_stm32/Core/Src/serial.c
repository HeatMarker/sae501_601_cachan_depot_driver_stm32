/**
 * @file    serial.c
 * @brief   Gestionnaire de communication Série (UART) avec DMA et Buffers Circulaires.
 * @details Ce fichier implémente un driver UART haute performance asynchrone.
 * - Réception : Utilise un buffer DMA linéaire couplé à un buffer circulaire logiciel.
 * - Transmission : Utilise un buffer circulaire logiciel vidé par DMA.
 * - Supporte les fonctions standard stdio (_write) pour printf.
 */

#include "serial.h"
#include "usart.h"
#include "dma.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>

/** @brief Buffer temporaire pour la réception DMA brute (Linear buffer). */
static uint8_t rx_chunk[SERIAL_RX_CHUNK_SIZE];

/** @brief Buffer circulaire logiciel pour la réception (Ring buffer). */
static uint8_t rx_ring[SERIAL_RX_RING_SIZE];

/** @brief Index de tête (écriture) du buffer circulaire RX. */
static volatile uint32_t rx_head=0;

/** @brief Index de queue (lecture) du buffer circulaire RX. */
static volatile uint32_t rx_tail=0;

/** @brief Masque pour le calcul modulo du buffer RX (taille doit être puissance de 2). */
#define RING_MASK (SERIAL_RX_RING_SIZE-1u)

#if (SERIAL_RX_RING_SIZE&(SERIAL_RX_RING_SIZE-1u))
#error "SERIAL_RX_RING_SIZE must be a power of two"
#endif

/**
 * @brief  Ajoute un octet dans le buffer circulaire de réception.
 * @note   Fonction interne. Écrase les données si le buffer est plein (Overrun).
 * @param  b Octet à ajouter.
 */
static inline void ring_push(uint8_t b){
    rx_ring[rx_head]=b;
    rx_head=(rx_head+1u)&RING_MASK;
    if(rx_head==rx_tail)rx_tail=(rx_tail+1u)&RING_MASK;
}

/**
 * @brief  Retourne le nombre d'octets disponibles en lecture dans le buffer RX.
 * @return Nombre d'octets.
 */
static inline uint32_t ring_count(void){return (rx_head-rx_tail)&RING_MASK;}

/** @brief Taille du buffer circulaire d'émission. */
#define TX_RING_SIZE 1024u

/** @brief Masque pour le calcul modulo du buffer TX. */
#define TX_RING_MASK (TX_RING_SIZE-1u)

#if (TX_RING_SIZE&(TX_RING_SIZE-1u))
#error "TX_RING_SIZE must be a power of two"
#endif

/** @brief Buffer circulaire pour la transmission. */
static uint8_t tx_ring[TX_RING_SIZE];

/** @brief Index de tête (écriture utilisateur) du buffer TX. */
static volatile uint32_t tx_head=0;

/** @brief Index de queue (lecture DMA) du buffer TX. */
static volatile uint32_t tx_tail=0;

/** @brief Indicateur d'activité du canal DMA TX. */
static volatile uint8_t tx_busy=0;

/**
 * @brief  Retourne le nombre d'octets en attente d'émission dans le buffer TX.
 * @return Nombre d'octets occupés.
 */
static inline uint32_t tx_count(void){return (tx_head-tx_tail)&TX_RING_MASK;}

/**
 * @brief  Retourne l'espace libre disponible dans le buffer TX.
 * @return Nombre d'octets libres.
 */
static inline uint32_t tx_space(void){return TX_RING_MASK-tx_count();}

/**
 * @brief  Déclenche le transfert DMA pour l'émission si nécessaire.
 * @details Cette fonction vérifie si le DMA est libre et s'il y a des données à envoyer.
 * Elle gère le découpage mémoire si les données bouclent à la fin du buffer (wrap-around).
 * Section critique protégée par désactivation des interruptions.
 */
static void serial_kick_tx(void){
    __disable_irq();
    if(tx_busy){
        __enable_irq();
        return;
    }
    uint32_t head=tx_head, tail=tx_tail;
    if(head==tail){
        __enable_irq();
        return;
    }

    uint32_t linear=(head>=tail)?(head-tail):(TX_RING_SIZE-tail);
    uint16_t chunk=(linear>SERIAL_TX_CHUNK_MAX)?SERIAL_TX_CHUNK_MAX:(uint16_t)linear;

    tx_busy=1;

    if(HAL_UART_Transmit_DMA(&SERIAL_UART, &tx_ring[tail], chunk) != HAL_OK){
        tx_busy=0;
    }

    __enable_irq();
}

/**
 * @brief  Initialise le driver Série.
 * @details Lance la réception DMA en mode "ReceiveToIdle" pour détecter les fins de trames
 * sans attendre que le buffer soit plein.
 */
void serial_init(void){
    HAL_UARTEx_ReceiveToIdle_DMA(&SERIAL_UART,rx_chunk,SERIAL_RX_CHUNK_SIZE);
    __HAL_DMA_DISABLE_IT(SERIAL_UART.hdmarx,DMA_IT_HT);
}

/**
 * @brief  Écrit des données dans le buffer d'émission (Non-bloquant partiel).
 * @note   Copie autant de données que possible. S'arrête si le buffer est plein.
 * @param  data Pointeur vers les données.
 * @param  len  Nombre d'octets à écrire.
 * @return Nombre d'octets réellement écrits ou code d'erreur négatif.
 */
int serial_write_nb(const uint8_t *data,uint16_t len){
    uint16_t written=0;
    while(written<len){
        uint32_t space=tx_space();
        if(space==0)return(written>0)?(int)written:-EWOULDBLOCK;
        uint32_t head=tx_head;
        uint32_t room_linear=(head>=tx_tail)?(TX_RING_SIZE-head-((tx_tail==0)?1u:0u)):(tx_tail-head-1u);
        if(room_linear==0)return(written>0)?(int)written:-EWOULDBLOCK;
        uint32_t to_copy=len-written;
        if(to_copy>room_linear)to_copy=room_linear;
        if(to_copy>space)to_copy=space;
        memcpy(&tx_ring[head],&data[written],to_copy);
        tx_head=(head+to_copy)&TX_RING_MASK;
        written+=(uint16_t)to_copy;
    }
    serial_kick_tx();
    return(int)written;
}

/**
 * @brief  Écrit des données seulement si tout le bloc peut entrer (Atomique).
 * @param  data Pointeur vers les données.
 * @param  len  Nombre d'octets.
 * @return len si succès, -EWOULDBLOCK si pas assez d'espace.
 */
int serial_write_all_nb(const uint8_t *data,uint16_t len){
    if(tx_space()<len)return -EWOULDBLOCK;
    uint32_t head=tx_head;
    uint32_t first=(head>=tx_tail)?(TX_RING_SIZE-head-((tx_tail==0)?1u:0u)):(tx_tail-head-1u);
    if(first>len)first=len;
    memcpy(&tx_ring[head],data,first);
    tx_head=(head+first)&TX_RING_MASK;
    uint16_t remain=len-(uint16_t)first;
    if(remain){
        memcpy(&tx_ring[tx_head],data+first,remain);
        tx_head=(tx_head+remain)&TX_RING_MASK;
    }
    serial_kick_tx();
    return(int)len;
}

/**
 * @brief  Wrapper bloquant pour l'écriture (simulé).
 * @note   En réalité non-bloquant ici, renvoie -1 en cas d'échec.
 * @param  data Pointeur vers les données.
 * @param  len  Longueur.
 * @return 0 si succès, -1 si erreur.
 */
int serial_write(const uint8_t *data,uint16_t len){
    int r=serial_write_all_nb(data,len);
    return(r>=0)?0:-1;
}

/**
 * @brief  Fonction système bas niveau pour rediriger printf().
 * @param  file Descripteur de fichier (ignoré).
 * @param  ptr  Données à écrire.
 * @param  len  Longueur.
 * @return Nombre d'octets écrits ou -1.
 */
int _write(int file,char *ptr,int len){
    (void)file;
    int r=serial_write_all_nb((const uint8_t*)ptr,(uint16_t)len);
    return(r>=0)?r:-1;
}

/**
 * @brief  Vérifie le nombre d'octets disponibles à la lecture.
 * @return Nombre d'octets dans le buffer RX.
 */
size_t serial_available(void){return ring_count();}

/**
 * @brief  Lit des données depuis le buffer RX.
 * @details Copie les données du buffer circulaire interne vers le buffer utilisateur.
 * Gère le cas où les données sont à cheval sur la fin du buffer (wrap).
 * @param  dst     Buffer de destination.
 * @param  max_len Nombre maximum d'octets à lire.
 * @return Nombre d'octets réellement lus.
 */
size_t serial_read(uint8_t *dst, size_t max_len){
    __disable_irq();

    size_t avail = ring_count();
    if(!avail || !max_len){
        __enable_irq();
        return 0;
    }

    size_t to_copy = (avail < max_len) ? avail : max_len;
    size_t first = to_copy;

    if(rx_tail + first > SERIAL_RX_RING_SIZE) {
        first = SERIAL_RX_RING_SIZE - rx_tail;
    }

    memcpy(dst, &rx_ring[rx_tail], first);
    if(first < to_copy) {
        memcpy(dst + first, &rx_ring[0], to_copy - first);
    }

    rx_tail = (rx_tail + to_copy) & RING_MASK;

    __enable_irq();

    return to_copy;
}

/**
 * @brief  Lit des données jusqu'à rencontrer un délimiteur.
 * @note   Fonction utile pour lire des lignes complètes (ex: jusqu'à '\\n').
 * @param  dst     Buffer de destination.
 * @param  max_len Taille max du buffer destination.
 * @param  delim   Caractère délimiteur recherché.
 * @return Nombre d'octets lus (incluant le délimiteur), ou 0 si non trouvé.
 */
size_t serial_read_until(uint8_t *dst,size_t max_len,uint8_t delim){
    if(!max_len)return 0;
    uint32_t head=rx_head,tail=rx_tail;
    if(head==tail)return 0;
    uint32_t i=tail;
    while(i!=head){
        if(rx_ring[i]==delim){
            size_t msg_len=(i>=tail)?(i-tail+1u):(SERIAL_RX_RING_SIZE-tail+i+1u);
            if(msg_len>max_len)return 0;
            size_t first=msg_len;
            if(tail+first>SERIAL_RX_RING_SIZE)first=SERIAL_RX_RING_SIZE-tail;
            memcpy(dst,&rx_ring[tail],first);
            if(first<msg_len)memcpy(dst+first,&rx_ring[0],msg_len-first);
            rx_tail=(tail+msg_len)&RING_MASK;
            return msg_len;
        }
        i=(i+1u)&RING_MASK;
    }
    return 0;
}

/**
 * @brief  Callback HAL appelé quand un transfert DMA TX est terminé.
 * @details Met à jour l'index de queue TX et relance une transmission si
 * il reste des données dans le buffer.
 * @param  huart Handle UART concerné.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart!=&SERIAL_UART)return;
    uint16_t sent=huart->TxXferSize;
    tx_tail=(tx_tail+sent)&TX_RING_MASK;
    tx_busy=0;
    serial_kick_tx();
}

/**
 * @brief  Callback HAL appelé lors d'un événement RX (Idle Line ou Transfer Complete).
 * @details Transfère les données du buffer DMA linéaire (rx_chunk) vers le
 * buffer circulaire logiciel (rx_ring).
 * @note   Gère la position précédente pour ne copier que les nouveaux octets reçus.
 * @param  huart Handle UART concerné.
 * @param  Size  Nombre total d'octets reçus dans le buffer DMA.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if(huart != &SERIAL_UART) return;

    static uint16_t old_pos = 0;

    if(Size != old_pos){
        if(Size > old_pos){
            for(uint16_t k = old_pos; k < Size; k++){
                ring_push(rx_chunk[k]);
            }
        }
        else{
            for(uint16_t k = old_pos; k < SERIAL_RX_CHUNK_SIZE; k++){
                ring_push(rx_chunk[k]);
            }
            for(uint16_t k = 0; k < Size; k++){
                ring_push(rx_chunk[k]);
            }
        }
        old_pos = Size;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_chunk, SERIAL_RX_CHUNK_SIZE);

    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

/**
 * @brief  Calcule un CRC-8 (Polynôme ATM : 0x07).
 * @param  data Pointeur vers les données.
 * @param  len  Longueur des données.
 * @return Valeur du CRC calculé.
 */
uint8_t serial_crc8_atm(const uint8_t *data,uint16_t len){
    uint8_t crc=0x00u;
    for(uint16_t i=0;i<len;i++){
        crc^=data[i];
        for(uint8_t b=0;b<8;b++){
            crc=(crc&0x80u)?(uint8_t)((crc<<1)^0x07u):(uint8_t)(crc<<1);
        }
    }
    return crc;
}

/**
 * @brief  Envoie une trame brute de 3 octets + CRC (Format interne).
 * @param  hdr Octet d'entête.
 * @param  d0  Premier octet de données.
 * @param  d1  Second octet de données.
 * @return Résultat de l'écriture série.
 */
static int proto_send_frame3(uint8_t hdr,uint8_t d0,uint8_t d1){
    uint8_t buf[4];
    buf[0]=hdr;
    buf[1]=d0;
    buf[2]=d1;
    buf[3]=serial_crc8_atm(buf,3);
    return serial_write_all_nb(buf,4);
}

/**
 * @brief  Envoie une commande d'écriture 16 bits protocole.
 * @param  addr  Adresse du registre virtuel.
 * @param  value Valeur 16 bits à écrire.
 * @return Résultat de l'envoi.
 */
int proto_send_write16(uint8_t addr,int16_t value){
    uint8_t hdr=PROTO_MAKE_HDR(0,addr);
    uint16_t u=(uint16_t)value;
    return proto_send_frame3(hdr,(uint8_t)(u&0xFFu),(uint8_t)(u>>8));
}

/**
 * @brief  Envoie une demande de lecture multiple (Burst).
 * @param  addr  Adresse de départ.
 * @param  count Nombre de registres à lire.
 * @param  flags Flags optionnels.
 * @return Résultat de l'envoi.
 */
int proto_send_read_burst(uint8_t addr,uint8_t count,uint8_t flags){
    uint8_t hdr=PROTO_MAKE_HDR(1,addr);
    return proto_send_frame3(hdr,count,flags);
}
