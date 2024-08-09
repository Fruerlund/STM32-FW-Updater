#ifndef INC_COMMON_DEFINES_H
#define INC_COMMON_DEFINES_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <unistd.h>             //Used for UART
#include <fcntl.h>              //Used for UART
#include <termios.h>		//Used for UART
#include <stdbool.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <netdb.h>
#include <pthread.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <poll.h>
#include <sys/prctl.h>

#include <errno.h>

#define RX_BUFFER_LENGTH 18
#define PACKET_BUFFER_LENGTH    (8)    // 8 * 18 bytes
#define PACKET_DATA_LENGTH      (16)
#define PACKET_LENGTH_BYTES     (1)
#define PACKET_CRC_BYTES        (1)
#define PACKET_LENGTH           (PACKET_LENGTH_BYTES + PACKET_DATA_LENGTH + PACKET_CRC_BYTES)
#define PACKET_RETX_DATA0       (0x19)
#define PACKET_ACK_DATA0        (0x15)

// Firmware update
#define BL_PACKET_SYNC_OBSERVED_DATA0   (0x11)
#define BL_PACKET_FW_UPDATE_REQ_DATA0   (0x21)
#define BL_PACKET_FW_UPDATE_RES_DATA0   (0x31)
#define BL_PACKET_DEVICE_ID_REQ_DATA0   (0x41)
#define BL_PACKET_DEVICE_ID_RES_DATA0   (0x51)
#define BL_PACKET_FW_LENGTH_REQ_DATA0   (0x61)
#define BL_PACKET_FW_LENGTH_RES_DATA0   (0x71)
#define BL_PACKET_READY_FOR_DATA_DATA0  (0x81)
#define BL_PACKET_UPDATE_OK             (0x91)
#define BL_PACKET_NACK_DATA0            (0x12)

// Targeted Device ID
#define DEVICE_ID                   (0x41)

#define DEFAULT_TIMEOUT             (5000)

// Four byte firmware update signature.
#define SYNC_SEQ_0                  (0xC1)
#define SYNC_SEQ_1                  (0xC2)
#define SYNC_SEQ_2                  (0xC3)
#define SYNC_SEQ_3                  (0xC4)


void     sigint_handler(int);

typedef enum bl_state_t {

    BL_State_SendSync,
    BL_State_WaitForSyncObservered,
    BL_State_SendFirmwareUpdateRequest,
    BL_State_WaitForFirmwareUpdateAccept,
    BL_State_WaitForDeviceIDRequest,
    BL_State_SendDeviceIDReply,
    BL_State_WaitForFirmwareLengthRequest,
    BL_State_SendFirmwareLengthReply,
    BL_State_WaitForReady,
    BL_State,WaitForReadyForData,
    BL_State_SendFirmware,
    BL_State_Done,

} bl_state_t;


// Will be placed directly after INTERRUPT VECTOR TABLE
typedef struct firmware_info_t {

    uint32_t sentinel;
    uint32_t deviceid;
    uint32_t version;
    uint32_t length;
    uint32_t reserved0;
    uint32_t reserved1;
    uint32_t reserved2;
    uint32_t reserved3;
    uint32_t reserved4;
    uint32_t crc32;

} firmware_info_t;

typedef struct comms_packet_t {

    uint8_t length;
    uint8_t data[PACKET_DATA_LENGTH];
    uint8_t crc;
    
} comms_packet_t;

static comms_packet_t retx_packet;
static comms_packet_t ack_packet;
static comms_packet_t rx_packet;
static comms_packet_t tx_packet;

/**
 * 
 */
void sigint_handler(int signum);
uint8_t compute_crc(uint8_t *data);
void craft_packet(comms_packet_t *packet, char *data, size_t length);
void send_packet(int uart, comms_packet_t *packet);
int handle_packet(int uart, uint8_t* data, size_t length);
void uartsend(int uart, char *data, size_t length);
int uartrecieve(int uart, char *data, size_t offset, size_t len);
void write_firmwareinfo(firmware_info_t *info, FILE *fptr);
void read_firmwareinfo(firmware_info_t *info, FILE *fptr);
void update_firmwareinfo(firmware_info_t *info, FILE *fptr);

#endif