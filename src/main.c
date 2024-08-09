#include "common-defines.h"
#include <errno.h>
#include <time.h>

#define FIRMWARE_MAX_BUFFER 4096
#define RX_BUFFER_LENGTH 18
#define DATA0 1

#define FLASH_BASE                          (0x08000000)
#define BOOTLOADER_SIZE                     (0x8000U) // 32 KiB
#define VECTOR_TABLE_SIZE                   (0x1B0)
#define FIRMWARE_INFO_SIZE                  (10 * 4)

#define FWINFO_SENTINEL                     (0xDEADC0DE)
#define MAIN_APP_START_ADDRESS              (FLASH_BASE + BOOTLOADER_SIZE) // 0x08008000
#define FW_INFO_ADDRESS                     (BOOTLOADER_SIZE + VECTOR_TABLE_SIZE);
#define FW_INFO_VALIDATE_FROM               (FW_INFO_ADDRESS + FIRMWARE_INFO_SIZE);

#define FW_INFO_SENTINEL_OFFSET             (FW_INFO_ADDRESS + (0 * 4))
#define FW_INFO_DEVICE_ID_OFFSET            (FW_INFO_ADDRESS + (1 * 4))
#define FW_INFO_VERSION_OFFSET              (FW_INFO_ADDRESS + (2 * 4))
#define FW_INFO_LENGTH_OFFSET               (FW_INFO_ADDRESS + (3 * 4))
#define FW_INFO_CRC32_OFFSET                (FW_INFO_ADDRESS + (9 * 4))

#define MAX_FW_LENGTH                       ((1024 * 512) - BOOTLOADER_SIZE)
#define DEVICE_ID                           (0x41)

#define FW_INFO_VALIDATE_LENGTH(fwLength)  (fwLength - (sizeof(vector_table_t) + sizeof(firmware_info_t)))


const char *serialDevice = "/dev/ttyACM0";
const uint32_t baudRate =  B115200;
bool volatile doExit = false;

static comms_packet_t retx_packet;
static comms_packet_t ack_packet;
static comms_packet_t rx_packet;
static comms_packet_t tx_packet;
static comms_packet_t bl_packet;

static bl_state_t state = BL_State_SendSync;
char SYNC_SEQ[4] = { 0xC1, 0xC2, 0xC3, 0xC4};
uint32_t FIRMWARE_SIZE = 0x00004141;
static firmware_info_t firmware_info;
/**
 * 
 */
void sigint_handler(int signum) {

    exit(1);

     char  c;
     signal(signum, SIG_IGN);
     printf("Do you really want to quit? [y/n] ");
     c = getchar();
     if (c == 'y' || c == 'Y') {
        printf("Breaking program");
        doExit = true;
    }
     else {
          signal(SIGINT, sigint_handler);
     }
     getchar(); // Get new line character

}

/**
 * 
 */
uint8_t compute_crc(uint8_t *data) {

    uint32_t length = PACKET_LENGTH - PACKET_CRC_BYTES;
    
    uint8_t crc = 0;
    for(uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for(uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc;

}


/**
 *
 */
void read_firmwareinfo(firmware_info_t *info, FILE *fptr) {

    printf("[*]: Reading firmware info\n");

    uint32_t offset = FW_INFO_ADDRESS;
    fseek(fptr, offset, SEEK_SET);

    if ( (fread(info, sizeof(char), sizeof(firmware_info_t), fptr) == 0 ) ) {
        printf("[!]: Failed to read firmware info\n");
        exit(1);
    }

    fseek(fptr, 0L, SEEK_SET);


}

/*
 *
 */
void write_firmwareinfo(firmware_info_t *info, FILE *fptr) {

    printf("[*]: Writing firmware info\n");

    uint32_t offset = FW_INFO_ADDRESS;
    fseek(fptr, offset, SEEK_SET);

    if ( (fwrite(info, sizeof(char), sizeof(firmware_info_t), fptr) == 0 ) ) {
        printf("%s\n", strerror(errno));
        printf("[!]: Failed to write firmware info\n");
        exit(1);
    }

    fseek(fptr, 0L, SEEK_SET);

}



/*
 *
 */
void update_firmwareinfo(firmware_info_t *info, FILE *fptr) {

    printf("[*]: Updating firmware info\n");

    info->deviceid = DEVICE_ID;
    info->length = FIRMWARE_SIZE;
    info->version = 0x1;
    info->sentinel = 0xDEADBEEF;
    info->crc32 = 0x41424344;


}


/**
 * 
 */
void craft_packet(comms_packet_t *packet, char *data, size_t length) {

    if(length > PACKET_DATA_LENGTH) {
        printf("Data too large!\n");
        return;
    }
    packet->length = length;

    for(int i = 0; i < length; i++) {
        packet->data[i] = data[i];
    }

    for(int i = length; i < (PACKET_DATA_LENGTH);  i++) {
        packet->data[i] = 0xFF;
    }
    packet->crc = compute_crc( (uint8_t *)packet);

}

/*
*
*/
void send_packet(int uart, comms_packet_t *packet) {

    // Save the last sent packet
    tx_packet.length = packet->length;
    memcpy(&tx_packet.data, &packet->data, PACKET_DATA_LENGTH);
    tx_packet.crc = packet->crc;

    // Send the new packet
    uartsend(uart, (char *)packet, sizeof(struct comms_packet_t));

    if(packet->data[0] == PACKET_ACK_DATA0 && packet->length == 1) {
        return;
    }

    while(true) {
        // Verify if packet was recieved succesfully by checking for ACK
        char rx_buffer[18] = {0};
        int rx_count = 0;
        rx_count += uartrecieve(uart, rx_buffer, rx_count, 18);
        if(rx_count == RX_BUFFER_LENGTH) {
            rx_count = 0;
            handle_packet(uart, rx_buffer, RX_BUFFER_LENGTH);
            break;
        }
    }

}


/**
 * 
 */
int handle_packet(int uart, uint8_t* data, size_t length) {

    comms_packet_t *packet = (comms_packet_t *)(data);

    rx_packet.length = packet->length;
    memcpy(&rx_packet.data, &packet->data, PACKET_DATA_LENGTH);
    rx_packet.crc = packet->crc;

    uint32_t crccheck = compute_crc(data);

    if(crccheck != packet->crc) {
        //printf("CRC Mismatch!\n");
        // Send RETX
        //return PACKET_RETX_DATA0;
    }

    switch(rx_packet.data[0]) {

        case PACKET_ACK_DATA0:      // 21
            return PACKET_ACK_DATA0;

        case PACKET_RETX_DATA0:     // 25
            printf("Got RETX\n");
            send_packet(uart, &tx_packet);
            break;

        case BL_PACKET_NACK_DATA0: 
            printf("[!]: ERROR In firmware update. Exiting!\n");
            state = BL_State_Done;
            break;

        case BL_PACKET_SYNC_OBSERVED_DATA0:
            return BL_PACKET_SYNC_OBSERVED_DATA0;
        
        case BL_PACKET_READY_FOR_DATA_DATA0:
            return BL_PACKET_READY_FOR_DATA_DATA0;

        case BL_PACKET_FW_UPDATE_RES_DATA0:
            return BL_PACKET_FW_UPDATE_RES_DATA0;

        case BL_PACKET_DEVICE_ID_REQ_DATA0:
            return BL_PACKET_DEVICE_ID_REQ_DATA0;

        case BL_PACKET_FW_LENGTH_REQ_DATA0:
            return BL_PACKET_FW_LENGTH_REQ_DATA0;

        default:
            send_packet(uart, &ack_packet);
            break;
    }
}


/**
 * 
 */
void uartsend(int uart, char *data, size_t length) {

    int tx_count = 0;
    while(tx_count != length) {
        int bytesSent = write(uart, (void *)data+tx_count, length );
        if(bytesSent < 0) {

        }
        else if(bytesSent == 0) {

        }
        else {
            tx_count += bytesSent;
        }
    }
}

/**
 * 
 */
int uartrecieve(int uart, char *data, size_t offset, size_t len) {

    int recievedLength = read(uart, (void *)data+offset, len);
    
    if(recievedLength < 0) {
        return 0;
    }

    else if(recievedLength == 0) {
        return 0;
    }

    else {
        return recievedLength;
    }
}


/*
*
*/
void checkTimeOut(void) {

}

/*
*
*/
void resetTimer(void) {

}

/*
*
*/
void checkFail(char *buffer) {

    if(buffer[DATA0] == BL_PACKET_NACK_DATA0) {
        printf("[!]: ERROR In firmware update. Exiting!\n");
        state = BL_State_Done;
    }
}


/**
 * 
 */
void print_progress_bar(int current, int max) {
    int barWidth = 50; // Width of the progress bar
    float percentage = (float)current / max * 100; // Calculate the percentage
    printf("[");
    int pos = (int)(barWidth * current / max); // Calculate the position of the bar
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) {
            printf("*");
        } else {
            printf(" ");
        }
    }
    printf("] %d%%\r", (int)percentage); // Print the percentage
    fflush(stdout); // Flush the output buffer to display changes
}



/**
 * 
*/
uint32_t getFileSize(char *fileName) {

   FILE *file = NULL;
    if ( (file = fopen(fileName, "r+b") ) == NULL) {
        printf("Failed to open file: %s\n", fileName);
        exit(0);
    }

    uint32_t size = 0;
    fseek(file, 0L, SEEK_END);
    size = ftell(file);
    fseek(file, 0L, SEEK_SET);
    fclose(file);
    printf("[*]: Current file size: %u\n", size);

    return size;

}


/**
 * 
 */
int main(int argc, char **argv) {

    int uart = -1;
    uint8_t rx_buffer[RX_BUFFER_LENGTH];
    memset(rx_buffer, '\x00', RX_BUFFER_LENGTH);
    memset(&firmware_info, '\x00', sizeof(struct firmware_info_t));
    int rx_count = 0;

    /*
    Packet setup
    */
    char sigACK = PACKET_ACK_DATA0;
    char sigRETX = PACKET_RETX_DATA0;
    craft_packet(&retx_packet, &sigRETX, 1);
    craft_packet(&ack_packet, &sigACK, 1);

    /*
    Interrupt handling
    */
    signal(SIGINT, sigint_handler);


    /*
    Opens the firmware.bin file for reading the length and updating the firmware_info structure.
    */
    char *filename = "/home/user/projects/STM32/app/firmware.bin";
    FILE *firmware = fopen(filename, "rb+");
    
    if(firmware == NULL) {
        printf("[!]: Failed to open firmware file!\n");
        exit(1);
    }
    
    FIRMWARE_SIZE = getFileSize(filename);
    FIRMWARE_SIZE = FIRMWARE_SIZE - BOOTLOADER_SIZE;
    printf("[*]: FIRMWARE size: %u\n", FIRMWARE_SIZE);

    read_firmwareinfo(&firmware_info, firmware);
    update_firmwareinfo(&firmware_info, firmware);
    write_firmwareinfo(&firmware_info, firmware);


    /*
    UART Handling
    */
	
    //The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.

    uart = open(serialDevice, O_RDWR | O_NOCTTY); // | O_NDELAY);
    if(uart == -1) {
        printf("Failed to open serial device\n");
        exit(1);
    }
    printf("[*]: Got handle to UART Device: %d (%s)\n", uart, serialDevice);

    /*
    Configure serial port
    */ 

    /**
     * No flow control
     * 8 Data bits
     * Baudrate of 115200
     * No parity
     * 1 Stop bit
     */
    struct termios options;
    memset(&options, '\x00', sizeof(struct termios));
    tcgetattr(uart, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;	// Control modes: 8 Data bits, ignore modem status, enable reciever	
	options.c_iflag = IGNPAR | ICRNL;                   // Input modes: Ignore characters with parity errors, Map NL to CR.
	options.c_oflag = 0;                                // Output modes:
	tcflush(uart, TCIFLUSH);                            // Flush pending input
    tcflush(uart, TCOFLUSH);                            // Flush pending input
	tcsetattr(uart, TCSANOW, &options);                 // Change attributes immediately
    

    /*
    State machine
    */
    while(true) {

        if(state == BL_State_Done) {
            printf("[*]: Firmware Update Completed!\n");
            break;
        }

        printf("[*]: Firmware update begin\n");
        craft_packet(&bl_packet, SYNC_SEQ, 4);
        send_packet(uart, &bl_packet);
        printf("[*]: Sent Firmware Update Sync Signature\n");
        state = BL_State_WaitForSyncObservered;

         while(state != BL_State_Done) {
            
            if(state == BL_State_WaitForSyncObservered) {

                // Read up to 18 bytes. After reading 18 bytes complete.
                rx_count += uartrecieve(uart, rx_buffer, rx_count, RX_BUFFER_LENGTH - rx_count);
                checkTimeOut();
                if(rx_count == RX_BUFFER_LENGTH) {
                    if (handle_packet(uart, rx_buffer, RX_BUFFER_LENGTH) == BL_PACKET_SYNC_OBSERVED_DATA0 ) {
                        printf("[*]: Firmware Update Synchronized Observed\n");
                        state = BL_State_SendFirmwareUpdateRequest;
                    }
                    rx_count = 0;
                    resetTimer();
                }
                continue;
            }

            switch(state) {

                case BL_State_SendFirmwareUpdateRequest: {
                    char byte = BL_PACKET_FW_UPDATE_REQ_DATA0;
                    craft_packet(&bl_packet, &byte, 1);
                    send_packet(uart, &bl_packet);
                    printf("[*]: Send Firmware Update Request\n");
                    resetTimer();
                    state = BL_State_WaitForFirmwareUpdateAccept;

                } break;

                case BL_State_WaitForFirmwareUpdateAccept: {

                    rx_count += uartrecieve(uart, rx_buffer, rx_count, RX_BUFFER_LENGTH - rx_count);
                    checkTimeOut();
                    if(rx_count == RX_BUFFER_LENGTH) {
                        if (handle_packet(uart, rx_buffer, RX_BUFFER_LENGTH) == BL_PACKET_FW_UPDATE_RES_DATA0 ) {
                            printf("[*]: Firmware Update Request Accepted\n");
                            state = BL_State_WaitForDeviceIDRequest;
                            resetTimer();
                        }
                        rx_count = 0;
                    }

                } break;

                case BL_State_WaitForDeviceIDRequest: {

                    rx_count += uartrecieve(uart, rx_buffer, rx_count, RX_BUFFER_LENGTH - rx_count);
                    checkTimeOut();
                    if(rx_count == RX_BUFFER_LENGTH) {
                        if (handle_packet(uart, rx_buffer, RX_BUFFER_LENGTH) == BL_PACKET_DEVICE_ID_REQ_DATA0 ) {
                            printf("[*]: Device ID Request Observed\n");
                            state = BL_State_SendDeviceIDReply;
                            resetTimer();
                        }
                        rx_count = 0;
                    }
                } break;

                case BL_State_SendDeviceIDReply: {
                    char deviceID[2] = { BL_PACKET_DEVICE_ID_RES_DATA0, DEVICE_ID};
                    craft_packet(&bl_packet, deviceID, 2);
                    send_packet(uart, &bl_packet);
                    printf("[*]: Sent Device ID: %d\n", DEVICE_ID);
                    resetTimer();
                    state = BL_State_WaitForFirmwareLengthRequest;

                } break;

                case BL_State_WaitForFirmwareLengthRequest: {

                    rx_count += uartrecieve(uart, rx_buffer, rx_count, RX_BUFFER_LENGTH - rx_count);
                    checkTimeOut();
                    if(rx_count == RX_BUFFER_LENGTH) {
                        if (handle_packet(uart, rx_buffer, RX_BUFFER_LENGTH) == BL_PACKET_FW_LENGTH_REQ_DATA0 ) {
                            printf("[*]: Firmware Update Synchronized Observed\n");
                            state = BL_State_SendFirmwareLengthReply;
                            resetTimer();
                        }
                        rx_count = 0;
                    }
                } break;

                case BL_State_SendFirmwareLengthReply: {

                    uint8_t data[5] = {
                        BL_PACKET_FW_LENGTH_RES_DATA0,
                        ((FIRMWARE_SIZE >> 24) & 0xFF),
                        ((FIRMWARE_SIZE >> 16) & 0xFF),
                        ((FIRMWARE_SIZE >> 8) & 0xFF),
                        ((FIRMWARE_SIZE) & 0xFF),
                    };

                    craft_packet(&bl_packet, data, 5);
                    send_packet(uart, &bl_packet);
                    printf("[*]: Sent Firmware Length %d\n", FIRMWARE_SIZE);
                    resetTimer();
                    state = BL_State_WaitForReady;

                } break;


                case BL_State_WaitForReady: {

                    rx_count += uartrecieve(uart, rx_buffer, rx_count, RX_BUFFER_LENGTH - rx_count);
                    checkTimeOut();
                    if(rx_count == RX_BUFFER_LENGTH) {
                        if (handle_packet(uart, rx_buffer, RX_BUFFER_LENGTH) == BL_PACKET_READY_FOR_DATA_DATA0 ) {
                            printf("[*]: Ready to recieve firmware\n");
                            state = BL_State_SendFirmware;
                            fseek(firmware, BOOTLOADER_SIZE, SEEK_SET); // Only write the MAIN Part of the firmware file and not the bootloader.
                            resetTimer();
                        }
                        rx_count = 0;
                    }
                } break;

                case BL_State_SendFirmware: {

                    sleep(1);
                    
                    printf("[*]: Beginning firmware transfer. Do not interrupt \n");  
                    uint32_t numberofpackets = (FIRMWARE_SIZE / PACKET_DATA_LENGTH) + 1;
                    uint32_t fileMaxTransferBytes = PACKET_DATA_LENGTH;
                    uint32_t fileReadOffset = 0;
                    uint32_t fileSentOffset = 0;
                    uint32_t fileByteOffset = 0;

                    char *filenameTest = "./test.bin";
                    FILE *firmwareTest = fopen(filenameTest, "wb");

                    char fileContents[FIRMWARE_MAX_BUFFER] = { 0 };
                    ssize_t fileBytesRead = fread(fileContents, sizeof(uint8_t), FIRMWARE_MAX_BUFFER, firmware);
                    
                    for(uint32_t i = 1; i < numberofpackets + 1; i++) {
                        
                        craft_packet(&bl_packet, &fileContents[fileByteOffset], 16);
                        send_packet(uart, &bl_packet);

                        while(true) {
                            rx_count += uartrecieve(uart, rx_buffer, rx_count, RX_BUFFER_LENGTH - rx_count);
                            checkTimeOut();
                            if(rx_count == RX_BUFFER_LENGTH) {
                                if (handle_packet(uart, rx_buffer, RX_BUFFER_LENGTH) == BL_PACKET_READY_FOR_DATA_DATA0 ) {
                                    rx_count = 0;
                                    break;
                                }
                            }
                        }
                        
                        fileSentOffset += ( fileMaxTransferBytes );
                        fileByteOffset += fileMaxTransferBytes;

                        if( (i % 256) == 0 ) {
                            fileReadOffset += 4096;
                            fileByteOffset = 0;
                            fileBytesRead = fread(fileContents, sizeof(uint8_t), FIRMWARE_MAX_BUFFER, firmware );
                        }

                        print_progress_bar(i, numberofpackets);
                    }

                    printf("\n[*]: Firmware transfer complete\n");

                    state = BL_State_Done;

                } break;
            }

        }


    }

    fclose(firmware);
    close(uart);

    return 0;
}