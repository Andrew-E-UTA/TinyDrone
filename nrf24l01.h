// Target Platform: EK-TM4C123GXL w/ nRF24L01+
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// nRF24L01+ Wireless Transceiver Module on SPI0
//   MOSI (SSI0Tx)  on PA5
//   MISO (SSI0Rx)  on PA4
//   SCLK (SSI0Clk) on PA2
//   CSN  (SSI0FSs) on PA3
//   CE   (Rx | Tx) on PB3
//   IRQ  (INT)     on PC6

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef NRF24L01_H_
#define NRF24L01_H_

#include <stdint.h>
#include <stdbool.h>

// Register Map
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RPD         0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

// Register Bit Descriptions
#define MASK_RX_DR          0x40    // Interrupt not reflected on IRQ pin if set
#define MASK_TX_DS          0x20    // Interrupt not reflected on IRQ pin if set
#define MASK_MAX_RT         0x10    // Interrupt not reflected on IRQ pin if set
#define EN_CRC              0x08    // Enabled by default
#define CRCO                0x04    // CRC Scheme --> '0' 1-Byte     | '1' 2-Byte
#define PWR_UP              0x02    // Device     --> '0' POWER DOWN | '1' POWER UP
#define PRIM_RX             0x01    // RX/TX Ctrl --> '0' Transmit   | '1' Receive
#define PWR_DWN_NO_CONFIG   0x00

#define ENAA_P5             0x20    // Enable Auto Acknowledgment on pipe x
#define ENAA_P4             0x10
#define ENAA_P3             0x08
#define ENAA_P2             0x04
#define ENAA_P1             0x02
#define ENAA_P0             0x01
#define ENNA_DISBL_AA       0x00

#define ERX_P5              0x20    // Enabled RX Addresses on pipe x
#define ERX_P4              0x10
#define ERX_P3              0x08
#define ERX_P2              0x04
#define ERX_P1              0x02
#define ERX_P0              0x01
#define ERX_DISBL_PIPES     0x00

#define AW_3_BYTES          0x01    // Setup of Address Widths
#define AW_4_BYTES          0x02
#define AW_5_BYTES          0x03

#define RETR_DISABLED       0x00    // No Automatic Retransmission

#define RF_DFLT_CH          0x00    // Sets the frequency channel nRF24L01 operates on

#define RF_PLL_LOCK         0x10
#define RF_DR               0x08    // '0' 1Mbps | '1' 2Mbps (2Mbps default)
#define RF_PWR_0_DBM        0x06    // Power output in TX mode (0dBm default)
#define RF_PWR_NEG_6_DBM    0x04
#define RF_PWR_NEG_12_DBM   0x02
#define RF_PWR_NEG_18_DBM   0x00
#define RF_LNA_HCUR         0x01

#define STATUS_RX_DR        0x40    // Interrupt when data received [R1C]
#define STATUS_TX_DS        0x20    // Interrupt when data sent     [R1C]
#define STATUS_MAX_RT       0x10    // Interrupt for max # of retransmissions [R1C]
#define STATUS_RX_EMPTY     0x0E    // Interrupt when RX FIFO empty
#define STATUS_RX_P5        0x0A    // Interrupt when P5 PAY ready to read
#define STATUS_RX_P4        0x08    // Interrupt when P4 PAY ready to read
#define STATUS_RX_P3        0x06    // Interrupt when P3 PAY ready to read
#define STATUS_RX_P2        0x04    // Interrupt when P2 PAY ready to read
#define STATUS_RX_P1        0x02    // Interrupt when P1 PAY ready to read
#define STATUS_RX_P0        0x00    // Interrupt when P0 PAY ready to read
#define STATUS_TX_FULL      0x01    // Interrupt when TX FIFO full
#define CLEAR_STATUS_INTS   0x70

#define FIFO_S_TX_REUSE     0x40    // Statuses of RX and TX FIFOs
#define FIFO_S_TX_FULL      0x20
#define FIFO_S_TX_EMPTY     0x10
#define FIFO_S_RX_FULL      0x02
#define FIFO_S_RX_EMPTY     0x01

#define DLP_P5              0x20    // Enable dynamic payload length on data pipe x
#define DLP_P4              0x10
#define DLP_P3              0x08
#define DLP_P2              0x04
#define DLP_P1              0x02
#define DLP_P0              0x01

#define EN_DLP              0x04    // Enables Dynamic Payload Length
#define EN_ACK_PAY          0x02    // Enables Payload with ACK
#define EN_DYN_ACK          0x01    // Enables the W_TX_PAYLOAD_NOACK command

// SPI Commands
#define READ_REG            0x00
#define WRITE_REG           0x20
#define R_RX_PYLD           0x61
#define W_TX_PYLD           0xA0
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define REUSE_TX_PL         0xE3
#define R_RX_PL_WID         0x60
#define W_ACK_PAYLOAD       0xA8
#define W_TX_PAYLOAD_NO_ACK 0xB0
#define NOP                 0xFF

// Other
#define MAX_PAY_LEN         32
#define MAX_ADD_LEN         5

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void nrfCsnEnable();
void nrfCsnDisable();

uint8_t nrfReadReg(uint8_t reg);
void nrfWriteReg(uint8_t reg, uint8_t data);
void nrfWriteRegMultBytes(uint8_t reg, uint8_t *data, uint8_t size);
void nrfReadRegMultBytes(uint8_t reg, uint8_t *data, uint8_t size);

void nrfWriteCommand(uint8_t command);

void nrfWriteTxPayload(uint8_t *data);
void nrfReadRxPayload(uint8_t *data);

void nrfSetTxMode(uint8_t channel, uint8_t *txAdd);
void nrfSetRxMode(uint8_t channel, uint8_t *rxAdd);
void nrfQuickRxMode();

void nrfInit();
void nrfReset();
void nrfTransmitData(uint8_t *data);

#endif
