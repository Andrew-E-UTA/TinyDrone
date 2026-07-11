// nrf24l01.c
//Andrew Espinoza

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------
// Target Platform: EK-TM4C123GXL w/ nRF24L01+
// Target uC:       TM4C123GH6PM
// System Clock:    80 MHz

// Hardware configuration:
// nRF24L01+ Wireless Transceiver Module on Spi2
//   MOSI (SSI0Tx)  on PD3
//   MISO (SSI0Rx)  on PD2
//   SCLK (SSI0Clk) on PD0
//   CSN  (SSI0FSs) on PD1
//   CE   (Rx | Tx) on PE3
//   IRQ  (INT)     on PB5

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "gpio.h"
#include "nvic.h"
#include "Spi2.h"
#include "uart0.h"
#include "nrf24l01.h"

// Pins
#define CE              PORTA,2
#define CSN             PORTA,3
#define NRF24l01_INT    PORTA,4

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

bool dataReceived = false;
uint8_t nrfPayload[MAX_PAY_LEN] = {};

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void nrfIrqIsr() {
    uint8_t status = nrfReadReg(STATUS);

    // Data Received
    if(status & STATUS_RX_DR) {
        dataReceived = true;
    }

    // Data Sent
    if(status & STATUS_TX_DS) {
    }

    // Max number of retries
    if(status & STATUS_MAX_RT) {
    }

    nrfWriteReg(STATUS, CLEAR_STATUS_INTS);         // Clear interrupts on module
    nrfReadRxPayload(nrfPayload);
    clearPinInterrupt(NRF24l01_INT);                         // Clear pin interrupt
}

void nrfCsnEnable() {
    setPinValue(CSN, 0);
    _delay_cycles(4);                    // allow line to settle
}

void nrfCsnDisable() {
    setPinValue(CSN, 1);
}

// Read single byte from register
uint8_t nrfReadReg(uint8_t reg)
{
    uint8_t data;
    nrfCsnEnable();
    writeSpi2Data(READ_REG | (reg & 0x1F));
    readSpi2Data();
    writeSpi2Data(0);
    data = readSpi2Data();
    nrfCsnDisable();
    return data;
}

// Write single byte to register
void nrfWriteReg(uint8_t reg, uint8_t data)
{
    nrfCsnEnable();
    writeSpi2Data(WRITE_REG | (reg & 0x1F));
    readSpi2Data();
    writeSpi2Data(data);
    readSpi2Data();
    nrfCsnDisable();
}

// Write multiple bytes to a register
void nrfWriteRegMultBytes(uint8_t reg, uint8_t *data, uint8_t size) {
    uint8_t i;
    nrfCsnEnable();
    writeSpi2Data(WRITE_REG | (reg & 0x1F));
    readSpi2Data();
    for(i = 0; i < size; i++) {
        writeSpi2Data(data[i]);
        readSpi2Data();
    }
    nrfCsnDisable();
}

// Read Multiple Bytes from a register
void nrfReadRegMultBytes(uint8_t reg, uint8_t *data, uint8_t size) {
    uint8_t i;
    nrfCsnEnable();
    writeSpi2Data(READ_REG | (reg & 0x1F));
    readSpi2Data();
    for(i = 0; i < size; i++) {
        writeSpi2Data(0);
        data[i] = readSpi2Data();
    }
    nrfCsnDisable();
}

// Write stand alone SPI commands
void nrfWriteCommand(uint8_t command) {
    nrfCsnEnable();
    writeSpi2Data(command);
    readSpi2Data();
    writeSpi2Data(0);
    readSpi2Data();
    nrfCsnDisable();
}

// Write Tx-Payload
void nrfWriteTxPayload(uint8_t *data) {
    uint8_t i;
    nrfCsnEnable();
    writeSpi2Data(W_TX_PYLD);               // SPI command to write Tx FIFO
    readSpi2Data();
    for(i = 0; i < MAX_PAY_LEN; i++) {      // Write payload (32-bytes max)
        writeSpi2Data(data[i]);
        readSpi2Data();
    }
    nrfCsnDisable();
}

// Read Rx-Payload
void nrfReadRxPayload(uint8_t *data) {
    uint8_t i;
    nrfCsnEnable();
    writeSpi2Data(R_RX_PYLD);               // SPI command to read Rx FIFO
    readSpi2Data();
    for(i = 0; i < MAX_PAY_LEN; i++) {      // Read payload (32-bytes max)
        writeSpi2Data(0);
        data[i] = readSpi2Data();
    }
    nrfCsnDisable();

    waitMicrosecond(1000);
    nrfWriteCommand(FLUSH_RX);
}

// Set up NRF24L01 in Transmit Mode
void nrfSetTxMode(uint8_t channel, uint8_t *txAdd) {
    setPinValue(CE, 0);

    nrfWriteReg(RF_CH, channel);                            // Set frequency channel

    nrfWriteRegMultBytes(TX_ADDR, txAdd, MAX_ADD_LEN);      // Set Tx Address

    nrfWriteReg(CONFIG, PWR_UP);                            // Power up device in Tx mode

}

// Set up NRF24L01 in Receiving Mode
void nrfSetRxMode(uint8_t channel, uint8_t *rxAdd) {
    setPinValue(CE, 0);

    nrfWriteReg(RF_CH, channel);

    nrfWriteReg(EN_RXADDR, ERX_P0);                         // Enable Rx address on pipe 2

    nrfWriteRegMultBytes(RX_ADDR_P0, rxAdd, MAX_ADD_LEN);   // Rx address on pipe 2 same as Tx address

    nrfWriteReg(RX_PW_P0, MAX_PAY_LEN);                     // Set Payload size of 32 bytes on pipe 2

    nrfWriteReg(CONFIG, (PWR_UP | PRIM_RX));                // Power up device in Rx mode

    setPinValue(CE, 1);

    waitMicrosecond(130);
}

// Assumes the nrf has perviosly been setup for rx
//  and the registers still hold necessary values
void nrfQuickRxMode(void) {
    nrfWriteReg(CONFIG, (PWR_UP | PRIM_RX));                // Power up device in Rx mode

    setPinValue(CE, 1);

    waitMicrosecond(130);
}

void nrfInit() {
    nrfCsnDisable();
    setPinValue(CE, 0);

    // Basic Module Configuration
    nrfWriteReg(CONFIG, PWR_DWN_NO_CONFIG);             // Power down & clear register
    nrfWriteReg(EN_AA, ENNA_DISBL_AA);                  // No Auto Acknowledgment on any pipe
    nrfWriteReg(EN_RXADDR, ERX_DISBL_PIPES);            // No RX addressed enabled on any pipe
    nrfWriteReg(SETUP_AW, AW_5_BYTES);                  // Address width of 5-bytes
    nrfWriteReg(SETUP_RETR, RETR_DISABLED);             // No auto-retransmisison
    nrfWriteReg(RF_CH, RF_DFLT_CH);                     // Default ch.2
    nrfWriteReg(RF_SETUP, RF_LNA_HCUR);                // Default 0dBm & 1Mbps
    nrfWriteReg(STATUS, CLEAR_STATUS_INTS);             // Clear status Interrupts if any

    nrfWriteCommand(FLUSH_RX);                          // Flush RX
    nrfWriteCommand(FLUSH_TX);                          // FLUSH TX

    setPinValue(CE, 1);
}

void nrfReset(void) {
    // Basic Module Configuration
    nrfWriteReg(CONFIG, PWR_DWN_NO_CONFIG);             // Power down & clear register
    nrfWriteReg(EN_AA, ENNA_DISBL_AA);                  // No Auto Acknowledgment on any pipe
    nrfWriteReg(EN_RXADDR, ERX_DISBL_PIPES);            // No RX addressed enabled on any pipe
    nrfWriteReg(SETUP_AW, AW_5_BYTES);                  // Address width of 5-bytes
    nrfWriteReg(SETUP_RETR, RETR_DISABLED);             // No auto-retransmisison
    nrfWriteReg(RF_CH, RF_DFLT_CH);                     // Default ch.2
    nrfWriteReg(RF_SETUP, RF_PWR_0_DBM);                // Default 0dBm & 1Mbps
    nrfWriteReg(STATUS, CLEAR_STATUS_INTS);             // Clear status Interrupts if any

    nrfWriteCommand(FLUSH_RX);                          // Flush RX
    nrfWriteCommand(FLUSH_TX);                          // FLUSH TX

    setPinValue(CE, 1);
}

void nrfTransmitData(uint8_t *data) {
    nrfWriteCommand(FLUSH_TX);                  // FLUSH TX beforehand
    nrfWriteTxPayload(data);                    // Write payload

    // Pulse CE to send data (At least 10us high)
    setPinValue(CE, 1);
    waitMicrosecond(15);
    setPinValue(CE, 0);

}
