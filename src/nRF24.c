#include "stm8l15x.h"

#include "nRF24.h"
#include "nRF24_hal.h"
#include "delay.h"


const uint8_t RX_PW_PIPES[6] = {
    nRF24_REG_RX_PW_P0,
    nRF24_REG_RX_PW_P1,
    nRF24_REG_RX_PW_P2,
    nRF24_REG_RX_PW_P3,
    nRF24_REG_RX_PW_P4,
    nRF24_REG_RX_PW_P5
};
const uint8_t RX_ADDR_PIPES[6] = {
    nRF24_REG_RX_ADDR_P0,
    nRF24_REG_RX_ADDR_P1,
    nRF24_REG_RX_ADDR_P2,
    nRF24_REG_RX_ADDR_P3,
    nRF24_REG_RX_ADDR_P4,
    nRF24_REG_RX_ADDR_P5
};

// GPIO and SPI initialization
void nRF24_Init() {
    
    Init_GPIO();
    Init_SPI();

    CSN_H();
    CE_L(); // CE pin low -> power down mode at startup

    nRF24_ClearIRQFlags();
}

// Write new value to the nRF24L01 register
// input:
//   reg - register number
//   value - new value
// return: nRF24L01 status
void nRF24_WriteReg(uint8_t reg, uint8_t value) {
    CSN_L();
    SPI_SendRecv(u8(nRF24_CMD_WREG | reg)); // Select register
    SPI_SendRecv(value); // Write value to register
    CSN_H();
}

// Read value of the nRF24L01 register
// input:
//   reg - register number
// return: register value
uint8_t nRF24_ReadReg(uint8_t reg) {
    uint8_t value;

    CSN_L();
    SPI_SendRecv(u8(reg & 0x1f)); // Select register to read from
    value = SPI_SendRecv(nRF24_CMD_NOP); // Read register value
    CSN_H();

    return value;
}

// Read specified amount of data from the nRF24L01 into data buffer
// input:
//   reg - register number
//   pBuf - pointer to the data buffer
//   count - number of bytes to read
void nRF24_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
    CSN_L();
    SPI_SendRecv(reg); // Send buffer address
    while (count--) {
      *pBuf++ = SPI_SendRecv(nRF24_CMD_NOP);
    }
    SPI_Wait_Not_BSY();
    CSN_H();
}

static void nRF24_WriteBuf_int(uint8_t *pBuf, uint8_t count) {
    while (count--) {
      SPI_SendRecv(*pBuf++);
    }
}

// Send data buffer to the nRF24L01
// input:
//   reg - register number
//   pBuf - pointer to the data buffer
//   count - number of bytes to send
void nRF24_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
    CSN_L();
    SPI_SendRecv(u8(nRF24_CMD_WREG | reg)); // Send buffer address
    nRF24_WriteBuf_int(pBuf, count);
    SPI_Wait_Not_BSY();
    CSN_H();
}

// Check if nRF24L01 present (send byte sequence, read it back and compare)
// return:
//   1 - looks like an nRF24L01 is online
//   0 - received sequence differs from original
uint8_t nRF24_Check(void) {
    uint8_t rxbuf[5];
    uint8_t *ptr = (uint8_t *)nRF24_TEST_ADDR;
    uint8_t i;

    nRF24_WriteBuf(nRF24_REG_TX_ADDR,ptr,5); // Write fake TX address
    nRF24_ReadBuf(nRF24_REG_TX_ADDR,rxbuf,5); // Read TX_ADDR register
    for (i = 0; i < 5; i++) {
      if (rxbuf[i] != ptr[i])
        return 0;
    }

    return 1;
}

// Set nRF24L01 frequency channel
// input:
//   RFChannel - Frequency channel (0..127) (frequency = 2400 + RFChan [MHz])
// Note, what part of the OBSERVER_TX register called "PLOS_CNT" will be cleared!
void nRF24_SetRFChannel(uint8_t RFChannel) {
    nRF24_WriteReg(nRF24_REG_RF_CH, RFChannel);
}

// Flush nRF24L01 TX FIFO buffer
void nRF24_FlushTX(void) {
    nRF24_WriteReg(nRF24_CMD_FLUSH_TX,0xFF);
}

// Flush nRF24L01 RX FIFO buffer
void nRF24_FlushRX(void) {
    nRF24_WriteReg(nRF24_CMD_FLUSH_RX,0xFF);
}

// Put nRF24L01 in TX mode
// input:
//   RetrCnt - Auto retransmit count on fail of AA (1..15 or 0 for disable)
//   RetrDelay - Auto retransmit delay 250us+(0..15)*250us (0 = 250us, 15 = 4000us)
//   RFChan - Frequency channel (0..127) (frequency = 2400 + RFChan [MHz])
//   DataRate - Set data rate: nRF24_DataRate_1Mbps or nRF24_DataRate_2Mbps
//   TXPower - RF output power (-18dBm, -12dBm, -6dBm, 0dBm)
//   CRCS - CRC encoding scheme (nRF24_CRC_[off | 1byte | 2byte])
//   Power - power state (nRF24_PWR_Up or nRF24_PWR_Down)
//   TX_Addr - buffer with TX address
//   TX_Addr_Width - size of the TX address (3..5 bytes)
void nRF24_TXMode(uint8_t RetrCnt, uint8_t RetrDelay, uint8_t RFChan, nRF24_DataRate_TypeDef DataRate,
                  nRF24_TXPower_TypeDef TXPower, nRF24_CRC_TypeDef CRCS, nRF24_PWR_TypeDef Power, uint8_t *TX_Addr,
                  uint8_t TX_Addr_Width) {
    uint8_t rreg;

    CE_L();
    nRF24_ReadReg(0x00); // Dummy read
    nRF24_WriteReg(nRF24_REG_SETUP_AW, u8(TX_Addr_Width - 2)); // Set address width
    nRF24_WriteBuf(nRF24_REG_TX_ADDR,TX_Addr,TX_Addr_Width); // Set static TX address
    nRF24_WriteReg(nRF24_REG_RF_SETUP,u8((uint8_t)DataRate | (uint8_t)TXPower)); // Setup register
    delay_ms(2);
    nRF24_WriteReg(nRF24_REG_CONFIG,u8((uint8_t)CRCS | (uint8_t)Power | nRF24_PRIM_TX)); // Config register
    nRF24_SetRFChannel(RFChan); // Set frequency channel (OBSERVER_TX part PLOS_CNT will be cleared)
    rreg = nRF24_ReadReg(nRF24_REG_EN_AA);
    nRF24_WriteReg(nRF24_REG_SETUP_RETR, u8((RetrDelay << 4) | (RetrCnt & 0x0f))); // Auto retransmit settings
    if (RetrCnt) {
        // Enable auto acknowledgment for data pipe 0
        rreg |= nRF24_ENAA_P0;
        // Static RX address of the PIPE0 must be same as TX address for auto ack
        nRF24_WriteBuf(nRF24_REG_RX_ADDR_P0,TX_Addr,TX_Addr_Width);
    } else {
        // Disable auto acknowledgment for data pipe 0
        rreg &= u8(~nRF24_ENAA_P0);
    }
    nRF24_WriteReg(nRF24_REG_EN_AA,rreg);
}

// Put nRF24L01 in RX mode
// input:
//   PIPE - RX data pipe (nRF24_RX_PIPE[0..5])
//   PIPE_AA - auto acknowledgment for data pipe (nRF24_ENAA_P[0..5] or nRF24_ENAA_OFF)
//   RFChan - Frequency channel (0..127) (frequency = 2400 + RFChan [MHz])
//   DataRate - Set data rate (nRF24_DataRate_[250kbps,1Mbps,2Mbps])
//   CRCS - CRC encoding scheme (nRF24_CRC_[off | 1byte | 2byte])
//   RX_Addr - buffer with TX address
//   RX_Addr_Width - size of TX address (3..5 byte)
//   RX_PAYLOAD - receive buffer length
//   TXPower - RF output power for ACK packets (-18dBm, -12dBm, -6dBm, 0dBm)
void nRF24_RXMode(nRF24_RX_PIPE_TypeDef PIPE, nRF24_ENAA_TypeDef PIPE_AA, uint8_t RFChan,
                  nRF24_DataRate_TypeDef DataRate, nRF24_CRC_TypeDef CRCS, uint8_t *RX_Addr, uint8_t RX_Addr_Width,
                  uint8_t RX_PAYLOAD, nRF24_TXPower_TypeDef TXPower) {
    uint8_t rreg;

    CE_L();
    nRF24_ReadReg(nRF24_CMD_NOP); // Dummy read
    rreg = nRF24_ReadReg(nRF24_REG_EN_AA);
    if (PIPE_AA != nRF24_ENAA_OFF) {
        // Enable auto acknowledgment for given data pipe
        rreg |= (uint8_t)PIPE_AA;
    } else {
        // Disable auto acknowledgment for given data pipe
        rreg &= u8(~(1 << (uint8_t)PIPE));
    }
    nRF24_WriteReg(nRF24_REG_EN_AA,rreg);
    rreg = nRF24_ReadReg(nRF24_REG_EN_RXADDR);
    nRF24_WriteReg(nRF24_REG_EN_RXADDR, u8(rreg | (1 << (uint8_t)PIPE))); // Enable given data pipe
    nRF24_WriteReg(RX_PW_PIPES[(uint8_t)PIPE],RX_PAYLOAD); // Set RX payload length
    nRF24_WriteReg(nRF24_REG_RF_SETUP, u8((uint8_t)DataRate | (uint8_t)TXPower)); // SETUP register
    delay_ms(2);
    nRF24_WriteReg(nRF24_REG_CONFIG, u8((uint8_t)CRCS | nRF24_PWR_Up | nRF24_PRIM_RX)); // Config register
    nRF24_SetRFChannel(RFChan); // Set frequency channel
    nRF24_WriteReg(nRF24_REG_SETUP_AW, u8(RX_Addr_Width - 2)); // Set of address widths (common for all data pipes)
    nRF24_WriteBuf(RX_ADDR_PIPES[(uint8_t)PIPE],RX_Addr,RX_Addr_Width); // Set static RX address for given data pipe
    nRF24_ClearIRQFlags();
    nRF24_FlushRX();
    CE_H(); // RX mode
}

// Send data packet
// input:
//   pBuf - buffer with data to send
//   TX_PAYLOAD - buffer size
// return:
//   nRF24_TX_XXX values
nRF24_TX_PCKT_TypeDef nRF24_TXPacket(void * pBuf, uint8_t TX_PAYLOAD) {
    uint8_t status;
    uint8_t irqFlag = 0;

    // Release CE pin (in case if it still high)
    CE_L();
    // Transfer data from specified buffer to the TX FIFO
    nRF24_WriteBuf(nRF24_CMD_W_TX_PAYLOAD,pBuf,TX_PAYLOAD);
    // CE pin high => Start transmit (must hold pin at least 10us)
    CE_H();
    delay_10us(2);
    // Wait for IRQ from nRF24L01
    if (!nRF24_Wait_IRQ(nRF24_WAIT_TIMEOUT)) {
      irqFlag = nRF24_NO_IRQ;
    }
    // Release CE pin
    CE_L();

    // Read the status register and clear pending IRQ flags
    status = nRF24_ClearIRQFlags();
    if (status & nRF24_MASK_MAX_RT) {
        // Auto retransmit counter exceeds the programmed maximum limit. FIFO is not removed.
        nRF24_FlushTX();

        return u8(irqFlag | nRF24_TX_MAXRT);
    }
    if (status & nRF24_MASK_TX_DS) {
        // Transmit successful
        return u8(irqFlag | nRF24_TX_SUCCESS);
    }

    // Some banana happens
    nRF24_FlushTX();
    nRF24_ClearIRQFlags();
    if (irqFlag) {
      return irqFlag;
    }
    return nRF24_TX_ERROR;
}

// Read received data packet from the nRF24L01
// input:
//   pBuf - buffer for received data
//   RX_PAYLOAD - buffer size
// return:
//   nRF24_RX_PCKT_PIPE[0..5] - packet received from specific data pipe
//   nRF24_RX_PCKT_ERROR - RX_DR bit was not set
//   nRF24_RX_PCKT_EMPTY - RX FIFO is empty
nRF24_RX_PCKT_TypeDef nRF24_RXPacket(void * pBuf, uint8_t RX_PAYLOAD) {
    uint8_t status;
    nRF24_RX_PCKT_TypeDef result = nRF24_RX_PCKT_ERROR;

    status = nRF24_ReadReg(nRF24_REG_STATUS); // Read the status register
    if (status & nRF24_MASK_RX_DR) {
        // RX_DR bit set (Data ready RX FIFO interrupt)
        result = (nRF24_RX_PCKT_TypeDef)((status & 0x0e) > 1); // Pipe number
        if ((uint8_t)result < 6) {
            // Read received payload from RX FIFO buffer
            nRF24_ReadBuf(nRF24_CMD_R_RX_PAYLOAD,pBuf,RX_PAYLOAD);
            // Clear pending IRQ flags
            nRF24_WriteReg(nRF24_REG_STATUS, u8(status | 0x70));
            // Check if RX FIFO is empty and flush it if not
            status = nRF24_ReadReg(nRF24_REG_FIFO_STATUS);
            if (!(status & nRF24_FIFO_RX_EMPTY)) nRF24_FlushRX();

            return result; // Data pipe number
        } else {
            // RX FIFO is empty
            return nRF24_RX_PCKT_EMPTY;
        }
    }

    // Some banana happens
    nRF24_FlushRX(); // Flush the RX FIFO buffer
    nRF24_ClearIRQFlags();

    return result;
}

// Clear pending IRQ flags
uint8_t nRF24_ClearIRQFlags(void) {
    uint8_t status;

    status = nRF24_ReadReg(nRF24_REG_STATUS);
    nRF24_WriteReg(nRF24_REG_STATUS, u8(status | 0x70));
    return status;
}

// Put nRF24 in Power Down mode
void nRF24_PowerDown(void) {
    uint8_t conf;

    CE_L(); // CE pin to low
    conf  = nRF24_ReadReg(nRF24_REG_CONFIG);
    conf &= u8(~(1<<1)); // Clear PWR_UP bit
    nRF24_WriteReg(nRF24_REG_CONFIG,conf); // Go Power down mode
}

// Wake nRF24 from Power Down mode
// note: with external crystal it wake to Standby-I mode within 1.5ms
void nRF24_Wake(void) {
    uint8_t conf;
    conf = u8(nRF24_ReadReg(nRF24_REG_CONFIG) | (1<<1)); // Set PWR_UP bit
    nRF24_WriteReg(nRF24_REG_CONFIG,conf); // Wake-up
}

// Configure RF output power in TX mode
// input:
//   TXPower - RF output power (-18dBm, -12dBm, -6dBm, 0dBm)
void nRF24_SetTXPower(nRF24_TXPower_TypeDef TXPower) {
    uint8_t rf_setup;

    rf_setup  = nRF24_ReadReg(nRF24_REG_RF_SETUP);
    rf_setup &= 0xf9; // Clear RF_PWR bits
    nRF24_WriteReg(nRF24_REG_RF_SETUP, u8(rf_setup | (uint8_t)TXPower));
}
