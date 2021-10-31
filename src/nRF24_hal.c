#include "nRF24_hal.h"
#include "nRF24_conf.h"

static volatile bool irqFired;

void CE_L(void) {
  GPIO_ResetBits(CE_PORT, CE_PIN);
}
void CE_H(void) {
  GPIO_SetBits(CE_PORT, CE_PIN);
}

void CSN_L(void) {
  GPIO_ResetBits(CSN_PORT, CSN_PIN);
}
void CSN_H(void) {
  GPIO_SetBits(CSN_PORT, CSN_PIN);
}

void Init_GPIO(void) {
    // IRQ  --> PB2
    // CE   <-- PB1
    // CSN  <-- PB0
    // SCK  <-- PB5
    // MOSI <-- PB6
    // MISO --> PB7

    // SCK,MOSI,CSN,CE pins set as output with push-pull at 10MHz
    GPIO_Init(CE_PORT, CE_PIN, GPIO_Mode_Out_PP_High_Fast);
    GPIO_Init(CSN_PORT, CSN_PIN, GPIO_Mode_Out_PP_High_Fast);
    GPIO_Init(SCK_PORT, SCK_PIN, GPIO_Mode_Out_PP_High_Fast);
    GPIO_Init(MOSI_PORT, MOSI_PIN, GPIO_Mode_Out_PP_High_Fast);


    // MISO pin (PB7) set as input with pull-up. Disable external interrupt
    GPIO_Init(MISO_PORT, MISO_PIN, GPIO_Mode_In_PU_No_IT);
    // IRQ pin (PC1) set as input without pull-up
    GPIO_Init(IRQ_PORT, IRQ_PIN, GPIO_Mode_In_FL_IT);
    
    EXTI_SetPinSensitivity(IRQ_EXTI_PIN, EXTI_Trigger_Falling);
}

void Init_SPI(void) {
  CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);
  SPI_Init(
    SPI1, 
    SPI_FirstBit_MSB, 
    SPI_BaudRatePrescaler_2, 
    SPI_Mode_Master,
    SPI_CPOL_Low,
    SPI_CPHA_1Edge,
    SPI_Direction_2Lines_FullDuplex,
    SPI_NSS_Soft,
    0x07
  );
  SPI_Cmd(SPI1, ENABLE);
}

// Transmit byte via SPI
// input:
//   data - byte to send
// return: received byte from SPI
uint8_t SPI_SendRecv(uint8_t data) {
    uint8_t rcv;

    SPI_SendData(SPI1, data); // Send byte to SPI (TXE cleared)
    while (SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE) != SET); // Wait for TXE flag --> transmit buffer is empty
    while (SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE) != SET); // Wait until byte is received
    rcv = SPI_ReceiveData(SPI1); // Read received byte (RXNE cleared)

    return rcv;
}

void SPI_Calculate_CRC(void) {
  SPI_CalculateCRCCmd(SPI1, ENABLE);
  SPI_ResetCRC(SPI1);
}

void SPI_Send_CRC(void) {
  SPI_TransmitCRC(SPI1); // Next transfer will be a CRC byte
}

void SPI_Wait_Not_BSY(void) {
  while(SPI_GetFlagStatus(SPI1, SPI_FLAG_BSY) != RESET);
}

bool nRF24_Wait_IRQ(uint16_t wait) {
  irqFired = FALSE;
  halt();
  return irqFired;
}

@far @interrupt void EXTI_nRF24_IRQ(void) {
  EXTI_ClearITPendingBit(IRQ_EXTI_IT);
  irqFired = TRUE;
}