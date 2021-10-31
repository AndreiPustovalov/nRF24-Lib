#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H

#include "stm8l15x.h"

void Disable_EXTI1(void);
void Clear_EXTI1_IRQ_flag(void);
void Init_GPIO(void);
void Init_SPI(void);

// Chip Enable Activates RX or TX mode
void CE_L(void);
void CE_H(void);

// Chip Select
void CSN_L(void);
void CSN_H(void);

uint8_t SPI_SendRecv(uint8_t data);

void SPI_Calculate_CRC(void);
void SPI_Send_CRC(void);
void SPI_Wait_Not_BSY(void);

void nRF24_BeforeSleep(void);
void nRF24_AfterWake(void);

bool nRF24_Wait_IRQ(uint16_t wait);

@far @interrupt void EXTI_nRF24_IRQ(void);

#endif /* __NRF24_HAL_H */