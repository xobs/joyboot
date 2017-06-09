#ifndef __SPI_H__
#define __SPI_H__

void spiAssertCs(void);
void spiDeassertCs(void);
void spiReadStatus(void);
uint8_t spiTransceive(uint8_t byte);

void spiInit(void);

#endif /*__SPI_H__*/
