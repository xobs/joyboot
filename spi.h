#ifndef __SPI_H__
#define __SPI_H__

void spiAssertCs(void);
void spiDeassertCs(void);
void spiReadStatus(void);

/**
 * @brief   Send a byte and discard the response
 *
 * @notapi
 */
void spiXmitByteSync(uint8_t byte);

/**
 * @brief   Send a dummy byte and return the response
 *
 * @return              value read from said register
 *
 * @notapi
 */
uint8_t spiRecvByteSync(void);

void spiInit(void);

#endif /*__SPI_H__*/
