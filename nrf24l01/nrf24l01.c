/*
 nrf24l01 lib 0x02

 copyright (c) Davide Gironi, 2012

 Released under GPLv3.
 Please refer to LICENSE file for licensing information.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

#include "nrf24l01.h"
#include "nrf24l01registers.h"

unsigned long millis();

//include spi library functions
#include NRF24L01_SPIPATH

//address variables
static uint8_t nrf24l01_addr0[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP0;
static uint8_t nrf24l01_addr1[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP1;
static uint8_t nrf24l01_addr2[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP2;
static uint8_t nrf24l01_addr3[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP3;
static uint8_t nrf24l01_addr4[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP4;
static uint8_t nrf24l01_addr5[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP5;
static uint8_t nrf24l01_addrtx[NRF24L01_ADDRSIZE] = NRF24L01_ADDRTX;

static uint8_t nrf24l01_dynamic_payloads_enabled = 0; /**< Whether dynamic payloads are enabled. */

/*
 * read one register
 */
uint8_t nrf24l01_readregister(uint8_t reg) {
	nrf24l01_CSNlo
	; //low CSN
	spi_writereadbyte(
			NRF24L01_CMD_R_REGISTER | (NRF24L01_CMD_REGISTER_MASK & reg));
	uint8_t result = spi_writereadbyte(NRF24L01_CMD_NOP); //read write register
	nrf24l01_CSNhi
	; //high CSN
	return result;
}

/*
 * read many registers
 */
uint8_t nrf24l01_readregisters(uint8_t reg, uint8_t *value, uint8_t len) {
	uint8_t i = 0;
	uint8_t status;

	nrf24l01_CSNlo
	; //low CSN
	status = spi_writereadbyte(
			NRF24L01_CMD_R_REGISTER | (NRF24L01_CMD_REGISTER_MASK & reg));
	for (i = 0; i < len; i++)
		value[i] = spi_writereadbyte(NRF24L01_CMD_NOP); //read write register
	nrf24l01_CSNhi
	; //high CSN
	return status;
}

/*
 * write one register
 */
void nrf24l01_writeregister(uint8_t reg, uint8_t value) {
	nrf24l01_CSNlo
	; //low CSN
	spi_writereadbyte(
			NRF24L01_CMD_W_REGISTER | (NRF24L01_CMD_REGISTER_MASK & reg));
	spi_writereadbyte(value); //write register
	nrf24l01_CSNhi
	; //high CSN
}

/*
 * write many registers
 */
void nrf24l01_writeregisters(uint8_t reg, uint8_t *value, uint8_t len) {
	uint8_t i = 0;
	nrf24l01_CSNlo
	; //low CSN
	spi_writereadbyte(
			NRF24L01_CMD_W_REGISTER | (NRF24L01_CMD_REGISTER_MASK & reg));
	for (i = 0; i < len; i++)
		spi_writereadbyte(value[i]); //write register
	nrf24l01_CSNhi
	; //high CSN
}

/*
 * reverse an array, NRF24L01 expects LSB first
 */
void nrf24l01_revaddress(uint8_t *addr, uint8_t *addrrev) {
	//reverse address
	uint8_t i = 0;
	for (i = 0; i < NRF24L01_ADDRSIZE; i++)
		memcpy(&addrrev[i], &addr[NRF24L01_ADDRSIZE - 1 - i], 1);
}

/*
 * set rx address
 */
void nrf24l01_setrxaddr(uint8_t pipe, uint8_t *addr) {
	if (pipe == 0) {
		memcpy(&nrf24l01_addr0, addr, NRF24L01_ADDRSIZE); //cache address
		uint8_t addrrev[NRF24L01_ADDRSIZE];
		nrf24l01_revaddress(addr, (uint8_t *) addrrev);
		nrf24l01_writeregisters(NRF24L01_REG_RX_ADDR_P0, addrrev,
				NRF24L01_ADDRSIZE);
	} else if (pipe == 1) {
		memcpy(&nrf24l01_addr1, addr, NRF24L01_ADDRSIZE); //cache address
		uint8_t addrrev[NRF24L01_ADDRSIZE];
		nrf24l01_revaddress(addr, (uint8_t *) addrrev);
		nrf24l01_writeregisters(NRF24L01_REG_RX_ADDR_P1, addrrev,
				NRF24L01_ADDRSIZE);
	} else if (pipe == 2) {
		memcpy(&nrf24l01_addr2, addr, NRF24L01_ADDRSIZE); //cache address
		nrf24l01_writeregister(NRF24L01_REG_RX_ADDR_P2,
				addr[NRF24L01_ADDRSIZE - 1]); //write only LSB MSBytes are equal to RX_ADDR_P
	} else if (pipe == 3) {
		memcpy(&nrf24l01_addr3, addr, NRF24L01_ADDRSIZE); //cache address
		nrf24l01_writeregister(NRF24L01_REG_RX_ADDR_P3,
				addr[NRF24L01_ADDRSIZE - 1]); //write only LSB MSBytes are equal to RX_ADDR_P
	} else if (pipe == 4) {
		memcpy(&nrf24l01_addr4, addr, NRF24L01_ADDRSIZE); //cache address
		nrf24l01_writeregister(NRF24L01_REG_RX_ADDR_P4,
				addr[NRF24L01_ADDRSIZE - 1]); //write only LSB MSBytes are equal to RX_ADDR_P
	} else if (pipe == 5) {
		memcpy(&nrf24l01_addr5, addr, NRF24L01_ADDRSIZE); //cache address
		nrf24l01_writeregister(NRF24L01_REG_RX_ADDR_P5,
				addr[NRF24L01_ADDRSIZE - 1]); //write only LSB MSBytes are equal to RX_ADDR_P
	}
}

/*
 * set tx address
 */
void nrf24l01_settxaddr(uint8_t *addr) {
	memcpy(&nrf24l01_addrtx, addr, NRF24L01_ADDRSIZE); //cache address
	uint8_t addrrev[NRF24L01_ADDRSIZE];
	nrf24l01_revaddress(addr, (uint8_t *) addrrev);
	nrf24l01_writeregisters(NRF24L01_REG_RX_ADDR_P0, addrrev,
			NRF24L01_ADDRSIZE); //set rx address for ack on pipe 0
	nrf24l01_writeregisters(NRF24L01_REG_TX_ADDR, addrrev, NRF24L01_ADDRSIZE); //set tx address
}

/*
 * flush RX fifo
 */
void nrf24l01_flushRXfifo() {
	nrf24l01_CSNlo
	; //low CSN
	spi_writereadbyte(NRF24L01_CMD_FLUSH_RX);
	nrf24l01_CSNhi
	; //high CSN
}

/*
 * flush RX fifo
 */
void nrf24l01_flushTXfifo() {
	nrf24l01_CSNlo
	; //low CSN
	spi_writereadbyte(NRF24L01_CMD_FLUSH_TX);
	nrf24l01_CSNhi
	; //high CSN
}

/*
 * set chip as RX
 */
void nrf24l01_setRX() {
	nrf24l01_setrxaddr(0, nrf24l01_addr0); //restore pipe 0 address
	nrf24l01_writeregister(NRF24L01_REG_CONFIG,
			nrf24l01_readregister(NRF24L01_REG_CONFIG)
					| (1 << NRF24L01_REG_PRIM_RX)); //prx mode
	nrf24l01_writeregister(NRF24L01_REG_CONFIG,
			nrf24l01_readregister(NRF24L01_REG_CONFIG)
					| (1 << NRF24L01_REG_PWR_UP)); //power up
	nrf24l01_writeregister(NRF24L01_REG_STATUS,
			(1 << NRF24L01_REG_RX_DR) | (1 << NRF24L01_REG_TX_DS)
					| (1 << NRF24L01_REG_MAX_RT)); //reset status
	nrf24l01_flushRXfifo(); //flush rx fifo
	nrf24l01_flushTXfifo(); //flush tx fifo
	nrf24l01_CEhi
	; //start listening
	_delay_us(150); //wait for the radio to power up
}

/*
 * set chip as TX
 */
void nrf24l01_setTX() {
	nrf24l01_CElo
	; //stop listening
	nrf24l01_writeregister(NRF24L01_REG_CONFIG,
			nrf24l01_readregister(NRF24L01_REG_CONFIG)
					& ~(1 << NRF24L01_REG_PRIM_RX)); //ptx mode
	nrf24l01_writeregister(NRF24L01_REG_CONFIG,
			nrf24l01_readregister(NRF24L01_REG_CONFIG)
					| (1 << NRF24L01_REG_PWR_UP)); //power up
	nrf24l01_writeregister(NRF24L01_REG_STATUS,
			(1 << NRF24L01_REG_RX_DR) | (1 << NRF24L01_REG_TX_DS)
					| (1 << NRF24L01_REG_MAX_RT)); //reset status
	nrf24l01_flushTXfifo(); //flush tx fifo
	_delay_us(150); //wait for the radio to power up
}

#if NRF24L01_PRINTENABLE == 1
/*
 * print info
 */
void nrf24l01_printinfo(void (*prints)(const char *),
		void (*printc)(unsigned char data)) {
	char buff[100];
	prints("info\r\n");
	sprintf(buff, "STATUS: %02X\r\n", nrf24l01_getstatus());
	prints(buff);
	sprintf(buff, "CONFIG: %02X\r\n",
			nrf24l01_readregister(NRF24L01_REG_CONFIG));
	prints(buff);
	sprintf(buff, "RF_CH: %02X\r\n", nrf24l01_readregister(NRF24L01_REG_RF_CH));
	prints(buff);
	sprintf(buff, "RF_SETUP: %02X\r\n",
			nrf24l01_readregister(NRF24L01_REG_RF_SETUP));
	prints(buff);
	sprintf(buff, "EN_AA: %02X\r\n", nrf24l01_readregister(NRF24L01_REG_EN_AA));
	prints(buff);
	sprintf(buff, "EN_RXADDR: %02X\r\n",
			nrf24l01_readregister(NRF24L01_REG_EN_RXADDR));
	prints(buff);
	sprintf(buff, "OBSERVE_TX: %02X\r\n",
			nrf24l01_readregister(NRF24L01_REG_OBSERVE_TX));
	prints(buff);
	prints("\r\n");
}
#endif

/*
 * get status register
 */
uint8_t nrf24l01_getstatus() {
	uint8_t status = 0;
	nrf24l01_CSNlo
	; //low CSN
	status = spi_writereadbyte(NRF24L01_CMD_NOP); //get status, send NOP request
	nrf24l01_CSNhi
	; //high CSN
	return status;
}

/*
 * check if there is data ready
 */
uint8_t nrf24l01_readready(uint8_t* pipe) {
	uint8_t status = nrf24l01_getstatus();
	uint8_t ret = status & (1 << NRF24L01_REG_RX_DR);
	if (ret) {
		//get the pipe number
		if (pipe)
			*pipe = (status >> NRF24L01_REG_RX_P_NO) & 0b111;
	}
	return ret;
}

/*
 * get data
 */
void nrf24l01_read(uint8_t *data, uint8_t len) {
	uint8_t* current = data;
	uint8_t data_len = len < NRF24L01_PAYLOAD ? len : NRF24L01_PAYLOAD;
	uint8_t blank_len =
			nrf24l01_dynamic_payloads_enabled ? 0 : NRF24L01_PAYLOAD - data_len;
	nrf24l01_CSNlo
	;
	spi_writereadbyte(NRF24L01_CMD_R_RX_PAYLOAD);
	while (data_len--) {
		uint8_t b = spi_writereadbyte(0xFF);
		*current++ = b;
	}
	while (blank_len--)
		spi_writereadbyte(0xFF);
	nrf24l01_CSNhi
	;
	//reset register
	nrf24l01_writeregister(NRF24L01_REG_STATUS, (1 << NRF24L01_REG_RX_DR));
	//handle ack payload receipt
	if (nrf24l01_getstatus() & (1 << NRF24L01_REG_TX_DS))
		nrf24l01_writeregister(NRF24L01_REG_STATUS, (1 << NRF24L01_REG_TX_DS));
}

/*
 * put data
 */
uint8_t nrf24l01_write(uint8_t *data, uint8_t len) {
	uint8_t ret = 0;

	nrf24l01_setTX();

	const uint8_t* current = data;
	uint8_t data_len = len < NRF24L01_PAYLOAD ? len : NRF24L01_PAYLOAD;
	uint8_t blank_len =
			nrf24l01_dynamic_payloads_enabled ? 0 : NRF24L01_PAYLOAD - data_len;

	nrf24l01_CSNlo
	;
	spi_writereadbyte(NRF24L01_CMD_W_TX_PAYLOAD);
	while (data_len--)
		spi_writereadbyte(*current++);
	while (blank_len--)
		spi_writereadbyte(0);
	nrf24l01_CSNhi
	;
	// Allons!
	nrf24l01_CEhi
	;
	_delay_us(10);
	nrf24l01_CElo
	;

	unsigned long _cycles = nrf24l01_getmaxtimeout();

	while ((!(nrf24l01_getstatus()
			& (1 << NRF24L01_REG_MAX_RT | 1 << NRF24L01_REG_TX_DS)))
			&& (_cycles > 0)) {
		_delay_ms(1);
		_cycles--;
	}

	if (nrf24l01_getstatus() & 1 << NRF24L01_REG_TX_DS)
		ret = 1;

	//reset PLOS_CNT
	nrf24l01_writeregister(NRF24L01_REG_RF_CH, NRF24L01_CH);

	//power down
	nrf24l01_writeregister(NRF24L01_REG_CONFIG,
			nrf24l01_readregister(NRF24L01_REG_CONFIG)
					& ~(1 << NRF24L01_REG_PWR_UP));

	//set rx mode
	nrf24l01_setRX();

	return ret;
}

/*
 * set power level
 */
void nrf24l01_setpalevel() {
	uint8_t setup = nrf24l01_readregister(NRF24L01_REG_RF_SETUP);
	setup &=
			~((1 << NRF24L01_REG_RF_PWR_LOW) | (1 << NRF24L01_REG_RF_PWR_HIGH));

	if (NRF24L01_RF24_PA == NRF24L01_RF24_PA_MAX) {
		setup |= (1 << NRF24L01_REG_RF_PWR_LOW)
				| (1 << NRF24L01_REG_RF_PWR_HIGH);
	} else if (NRF24L01_RF24_PA == NRF24L01_RF24_PA_HIGH) {
		setup |= (1 << NRF24L01_REG_RF_PWR_HIGH);
	} else if (NRF24L01_RF24_PA == NRF24L01_RF24_PA_LOW) {
		setup |= (1 << NRF24L01_REG_RF_PWR_LOW);
	} else if (NRF24L01_RF24_PA == NRF24L01_RF24_PA_MIN) {
	} else {
		//default is max power
		setup |= (1 << NRF24L01_REG_RF_PWR_LOW)
				| (1 << NRF24L01_REG_RF_PWR_HIGH);
	}

	nrf24l01_writeregister(NRF24L01_REG_RF_SETUP, setup);
}

/*
 * set datarate
 */
void nrf24l01_setdatarate() {
	uint8_t setup = nrf24l01_readregister(NRF24L01_REG_RF_SETUP);

	setup &= ~((1 << NRF24L01_REG_RF_DR_LOW) | (1 << NRF24L01_REG_RF_DR_HIGH));
	if (NRF24L01_RF24_SPEED == NRF24L01_RF24_SPEED_250KBPS) {
		setup |= (1 << NRF24L01_REG_RF_DR_LOW);
	} else {
		if (NRF24L01_RF24_SPEED == NRF24L01_RF24_SPEED_2MBPS) {
			setup |= (1 << NRF24L01_REG_RF_DR_HIGH);
		} else if (NRF24L01_RF24_SPEED == NRF24L01_RF24_SPEED_2MBPS) {
		} else {
			//default is 1Mbps
		}
	}

	nrf24l01_writeregister(NRF24L01_REG_RF_SETUP, setup);
}

/*
 * set crc length
 */
void nrf24l01_setcrclength() {
	uint8_t config = nrf24l01_readregister(NRF24L01_REG_CONFIG)
			& ~((1 << NRF24L01_REG_CRCO) | (1 << NRF24L01_REG_EN_CRC));

	if (NRF24L01_RF24_CRC == NRF24L01_RF24_CRC_DISABLED) {
		//nothing
	} else if (NRF24L01_RF24_CRC == NRF24L01_RF24_CRC_8) {
		config |= (1 << NRF24L01_REG_EN_CRC);
	} else if (NRF24L01_RF24_CRC == NRF24L01_RF24_CRC_16) {
		config |= (1 << NRF24L01_REG_EN_CRC);
		config |= (1 << NRF24L01_REG_CRCO);
	} else {
		//default is disabled
	}

	nrf24l01_writeregister(NRF24L01_REG_CONFIG, config);
}

/*
 * init nrf24l01
 */
void nrf24l01_init() {
	//setup port
	NRF24L01_DDR |= (1 << NRF24L01_CSN); //output
	NRF24L01_DDR |= (1 << NRF24L01_CE); //output

	spi_init(); //init spi

	nrf24l01_CElo
	; //low CE
	nrf24l01_CSNhi
	; //high CSN

	_delay_ms(5); //wait for the radio to init

	nrf24l01_setpalevel(); //set power level
	nrf24l01_setdatarate(); //set data rate
	nrf24l01_setcrclength(); //set crc length
	nrf24l01_writeregister(NRF24L01_REG_SETUP_RETR, NRF24L01_RETR); // set retries
	nrf24l01_writeregister(NRF24L01_REG_DYNPD, 0); //disable dynamic payloads
	nrf24l01_writeregister(NRF24L01_REG_RF_CH, NRF24L01_CH); //set RF channel

	//payload size
#if NRF24L01_ENABLEDP0 == 1
	nrf24l01_writeregister(NRF24L01_REG_RX_PW_P0, NRF24L01_PAYLOAD);
#endif
#if NRF24L01_ENABLEDP1 == 1
	nrf24l01_writeregister(NRF24L01_REG_RX_PW_P1, NRF24L01_PAYLOAD);
#endif
#if NRF24L01_ENABLEDP2 == 1
	nrf24l01_writeregister(NRF24L01_REG_RX_PW_P2, NRF24L01_PAYLOAD);
#endif
#if NRF24L01_ENABLEDP3 == 1
	nrf24l01_writeregister(NRF24L01_REG_RX_PW_P3, NRF24L01_PAYLOAD);
#endif
#if NRF24L01_ENABLEDP4 == 1
	nrf24l01_writeregister(NRF24L01_REG_RX_PW_P4, NRF24L01_PAYLOAD);
#endif
#if NRF24L01_ENABLEDP5 == 1
	nrf24l01_writeregister(NRF24L01_REG_RX_PW_P5, NRF24L01_PAYLOAD);
#endif

	//enable pipe
	nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR, 0);
#if NRF24L01_ENABLEDP0 == 1
	nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR,
			nrf24l01_readregister(NRF24L01_REG_EN_RXADDR)
					| (1 << NRF24L01_REG_ERX_P0));
#endif
#if NRF24L01_ENABLEDP1 == 1
	nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR,
			nrf24l01_readregister(NRF24L01_REG_EN_RXADDR)
					| (1 << NRF24L01_REG_ERX_P1));
#endif
#if NRF24L01_ENABLEDP2 == 1
	nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR,
			nrf24l01_readregister(NRF24L01_REG_EN_RXADDR)
					| (1 << NRF24L01_REG_ERX_P2));
#endif
#if NRF24L01_ENABLEDP3 == 1
	nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR,
			nrf24l01_readregister(NRF24L01_REG_EN_RXADDR)
					| (1 << NRF24L01_REG_ERX_P3));
#endif
#if NRF24L01_ENABLEDP4 == 1
	nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR,
			nrf24l01_readregister(NRF24L01_REG_EN_RXADDR)
					| (1 << NRF24L01_REG_ERX_P4));
#endif
#if NRF24L01_ENABLEDP5 == 1
	nrf24l01_writeregister(NRF24L01_REG_EN_RXADDR,
			nrf24l01_readregister(NRF24L01_REG_EN_RXADDR)
					| (1 << NRF24L01_REG_ERX_P5));
#endif

	//auto ack
#if NRF24L01_ACK == 1
	nrf24l01_writeregister(NRF24L01_REG_EN_AA,
			nrf24l01_readregister(NRF24L01_REG_EN_AA)
					| (1 << NRF24L01_REG_ENAA_P0));
	nrf24l01_writeregister(NRF24L01_REG_EN_AA,
			nrf24l01_readregister(NRF24L01_REG_EN_AA)
					| (1 << NRF24L01_REG_ENAA_P1));
	nrf24l01_writeregister(NRF24L01_REG_EN_AA,
			nrf24l01_readregister(NRF24L01_REG_EN_AA)
					| (1 << NRF24L01_REG_ENAA_P2));
	nrf24l01_writeregister(NRF24L01_REG_EN_AA,
			nrf24l01_readregister(NRF24L01_REG_EN_AA)
					| (1 << NRF24L01_REG_ENAA_P3));
	nrf24l01_writeregister(NRF24L01_REG_EN_AA,
			nrf24l01_readregister(NRF24L01_REG_EN_AA)
					| (1 << NRF24L01_REG_ENAA_P4));
	nrf24l01_writeregister(NRF24L01_REG_EN_AA,
			nrf24l01_readregister(NRF24L01_REG_EN_AA)
					| (1 << NRF24L01_REG_ENAA_P5));
#else
	nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) & ~(1<<NRF24L01_REG_ENAA_P0));
	nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) & ~(1<<NRF24L01_REG_ENAA_P1));
	nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) & ~(1<<NRF24L01_REG_ENAA_P2));
	nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) & ~(1<<NRF24L01_REG_ENAA_P3));
	nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) & ~(1<<NRF24L01_REG_ENAA_P4));
	nrf24l01_writeregister(NRF24L01_REG_EN_AA, nrf24l01_readregister(NRF24L01_REG_EN_AA) & ~(1<<NRF24L01_REG_ENAA_P5));
#endif

	//rx address
	nrf24l01_setrxaddr(0, nrf24l01_addr0);
	nrf24l01_setrxaddr(1, nrf24l01_addr1);
	nrf24l01_setrxaddr(2, nrf24l01_addr2);
	nrf24l01_setrxaddr(3, nrf24l01_addr3);
	nrf24l01_setrxaddr(4, nrf24l01_addr4);
	nrf24l01_setrxaddr(5, nrf24l01_addr5);

	//tx address
	nrf24l01_settxaddr(nrf24l01_addrtx);

	//set rx mode
	nrf24l01_setRX();
}

uint8_t nrf24l01_getretries(void) {
	return nrf24l01_readregister( NRF24L01_REG_SETUP_RETR);
}

uint16_t nrf24l01_getmaxtimeout(void) {
	uint8_t retries = nrf24l01_getretries();
	uint16_t to = ((250 + (250 * ((retries & 0xf0) >> 4))) * (retries & 0x0f));

	return to;

}

void nrf24l01_enabledynamicpayloads(void) {
	// Enable dynamic payload throughout the system
	nrf24l01_writeregister(NRF24L01_REG_FEATURE,
			nrf24l01_readregister(
					NRF24L01_REG_FEATURE) | _BV(NRF24L01_REG_EN_DPL));

	// If it didn't work, the features are not enabled
	if (!nrf24l01_readregister(NRF24L01_REG_FEATURE)) {
		// So enable them and try again
		nrf24l01_CSNlo
		;
		spi_writereadbyte(NRF24L01_CMD_ACTIVATE);
		spi_writereadbyte(0x73);
		nrf24l01_CSNhi
		;
		nrf24l01_writeregister(NRF24L01_REG_FEATURE,
				nrf24l01_readregister(
						NRF24L01_REG_FEATURE) | _BV(NRF24L01_REG_EN_DPL));
	}

	// Enable dynamic payload on all pipes
	//
	// Not sure the use case of only having dynamic payload on certain
	// pipes, so the library does not support it.
	nrf24l01_writeregister(NRF24L01_REG_DYNPD,
			nrf24l01_readregister(
					NRF24L01_REG_DYNPD) | _BV(NRF24L01_REG_DPL_P5) | _BV(NRF24L01_REG_DPL_P4) | _BV(NRF24L01_REG_DPL_P3) | _BV(NRF24L01_REG_DPL_P2) | _BV(NRF24L01_REG_DPL_P1) | _BV(NRF24L01_REG_DPL_P0));

	nrf24l01_dynamic_payloads_enabled = 1;
}

uint8_t nrf24l01_getdynamicpayloadsize(void) {
	uint8_t result = 0;

	nrf24l01_CSNlo
	;
	spi_writereadbyte(NRF24L01_CMD_R_RX_PL_WID);
	result = spi_writereadbyte(0xff);
	nrf24l01_CSNhi
	;
//  csn(LOW);
//  SPI.transfer( R_RX_PL_WID );
//  result = SPI.transfer(0xff);
//  csn(HIGH);
//
	return result;
}

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
ISR(TIM0_OVF_vect)
#else
ISR(TIMER0_OVF_vect)
#endif
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

unsigned long millis() {
	unsigned long m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer0_millis;
	SREG = oldSREG;

	return m;
}
