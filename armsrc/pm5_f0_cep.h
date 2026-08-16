//-----------------------------------------------------------------------------
// Proxmark5 <-> Flipper Zero (F0) CEP Type-C handshake (experimental)
// UART: Flipper sends STX + "iamf0rupm5"
// SPI:  PM5 replies length-prefixed "yes"
//-----------------------------------------------------------------------------
#ifndef PM5_F0_CEP_H__
#define PM5_F0_CEP_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

void pm5_f0_cep_init(void);
// Non-blocking: call from AppMain loop. Returns true once handshake succeeded.
bool pm5_f0_cep_poll_handshake(void);
bool pm5_f0_cep_is_ready(void);

// SPI length-prefixed NG transport (Flipper CEP)
bool cep_spi_data_available(void);
uint32_t cep_spi_read_ng(uint8_t *data, size_t len);
int cep_spi_write_sync(uint8_t *data, size_t len);

#endif
