//-----------------------------------------------------------------------------
// Proxmark5 <-> Flipper Zero CEP (handshake + SPI NG framing)
//-----------------------------------------------------------------------------
#include "pm5_f0_cep.h"

#include <string.h>

#include "at32f435_437.h"
#include "at32f435_437_crm.h"
#include "at32f435_437_gpio.h"
#include "at32f435_437_misc.h"
#include "at32f435_437_spi.h"
#include "at32f435_437_usart.h"

#include "cmd.h"
#include "gpio_apis.h"
#include "proxmark3_arm.h"
#include "ticks_apis.h"

#define PM5_F0_HANDSHAKE_MSG "iamf0rupm5"
#define PM5_F0_HANDSHAKE_LEN 10
#define PM5_F0_SPI_MAX_PAYLOAD (PM3_CMD_DATA_SIZE * 2)

static bool s_inited = false;
static bool s_ready = false;
static uint8_t s_rx_buf[16];
static uint8_t s_rx_len = 0;

// Remaining bytes of current SPI payload (served from s_spi_frame_buf)
static uint16_t s_spi_payload_left = 0;
static uint8_t s_spi_frame_buf[PM5_F0_SPI_MAX_PAYLOAD];
static uint16_t s_spi_frame_len = 0;
static uint16_t s_spi_frame_pos = 0;

// IRQ-filled ring: Flipper clocks faster than the main loop can poll the 4-byte FIFO
#define CEP_RING_SIZE 1024
#define CEP_RING_MASK (CEP_RING_SIZE - 1)
static volatile uint8_t s_ring[CEP_RING_SIZE];
static volatile uint16_t s_ring_w = 0;
static volatile uint16_t s_ring_r = 0;

#define CEP_TX_SIZE (PM5_F0_SPI_MAX_PAYLOAD + 4)
static uint8_t s_tx_buf[CEP_TX_SIZE];
static volatile uint16_t s_tx_len = 0;
static volatile uint16_t s_tx_pos = 0;
static volatile bool s_tx_active = false;

void SPI1_IRQHandler(void) {
    if(spi_i2s_flag_get(SPI1, SPI_I2S_ROERR_FLAG) != RESET) {
        spi_i2s_flag_clear(SPI1, SPI_I2S_ROERR_FLAG);
    }
    while(spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) != RESET) {
        uint8_t b = (uint8_t)spi_i2s_data_receive(SPI1);
        uint16_t next = (uint16_t)((s_ring_w + 1) & CEP_RING_MASK);
        if(next != s_ring_r) {
            s_ring[s_ring_w] = b;
            s_ring_w = next;
        }
    }

    if(s_tx_active && spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) != RESET) {
        spi_i2s_data_transmit(SPI1, s_tx_buf[s_tx_pos++]);
        if(s_tx_pos >= s_tx_len) {
            s_tx_active = false;
            spi_i2s_interrupt_enable(SPI1, SPI_I2S_TDBE_INT, FALSE);
        }
    }
}

static uint16_t cep_ring_count(void) {
    return (uint16_t)((s_ring_w - s_ring_r) & CEP_RING_MASK);
}

static uint8_t cep_ring_peek(uint16_t offset) {
    return s_ring[(s_ring_r + offset) & CEP_RING_MASK];
}

static uint8_t cep_ring_pop(void) {
    uint8_t b = s_ring[s_ring_r];
    s_ring_r = (uint16_t)((s_ring_r + 1) & CEP_RING_MASK);
    return b;
}

static void cep_ring_reset(void) {
    s_ring_r = s_ring_w;
}

bool pm5_f0_cep_is_ready(void) {
    return s_ready;
}

void pm5_f0_cep_init(void) {
    if(s_inited) {
        return;
    }

    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_7);

    usart_init(USART1, 2400, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_parity_selection_config(USART1, USART_PARITY_NONE);
    usart_transmitter_enable(USART1, FALSE);
    usart_receiver_enable(USART1, TRUE);
    usart_single_line_halfduplex_select(USART1, TRUE);
    usart_enable(USART1, TRUE);

    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init(GPIOA, &gpio_init_struct);

    // Flipper is CEP host → PM5 SPI slave
    gpio_inter_usb_spi_role_setup();
    Gpio_Inter_USB_SPI_Role_High();

    gpio_init_type gpio_initstructure;
    gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_initstructure.gpio_pull = GPIO_PULL_UP;
    gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_initstructure.gpio_mode = GPIO_MODE_MUX;
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE4, GPIO_MUX_5);
    gpio_initstructure.gpio_pins = GPIO_PINS_4;
    gpio_init(GPIOA, &gpio_initstructure);

    gpio_initstructure.gpio_pull = GPIO_PULL_DOWN;
    gpio_initstructure.gpio_pins = GPIO_PINS_5;
    gpio_init(GPIOA, &gpio_initstructure);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE5, GPIO_MUX_5);

    gpio_initstructure.gpio_pull = GPIO_PULL_UP;
    gpio_initstructure.gpio_pins = GPIO_PINS_6;
    gpio_init(GPIOA, &gpio_initstructure);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE6, GPIO_MUX_5);

    gpio_initstructure.gpio_pins = GPIO_PINS_7;
    gpio_init(GPIOA, &gpio_initstructure);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_5);

    spi_init_type spi_init_struct;
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    spi_default_para_init(&spi_init_struct);
    spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode = SPI_MODE_SLAVE;
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_1024;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_1EDGE;
    spi_init_struct.cs_mode_selection = SPI_CS_HARDWARE_MODE;
    spi_init(SPI1, &spi_init_struct);
    spi_enable(SPI1, TRUE);
    spi_i2s_interrupt_enable(SPI1, SPI_I2S_RDBF_INT, TRUE);
    nvic_irq_enable(SPI1_IRQn, 1, 0);

    s_rx_len = 0;
    s_spi_payload_left = 0;
    s_ring_w = 0;
    s_ring_r = 0;
    s_tx_len = 0;
    s_tx_pos = 0;
    s_tx_active = false;
    s_ready = false;
    s_inited = true;
}

static void pm5_f0_cep_send_yes(void) {
    const uint8_t response_f0[] = {0x04, 0x00, 'y', 'e', 's', 0x00};
    for(size_t i = 0; i < sizeof(response_f0); i++) {
        uint32_t t0 = GetTicks();
        while(spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET) {
            if(GetTicks() - t0 > 1000 * 1000) {
                return;
            }
        }
        spi_i2s_data_transmit(SPI1, response_f0[i]);
    }
}

bool pm5_f0_cep_poll_handshake(void) {
    if(!s_inited) {
        return false;
    }

    if(usart_flag_get(USART1, USART_FERR_FLAG) != RESET ||
            usart_flag_get(USART1, USART_NERR_FLAG) != RESET ||
            usart_flag_get(USART1, USART_ROERR_FLAG) != RESET) {
        (void)USART1->dt;
        s_rx_len = 0;
    }

    while(usart_flag_get(USART1, USART_RDBF_FLAG) != RESET) {
        uint8_t rx_data = (uint8_t)usart_data_receive(USART1);
        if(rx_data == 0x02) {
            // A reopened Flipper app starts a new session. The PM5 may still
            // be powered over PC USB, so explicitly reset the old CEP state.
            s_ready = false;
            s_rx_len = 0;
            s_spi_payload_left = 0;
            cep_ring_reset();
            spi_i2s_interrupt_enable(SPI1, SPI_I2S_TDBE_INT, FALSE);
            s_tx_active = false;
            s_tx_len = 0;
            s_tx_pos = 0;
            continue;
        }

        if(s_rx_len < sizeof(s_rx_buf)) {
            s_rx_buf[s_rx_len++] = rx_data;
        } else {
            s_rx_len = 0;
        }

        if(s_rx_len == PM5_F0_HANDSHAKE_LEN) {
            if(memcmp(PM5_F0_HANDSHAKE_MSG, s_rx_buf, PM5_F0_HANDSHAKE_LEN) == 0) {
                pm5_f0_cep_send_yes();
                cep_ring_reset();
                s_spi_payload_left = 0;
                s_ready = true;
                s_rx_len = 0;
                return true;
            }
            memmove(s_rx_buf, s_rx_buf + 1, PM5_F0_HANDSHAKE_LEN - 1);
            s_rx_len = PM5_F0_HANDSHAKE_LEN - 1;
        }
    }

    return false;
}

// --- SPI NG transport used by cmd.c (length-prefixed frames) ---

static bool cep_ring_is_pm3a(uint16_t off) {
    return cep_ring_peek(off) == 0x50 && cep_ring_peek(off + 1) == 0x4d &&
           cep_ring_peek(off + 2) == 0x33 && cep_ring_peek(off + 3) == 0x61;
}

static void cep_take_frame(uint16_t skip, uint16_t data_len, bool has_len_prefix) {
    for(uint16_t i = 0; i < skip; i++) {
        (void)cep_ring_pop();
    }
    if(has_len_prefix) {
        (void)cep_ring_pop();
        (void)cep_ring_pop();
    }
    for(uint16_t i = 0; i < data_len; i++) {
        s_spi_frame_buf[i] = cep_ring_pop();
    }
    if(cep_ring_count() > 0 && cep_ring_peek(0) == 0x00) {
        (void)cep_ring_pop();
    }
    s_spi_frame_len = data_len;
    s_spi_frame_pos = 0;
    s_spi_payload_left = data_len;
    s_ready = true;
}

bool cep_spi_data_available(void) {
    if(s_spi_payload_left > 0) {
        return true;
    }

    // Drop idle clocks (Flipper TX dummy 0x00) before looking for a frame
    while(cep_ring_count() > 0 && cep_ring_peek(0) == 0x00) {
        (void)cep_ring_pop();
    }

    uint16_t n = cep_ring_count();
    if(n < 8) {
        return false;
    }

    // Fast path: [u16le len][PM3a ...]
    uint16_t data_len = (uint16_t)cep_ring_peek(0) | ((uint16_t)cep_ring_peek(1) << 8);
    if(data_len >= 8 && data_len <= 600 && n >= (uint16_t)(2 + data_len) && cep_ring_is_pm3a(2)) {
        cep_take_frame(0, data_len, true);
        s_ready = true;
        return true;
    }

    // Scan for PM3a — Flipper may have prepended dummy clocks
    for(uint16_t i = 0; i + 4 <= n; i++) {
        if(!cep_ring_is_pm3a(i)) {
            continue;
        }
        if(i >= 2) {
            data_len = (uint16_t)cep_ring_peek(i - 2) | ((uint16_t)cep_ring_peek(i - 1) << 8);
            if(data_len >= 8 && data_len <= 600 && n >= (uint16_t)(i - 2 + 2 + data_len)) {
                cep_take_frame((uint16_t)(i - 2), data_len, true);
                return true;
            }
        }
        // No usable length prefix: size from NG header (len:15 at offset 4)
        uint16_t ng_len = (uint16_t)cep_ring_peek(i + 4) | ((uint16_t)(cep_ring_peek(i + 5) & 0x7F) << 8);
        uint16_t need = (uint16_t)(8 + ng_len + 2);
        if(ng_len <= 512 && n >= (uint16_t)(i + need)) {
            cep_take_frame(i, need, false);
            return true;
        }
        return false; // magic seen, frame still arriving
    }

    // Stuck garbage: drop one byte so we can resync
    if(n > 32) {
        (void)cep_ring_pop();
    }
    return false;
}

uint32_t cep_spi_read_ng(uint8_t *data, size_t len) {
    size_t got = 0;
    while(got < len && s_spi_payload_left > 0 && s_spi_frame_pos < s_spi_frame_len) {
        data[got++] = s_spi_frame_buf[s_spi_frame_pos++];
        s_spi_payload_left--;
    }
    return (uint32_t)got;
}

int cep_spi_write_sync(uint8_t *data, size_t len) {
    if(data == NULL || len == 0 || len + 4 > CEP_TX_SIZE) {
        return PM3_EIO;
    }

    s_ready = true;

    // Queue the complete response. TDBE IRQ feeds bytes as the Flipper clocks
    // them, so AppMain never blocks waiting for CS or starves USB.
    spi_i2s_interrupt_enable(SPI1, SPI_I2S_TDBE_INT, FALSE);
    s_tx_buf[0] = (uint8_t)(len & 0xFF);
    s_tx_buf[1] = (uint8_t)((len >> 8) & 0xFF);
    memcpy(s_tx_buf + 2, data, len);
    s_tx_buf[2 + len] = 0x00;
    s_tx_buf[3 + len] = 0x00;
    s_tx_len = (uint16_t)(len + 4);
    s_tx_pos = 0;
    s_tx_active = true;
    spi_i2s_interrupt_enable(SPI1, SPI_I2S_TDBE_INT, TRUE);
    return PM3_SUCCESS;
}
