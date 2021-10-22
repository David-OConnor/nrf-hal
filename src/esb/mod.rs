//! Implementation of the Enhanced ShockBurst (ESB) proprieary Nordic RF protocol.
//! Adapted from the nRF5-SDK, Version 17.10. https://www.nordicsemi.com/Products/Development-software/nrf5-sdk
//!
//! We generally keep the original (C) naming conventions and comment syntax.

//! This file mirrors `nrf_esb.c`.
//!
//! todo: Release as standalone crate that uses the PAC directly

use cortex_m::peripheral::NVIC;

use crate::pac::{
    Interrupt,
};

use cfg_if::cfg_if;

/**
 * Copyright (c) 2016 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


const BIT_MASK_UINT_8(x) (0xFF >> (8 - (x)))

// Constant parameters
const RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS: u8 =        48;        /**< 2 Mb RX wait for acknowledgment time-out value. Smallest reliable value - 43. */
const RX_WAIT_FOR_ACK_TIMEOUT_US_1MBPS : u8 =       73;        /**< 1 Mb RX wait for acknowledgment time-out value. Smallest reliable value - 68. */
const RX_WAIT_FOR_ACK_TIMEOUT_US_250KBPS : u8 =     250  ;     /**< 250 Kb RX wait for acknowledgment time-out value. */
const RX_WAIT_FOR_ACK_TIMEOUT_US_1MBPS_BLE : u8 =   73 ;       /**< 1 Mb RX wait for acknowledgment time-out combined with BLE. Smallest reliable value - 68.*/
const RETRANSMIT_DELAY_US_OFFSET    : u8 =          62  ;      /**< Never retransmit before the wait for ack time plus this offset. */

// Interrupt flags
const     NRF_ESB_INT_TX_SUCCESS_MSK  : u8 =        0x01  ;      /**< Interrupt mask value for TX success. */
const     NRF_ESB_INT_TX_FAILED_MSK  : u8 =         0x02 ;       /**< Interrupt mask value for TX failure. */
const     NRF_ESB_INT_RX_DATA_RECEIVED_MSK  : u8 =  0x04  ;      /**< Interrupt mask value for RX_DR. */

const     NRF_ESB_PID_RESET_VALUE   : u8 =          0xFF  ;      /**< Invalid PID value which is guaranteed to not collide with any valid PID value. */
const     NRF_ESB_PID_MAX         : u8 =            3    ;      /**< Maximum value for PID. */
const     NRF_ESB_CRC_RESET_VALUE   : u8 =          0xFFFF ;     /**< CRC reset value. */

// Internal Enhanced ShockBurst module state.
enum nrf_esb_mainstate_t {
    NRF_ESB_STATE_IDLE,                                     /**< Module idle. */
    NRF_ESB_STATE_PTX_TX,                                   /**< Module transmitting without acknowledgment. */
    NRF_ESB_STATE_PTX_TX_ACK,                               /**< Module transmitting with acknowledgment. */
    NRF_ESB_STATE_PTX_RX_ACK,                               /**< Module transmitting with acknowledgment and reception of payload with the acknowledgment response. */
    NRF_ESB_STATE_PRX,                                      /**< Module receiving packets without acknowledgment. */
    NRF_ESB_STATE_PRX_SEND_ACK,                             /**< Module transmitting acknowledgment in RX mode. */
}

const _RADIO_SHORTS_COMMON ( RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | \
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk | RADIO_SHORTS_DISABLED_RSSISTOP_Msk )

const VERIFY_PAYLOAD_LENGTH(p)                            \
do                                                          \
{                                                           \
    if (p.length == 0 ||                                   \
       p.length > NRF_ESB_MAX_PAYLOAD_LENGTH ||            \
       (m_config_local.protocol == NRF_ESB_PROTOCOL_ESB &&  \
        p.length > m_config_local.payload_length))         \
    {                                                       \
        return NRF_ERROR_INVALID_LENGTH;                    \
    }                                                       \
}while (0)


/* @brief Structure holding pipe info PID and CRC and acknowledgment payload. */
struct pipe_info_t
{
   crc: u16   ,                                /**< CRC value of the last received packet (Used to detect retransmits). */
   pid: u8     ,                              /**< Packet ID of the last received packet (Used to detect retransmits). */
    ack_payload: bool  ,                            /**< Flag indicating the state of the transmission of acknowledgment payloads. */
}


/* @brief Structure used by the PRX to organize ACK payloads for multiple pipes. */
struct nrf_esb_payload_random_access_buf_wrapper_t
{
    p_payload: &nrf_esb_payload_t,                     /**< Pointer to the ACK payload. */
     in_use: bool,                           /**< Value used to determine if the current payload pointer is used. */
    struct nrf_esb_payload_random_access_buf_wrapper_t * p_next, /**< Pointer to the next ACK payload queued on the same pipe. */
}


/* @brief  First-in, first-out queue of payloads to be transmitted. */
struct nrf_esb_payload_tx_fifo_t
{
    p_payload: [nrf_esb_payload_t; NRF_ESB_TX_FIFO_SIZE],  /**< Pointer to the actual queue. */
    entry_point: u32,                      /**< Current start of queue. */
    exit_point: u32,                       /**< Current end of queue. */
    count: u32,                            /**< Current number of elements in the queue. */
}


/* @brief First-in, first-out queue of received payloads. */
struct nrf_esb_payload_rx_fifo_t
{
    p_payload: [ nrf_esb_payload_t; NRF_ESB_RX_FIFO_SIZE],  /**< Pointer to the actual queue. */
    entry_point: u32   ,                  /**< Current start of queue. */
    exit_point: u32   ,                   /**< Current end of queue. */
    count: u32    ,                      /**< Current number of elements in the queue. */
}


/**@brief Enhanced ShockBurst address.
 *
 * Enhanced ShockBurst addresses consist of a base address and a prefix
 *          that is unique for each pipe. See @ref esb_addressing in the ESB user
 *          guide for more information.
*/
struct nrf_esb_address_t
{
    base_addr_p0: [u8; 4],        /**< Base address for pipe 0 encoded in big endian. */
    base_addr_p1: [u8; 4],        /**< Base address for pipe 1-7 encoded in big endian. */
   pipe_prefixes: [u8; 8],       /**< Address prefix for pipe 0 to 7. */
    num_pipes: u8,              /**< Number of pipes available. */
    addr_length: u8,            /**< Length of the address including the prefix. */
    rx_pipes_enabled: u8,       /**< Bitfield for enabled pipes. */
    rf_channel: u8,             /**< Channel to use (must be between 0 and 100). */
}


// Module state
static                        m_esb_initialized: bool           = false;
static  m_nrf_esb_mainstate: nrf_esb_mainstate_t         = NRF_ESB_STATE_IDLE;
static         mp_current_payload: nrf_esb_payload_t  ;

static       m_event_handler: nrf_esb_event_handler_t;

// Address parameters
__ALIGN(4) static nrf_esb_address_t m_esb_addr = NRF_ESB_ADDR_DEFAULT;

// RF parameters
static             m_config_local: nrf_esb_config_t ;

// TX FIFO
static            m_tx_fifo_payload[NRF_ESB_TX_FIFO_SIZE]: nrf_esb_payload_t ;
static   m_tx_fifo: nrf_esb_payload_tx_fifo_t  ;

// RX FIFO
static            m_rx_fifo_payload[NRF_ESB_RX_FIFO_SIZE]: nrf_esb_payload_t ;
static   m_rx_fifo: nrf_esb_payload_rx_fifo_t  ;

// Payload buffers
static                     m_tx_payload_buffer: [u8; NRF_ESB_MAX_PAYLOAD_LENGTH + 2] = [0; NRF_ESB_MAX_PAYLOAD_LENGTH + 2];
static                     m_rx_payload_buffer: [u8; NRF_ESB_MAX_PAYLOAD_LENGTH + 2] = [0; NRF_ESB_MAX_PAYLOAD_LENGTH + 2];


// Random access buffer variables for better ACK payload handling
nrf_esb_payload_random_access_buf_wrapper_t m_ack_pl_container: [NRF_ESB_TX_FIFO_SIZE];
nrf_esb_payload_random_access_buf_wrapper_t * m_ack_pl_container_entry_point_pr_pipe[NRF_ESB_PIPE_COUNT];


// Run time variables
static            m_interrupt_flags: u32 = 0;
static                    m_pids: [u8; NRF_ESB_PIPE_COUNT] = [0; NRF_ESB_PIPE_COUNT];
static                   m_rx_pipe_info: [pipe_info_t; NRF_ESB_PIPE_COUNT] = [0; NRF_ESB_PIPE_COUNT];
static           m_retransmits_remaining: u32 = 0;
static            m_last_tx_attempts: u32 = 0;
static          m_wait_for_ack_timeout_us: u32 = 0;

static           m_radio_shorts_common: u32 = _RADIO_SHORTS_COMMON;

// These function pointers are changed dynamically, depending on protocol configuration and state.
// fn (*on_radio_disabled)() = 0;
// fn (*on_radio_end)() = 0;
// fn (*update_rf_payload_format)(u32 payload_length) = 0;


// The following functions are assigned to the function pointers above.
// fn on_radio_disabled_tx_noack();
// fn on_radio_disabled_tx();
// fn on_radio_disabled_tx_wait_for_ack();
// fn on_radio_disabled_rx();
// fn on_radio_disabled_rx_ack();


const NRF_ESB_ADDR_UPDATE_MASK_BASE0: u8 = 1 << 0;    /*< Mask value to signal updating BASE0 radio address. */
const NRF_ESB_ADDR_UPDATE_MASK_BASE1: u8 = 1 << 1;    /*< Mask value to signal updating BASE1 radio address. */
const NRF_ESB_ADDR_UPDATE_MASK_PREFIX: u8 = 1 << 2;    /*< Mask value to signal updating radio prefixes. */


// Function to do bytewise bit-swap on an unsigned 32-bit value
fn bytewise_bit_swap(p_inp: *const u8) -> u32
{
#if __CORTEX_M == (0x04U)
    u32 inp = (*(u32*)p_inp);
    return __REV((u32)__RBIT(inp)); //lint -esym(628, __rev) -esym(526, __rev) -esym(628, __rbit) -esym(526, __rbit) */
#else
    u32 inp = (p_inp[3] << 24) | (p_inp[2] << 16) | (p_inp[1] << 8) | (p_inp[0]);
    inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
    inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
    inp = (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
    return inp;
#endif
}


// Convert a base address from nRF24L format to nRF5 format
fn addr_conv(p_addr: *const u8) -> u32
{
    return __REV(bytewise_bit_swap(p_addr)); //lint -esym(628, __rev) -esym(526, __rev) */
}

#[cfg(feature = "52832")]
fn  apply_address_workarounds() -> ret_code_t
{
    if ((NRF_FICR.INFO.VARIANT & 0x0000FF00) == 0x00004200) //Check if the device is an nRF52832 Rev. 1.
    {
        // Workaround for nRF52832 Rev 1 erratas
        //  Set up radio parameters.
        NRF_RADIO.MODECNF0 = (NRF_RADIO.MODECNF0 & ~RADIO_MODECNF0_RU_Msk) | RADIO_MODECNF0_RU_Default << RADIO_MODECNF0_RU_Pos;

        // Workaround for nRF52832 Rev 1 Errata 102 and nRF52832 Rev 1 Errata 106. This will reduce sensitivity by 3dB.
        *((volatile u32 *)0x40001774) = (*((volatile u32 *)0x40001774) & 0xFFFFFFFE) | 0x01000000;
    }

    if ((NRF_FICR.INFO.VARIANT & 0x0000FF00) == 0x00004500)//Check if the device is an nRF52832 Rev. 2.
    {
        /*
        Workaround for nRF52832 Rev 2 Errata 143
        Check if the most significant bytes of address 0 (including prefix) match those of another address.
        It's recommended to use a unique address 0 since this will avoid the 3dBm penalty incurred from the workaround.
        */
        u32 base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;

        // Load the two addresses before comparing them to ensure defined ordering of volatile accesses.
        u32 addr0 = NRF_RADIO.BASE0 & base_address_mask;
        u32 addr1 = NRF_RADIO.BASE1 & base_address_mask;
        if (addr0 == addr1)
        {
            u32 prefix0 = NRF_RADIO.PREFIX0 & 0x000000FF;
            u32 prefix1 = (NRF_RADIO.PREFIX0 & 0x0000FF00) >> 8;
            u32 prefix2 = (NRF_RADIO.PREFIX0 & 0x00FF0000) >> 16;
            u32 prefix3 = (NRF_RADIO.PREFIX0 & 0xFF000000) >> 24;
            u32 prefix4 = NRF_RADIO.PREFIX1 & 0x000000FF;
            u32 prefix5 = (NRF_RADIO.PREFIX1 & 0x0000FF00) >> 8;
            u32 prefix6 = (NRF_RADIO.PREFIX1 & 0x00FF0000) >> 16;
            u32 prefix7 = (NRF_RADIO.PREFIX1 & 0xFF000000) >> 24;

            if (prefix0 == prefix1 || prefix0 == prefix2 || prefix0 == prefix3 || prefix0 == prefix4 ||
                prefix0 == prefix5 || prefix0 == prefix6 || prefix0 == prefix7)
            {
                // This will cause a 3dBm sensitivity loss, avoid using such address combinations if possible.
                *(volatile u32 *) 0x40001774 = ((*(volatile u32 *) 0x40001774) & 0xfffffffe) | 0x01000000;
            }
        }
    }
    return NRF_SUCCESS;
}



fn update_rf_payload_format_esb_dpl(payload_length: u32)
{
    if (NRF_ESB_MAX_PAYLOAD_LENGTH <= 32) {
        // Using 6 bits for length
        NRF_RADIO.PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) |
            (6 << RADIO_PCNF0_LFLEN_Pos) |
            (3 << RADIO_PCNF0_S1LEN_Pos);
    } else {
        // Using 8 bits for length
        NRF_RADIO.PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) |
            (8 << RADIO_PCNF0_LFLEN_Pos) |
            (3 << RADIO_PCNF0_S1LEN_Pos);
    }

    NRF_RADIO.PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos) |
        (RADIO_PCNF1_ENDIAN_Big          << RADIO_PCNF1_ENDIAN_Pos)  |
        ((m_esb_addr.addr_length - 1)    << RADIO_PCNF1_BALEN_Pos)   |
        (0                               << RADIO_PCNF1_STATLEN_Pos) |
        (NRF_ESB_MAX_PAYLOAD_LENGTH      << RADIO_PCNF1_MAXLEN_Pos);
}


fn update_rf_payload_format_esb(payload_length: u32)
{
    NRF_RADIO.PCNF0 = (1 << RADIO_PCNF0_S0LEN_Pos) |
                       (0 << RADIO_PCNF0_LFLEN_Pos) |
                       (1 << RADIO_PCNF0_S1LEN_Pos);

    NRF_RADIO.PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big          << RADIO_PCNF1_ENDIAN_Pos)  |
                       ((m_esb_addr.addr_length - 1)    << RADIO_PCNF1_BALEN_Pos)   |
                       (payload_length                  << RADIO_PCNF1_STATLEN_Pos) |
                       (payload_length                  << RADIO_PCNF1_MAXLEN_Pos);
}


fn update_radio_addresses(update_mask: u8)
{
    if (update_mask & NRF_ESB_ADDR_UPDATE_MASK_BASE0) != 0
    {
        NRF_RADIO.BASE0 = addr_conv(m_esb_addr.base_addr_p0);
    }

    if (update_mask & NRF_ESB_ADDR_UPDATE_MASK_BASE1) != 0
    {
        NRF_RADIO.BASE1 = addr_conv(m_esb_addr.base_addr_p1);
    }

    if (update_mask & NRF_ESB_ADDR_UPDATE_MASK_PREFIX) != 0
    {
        NRF_RADIO.PREFIX0 = bytewise_bit_swap(&m_esb_addr.pipe_prefixes[0]);
        NRF_RADIO.PREFIX1 = bytewise_bit_swap(&m_esb_addr.pipe_prefixes[4]);
    }
}


fn update_radio_tx_power()
{
    NRF_RADIO.TXPOWER = m_config_local.tx_output_power << RADIO_TXPOWER_TXPOWER_Pos;
}


fn update_radio_bitrate() -> bool
{
    NRF_RADIO.MODE = m_config_local.bitrate << RADIO_MODE_MODE_Pos;

    match m_config_local.bitrate
    {
        NRF_ESB_BITRATE_2MBPS =>
#ifdef RADIO_MODE_MODE_Ble_2Mbit
        NRF_ESB_BITRATE_2MBPS_BLE => 
#endif
            m_wait_for_ack_timeout_us = RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS;

        NRF_ESB_BITRATE_1MBPS =>
            m_wait_for_ack_timeout_us = RX_WAIT_FOR_ACK_TIMEOUT_US_1MBPS;

#ifdef RADIO_MODE_MODE_Nrf_250Kbit
        NRF_ESB_BITRATE_250KBPS =>
            m_wait_for_ack_timeout_us = RX_WAIT_FOR_ACK_TIMEOUT_US_250KBPS;
#endif

        NRF_ESB_BITRATE_1MBPS_BLE =>
            m_wait_for_ack_timeout_us = RX_WAIT_FOR_ACK_TIMEOUT_US_1MBPS_BLE;

       _ =>
            // Should not be reached
            return false,
    }

    // Ensure that we do not attempt retransmitting before ack timeout.
    if (m_config_local.retransmit_delay < m_wait_for_ack_timeout_us + RETRANSMIT_DELAY_US_OFFSET)
    {
        m_config_local.retransmit_delay = m_wait_for_ack_timeout_us + RETRANSMIT_DELAY_US_OFFSET;
    }

    return true;
}


fn bool update_radio_protocol() -> bool
{
    switch (m_config_local.protocol)
    {
        case NRF_ESB_PROTOCOL_ESB_DPL:
            update_rf_payload_format = update_rf_payload_format_esb_dpl;
            break;

        case NRF_ESB_PROTOCOL_ESB:
            update_rf_payload_format = update_rf_payload_format_esb;
            break;

        default:
            // Should not be reached
            return false;
    }
    return true;
}


fn update_radio_crc() -> bool
{
    switch(m_config_local.crc)
    {
        case NRF_ESB_CRC_16BIT:
            NRF_RADIO.CRCINIT = 0xFFFFUL;      // Initial value
            NRF_RADIO.CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
            break;

        case NRF_ESB_CRC_8BIT:
            NRF_RADIO.CRCINIT = 0xFFUL;        // Initial value
            NRF_RADIO.CRCPOLY = 0x107UL;       // CRC poly: x^8+x^2^x^1+1
            break;

        case NRF_ESB_CRC_OFF:
            NRF_RADIO.CRCINIT = 0x00UL;
            NRF_RADIO.CRCPOLY = 0x00UL;
            break;

        default:
            return false;
    }
    NRF_RADIO.CRCCNF = m_config_local.crc << RADIO_CRCCNF_LEN_Pos;
    return true;
}


fn update_radio_parameters() -> bool
{
    let mut params_valid = true;
    update_radio_tx_power();
    params_valid &= update_radio_bitrate();
    params_valid &= update_radio_protocol();
    params_valid &= update_radio_crc();
    update_rf_payload_format(m_config_local.payload_length);
    return params_valid;
}


fn reset_fifos()
{
    m_tx_fifo.entry_point = 0;
    m_tx_fifo.exit_point  = 0;
    m_tx_fifo.count       = 0;

    m_rx_fifo.entry_point = 0;
    m_rx_fifo.exit_point  = 0;
    m_rx_fifo.count       = 0;
}


fn initialize_fifos()
{
    reset_fifos();

    for i in 0..NRF_ESB_TX_FIFO_SIZE
    {
        m_tx_fifo.p_payload[i] = &m_tx_fifo_payload[i];
    }

    for i in 0..NRF_ESB_RX_FIFO_SIZE
    {
        m_rx_fifo.p_payload[i] = &m_rx_fifo_payload[i];
    }

    for i in 0..NRF_ESB_TX_FIFO_SIZE {
    {
        m_ack_pl_container[i].p_payload = &m_tx_fifo_payload[i];
        m_ack_pl_container[i].in_use = false;
        m_ack_pl_container[i].p_next = 0;
    }
    for i in 0..NRF_ESB_PIPE_COUNT {
    {
        m_ack_pl_container_entry_point_pr_pipe[i] = 0;
    }
}


/**@brief Function for removing the oldest entry from the TX buffer.
 *
 * This function will remove the next element scheduled to be sent from the TX FIFO queue.
 * This is useful if you want to skip a packet which was never acknowledged.
 *
 * @retval  NRF_SUCCESS                     If the operation completed successfully.
 * @retval  NRF_INVALID_STATE               If the module is not initialized.
 * @retval  NRF_ERROR_BUFFER_EMPTY          If there are no items in the queue to remove.
 */
fn nrf_esb_skip_tx() -> u32
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    VERIFY_TRUE(m_tx_fifo.count > 0, NRF_ERROR_BUFFER_EMPTY);

    unsafe { NVIC::mask(Interrupt::RADIO) }

    m_tx_fifo.count -= 1;
    if ( m_tx_fifo.exit_point += 1 >= NRF_ESB_TX_FIFO_SIZE)
    {
        m_tx_fifo.exit_point = 0;
    }

    unsafe { NVIC::unmask(Interrupt::RADIO) }

    return NRF_SUCCESS;
}

/** @brief  Function to push the content of the rx_buffer to the RX FIFO.
 *
 *  The module will point the register NRF_RADIO.PACKETPTR to a buffer for receiving packets.
 *  After receiving a packet the module will call this function to copy the received data to
 *  the RX FIFO.
 *
 *  @param  pipe Pipe number to set for the packet.
 *  @param  pid  Packet ID.
 *
 *  @retval true   Operation successful.
 *  @retval false  Operation failed.
 */
fn rx_fifo_push_rfbuf(pipe: u8, pid: u8) -> bool
{
    if (m_rx_fifo.count < NRF_ESB_RX_FIFO_SIZE)
    {
        if (m_config_local.protocol == NRF_ESB_PROTOCOL_ESB_DPL)
        {
            if (m_rx_payload_buffer[0] > NRF_ESB_MAX_PAYLOAD_LENGTH)
            {
                return false;
            }

            m_rx_fifo.p_payload[m_rx_fifo.entry_point].length = m_rx_payload_buffer[0];
        }
        else if (m_config_local.mode == NRF_ESB_MODE_PTX)
        {
            // Received packet is an acknowledgment
            m_rx_fifo.p_payload[m_rx_fifo.entry_point].length = 0;
        }
        else
        {
            m_rx_fifo.p_payload[m_rx_fifo.entry_point].length = m_config_local.payload_length;
        }

        memcpy(m_rx_fifo.p_payload[m_rx_fifo.entry_point].data, &m_rx_payload_buffer[2],
               m_rx_fifo.p_payload[m_rx_fifo.entry_point].length);

        m_rx_fifo.p_payload[m_rx_fifo.entry_point].pipe  = pipe;
        m_rx_fifo.p_payload[m_rx_fifo.entry_point].rssi  = NRF_RADIO.RSSISAMPLE;
        m_rx_fifo.p_payload[m_rx_fifo.entry_point].pid   = pid;
        m_rx_fifo.p_payload[m_rx_fifo.entry_point].noack = !(m_rx_payload_buffer[1] & 0x01);
        if ( m_rx_fifo.entry_point += 1 >= NRF_ESB_RX_FIFO_SIZE)
        {
            m_rx_fifo.entry_point = 0;
        }
        m_rx_fifo.count += 1;

        return true;
    }

    return false;
}


fn sys_timer_init()
{
    // Configure the system timer with a 1 MHz base frequency
    NRF_ESB_SYS_TIMER.PRESCALER = 4;
    NRF_ESB_SYS_TIMER.BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
    NRF_ESB_SYS_TIMER.SHORTS    = TIMER_SHORTS_COMPARE1_CLEAR_Msk | TIMER_SHORTS_COMPARE1_STOP_Msk;
}


fn ppi_init()
{
    NRF_PPI.CH[NRF_ESB_PPI_TIMER_START].EEP = (u32)&NRF_RADIO.EVENTS_READY;
    NRF_PPI.CH[NRF_ESB_PPI_TIMER_START].TEP = (u32)&NRF_ESB_SYS_TIMER.TASKS_START;

    NRF_PPI.CH[NRF_ESB_PPI_TIMER_STOP].EEP  = (u32)&NRF_RADIO.EVENTS_ADDRESS;
    NRF_PPI.CH[NRF_ESB_PPI_TIMER_STOP].TEP  = (u32)&NRF_ESB_SYS_TIMER.TASKS_SHUTDOWN;

    NRF_PPI.CH[NRF_ESB_PPI_RX_TIMEOUT].EEP  = (u32)&NRF_ESB_SYS_TIMER.EVENTS_COMPARE[0];
    NRF_PPI.CH[NRF_ESB_PPI_RX_TIMEOUT].TEP  = (u32)&NRF_RADIO.TASKS_DISABLE;

    NRF_PPI.CH[NRF_ESB_PPI_TX_START].EEP    = (u32)&NRF_ESB_SYS_TIMER.EVENTS_COMPARE[1];
    NRF_PPI.CH[NRF_ESB_PPI_TX_START].TEP    = (u32)&NRF_RADIO.TASKS_TXEN;
}


fn start_tx_transaction()
{
    let ack;

    m_last_tx_attempts = 1;
    // Prepare the payload
    mp_current_payload = m_tx_fifo.p_payload[m_tx_fifo.exit_point];


    match m_config_local.protocol
    {
        NRF_ESB_PROTOCOL_ESB => {
            update_rf_payload_format(mp_current_payload.length);
            m_tx_payload_buffer[0] = mp_current_payload.pid;
            m_tx_payload_buffer[1] = 0;
            memcpy(&m_tx_payload_buffer[2], mp_current_payload.data, mp_current_payload.length);

            NRF_RADIO.SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_RXEN_Msk;
            NRF_RADIO.INTENSET = RADIO_INTENSET_DISABLED_Msk | RADIO_INTENSET_READY_Msk;

            // Configure the retransmit counter
            m_retransmits_remaining = m_config_local.retransmit_count;
            on_radio_disabled = on_radio_disabled_tx;
            m_nrf_esb_mainstate = NRF_ESB_STATE_PTX_TX_ACK;
        }

        NRF_ESB_PROTOCOL_ESB_DPL => {
            ack = !mp_current_payload.noack || !m_config_local.selective_auto_ack;
            m_tx_payload_buffer[0] = mp_current_payload.length;
            m_tx_payload_buffer[1] = mp_current_payload.pid << 1;
            m_tx_payload_buffer[1] |= mp_current_payload.noack?
            0x00: 0x01;
            memcpy(&m_tx_payload_buffer[2], mp_current_payload.data, mp_current_payload.length);

            // Handling ack if noack is set to false or if selective auto ack is turned off
            if (ack)
            {
                NRF_RADIO.SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_RXEN_Msk;
                NRF_RADIO.INTENSET = RADIO_INTENSET_DISABLED_Msk | RADIO_INTENSET_READY_Msk;

                // Configure the retransmit counter
                m_retransmits_remaining = m_config_local.retransmit_count;
                on_radio_disabled = on_radio_disabled_tx;
                m_nrf_esb_mainstate = NRF_ESB_STATE_PTX_TX_ACK;
            } else {
                NRF_RADIO.SHORTS = m_radio_shorts_common;
                NRF_RADIO.INTENSET = RADIO_INTENSET_DISABLED_Msk;
                on_radio_disabled = on_radio_disabled_tx_noack;
                m_nrf_esb_mainstate = NRF_ESB_STATE_PTX_TX;
            }
        }

        _ => ()
    }

    NRF_RADIO.TXADDRESS    = mp_current_payload.pipe;
    NRF_RADIO.RXADDRESSES  = 1 << mp_current_payload.pipe;

    NRF_RADIO.FREQUENCY    = m_esb_addr.rf_channel;
    NRF_RADIO.PACKETPTR    = (u32)m_tx_payload_buffer;

    unsafe {
        NVIC::unpend(Interrupt::RADIO);
        NVIC::unmask(pac::Interrupt::RADIO)
    }

    NRF_RADIO.EVENTS_ADDRESS = 0;
    NRF_RADIO.EVENTS_PAYLOAD = 0;
    NRF_RADIO.EVENTS_DISABLED = 0;

    DEBUG_PIN_SET(DEBUGPIN4);
    NRF_RADIO.TASKS_TXEN  = 1;
}


fn on_radio_disabled_tx_noack()
{
    m_interrupt_flags |= NRF_ESB_INT_TX_SUCCESS_MSK;
    nrf_esb_skip_tx();

    if (m_tx_fifo.count == 0)
    {
        m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
        // NVIC_SetPendingIRQ(ESB_EVT_IRQ);
        // NVIC::pend(Interrupt::RADIO);
    }
    else
    {
        NVIC_SetPendingIRQ(ESB_EVT_IRQ);
        start_tx_transaction();
    }
}


fn on_radio_disabled_tx()
{
    // Remove the DISABLED . RXEN shortcut, to make sure the radio stays
    // disabled after the RX window
    NRF_RADIO.SHORTS           = m_radio_shorts_common;

    // Make sure the timer is started the next time the radio is ready,
    // and that it will disable the radio automatically if no packet is
    // received by the time defined in m_wait_for_ack_timeout_us
    NRF_ESB_SYS_TIMER.CC[0]    = m_wait_for_ack_timeout_us;
    NRF_ESB_SYS_TIMER.CC[1]    = m_config_local.retransmit_delay - 130;
    NRF_ESB_SYS_TIMER.TASKS_CLEAR = 1;
    NRF_ESB_SYS_TIMER.EVENTS_COMPARE[0] = 0;
    NRF_ESB_SYS_TIMER.EVENTS_COMPARE[1] = 0;

    NRF_PPI.CHENSET            = (1 << NRF_ESB_PPI_TIMER_START) |
                                  (1 << NRF_ESB_PPI_RX_TIMEOUT) |
                                  (1 << NRF_ESB_PPI_TIMER_STOP);
    NRF_PPI.CHENCLR            = (1 << NRF_ESB_PPI_TX_START);
    NRF_RADIO.EVENTS_END       = 0;

    if (m_config_local.protocol == NRF_ESB_PROTOCOL_ESB)
    {
        update_rf_payload_format(0);
    }

    NRF_RADIO.PACKETPTR        = (u32)m_rx_payload_buffer;
    on_radio_disabled           = on_radio_disabled_tx_wait_for_ack;
    m_nrf_esb_mainstate         = NRF_ESB_STATE_PTX_RX_ACK;
}


fn on_radio_disabled_tx_wait_for_ack()
{
    // This marks the completion of a TX_RX sequence (TX with ACK)

    // Make sure the timer will not deactivate the radio if a packet is received
    NRF_PPI.CHENCLR = (1 << NRF_ESB_PPI_TIMER_START) |
                       (1 << NRF_ESB_PPI_RX_TIMEOUT)  |
                       (1 << NRF_ESB_PPI_TIMER_STOP);

    // If the radio has received a packet and the CRC status is OK
    if (NRF_RADIO.EVENTS_END && NRF_RADIO.CRCSTATUS != 0)
    {
        NRF_ESB_SYS_TIMER.TASKS_SHUTDOWN = 1;
        NRF_PPI.CHENCLR = (1 << NRF_ESB_PPI_TX_START);
        m_interrupt_flags |= NRF_ESB_INT_TX_SUCCESS_MSK;
        m_last_tx_attempts = m_config_local.retransmit_count - m_retransmits_remaining + 1;

        () nrf_esb_skip_tx();

        if (m_config_local.protocol != NRF_ESB_PROTOCOL_ESB && m_rx_payload_buffer[0] > 0)
        {
            if (rx_fifo_push_rfbuf((u8)NRF_RADIO.TXADDRESS, m_rx_payload_buffer[1] >> 1))
            {
                m_interrupt_flags |= NRF_ESB_INT_RX_DATA_RECEIVED_MSK;
            }
        }

        if ((m_tx_fifo.count == 0) || (m_config_local.tx_mode == NRF_ESB_TXMODE_MANUAL))
        {
            m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
            NVIC_SetPendingIRQ(ESB_EVT_IRQ);
        }
        else
        {
            NVIC_SetPendingIRQ(ESB_EVT_IRQ);
            start_tx_transaction();
        }
    }
    else
    {
        if (m_retransmits_remaining -= 1 == 0)
        {
            NRF_ESB_SYS_TIMER.TASKS_SHUTDOWN = 1;
            NRF_PPI.CHENCLR = (1 << NRF_ESB_PPI_TX_START);
            // All retransmits are expended, and the TX operation is suspended
            m_last_tx_attempts = m_config_local.retransmit_count + 1;
            m_interrupt_flags |= NRF_ESB_INT_TX_FAILED_MSK;

            m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
            NVIC_SetPendingIRQ(ESB_EVT_IRQ);
        }
        else
        {
            // There are still more retransmits left, TX mode should be
            // entered again as soon as the system timer reaches CC[1].
            NRF_RADIO.SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_RXEN_Msk;
            update_rf_payload_format(mp_current_payload.length);
            NRF_RADIO.PACKETPTR = (u32)m_tx_payload_buffer;
            on_radio_disabled = on_radio_disabled_tx;
            m_nrf_esb_mainstate = NRF_ESB_STATE_PTX_TX_ACK;
            NRF_ESB_SYS_TIMER.TASKS_START = 1;
            NRF_PPI.CHENSET = (1 << NRF_ESB_PPI_TX_START);
            if (NRF_ESB_SYS_TIMER.EVENTS_COMPARE[1])
            {
                NRF_RADIO.TASKS_TXEN = 1;
            }
        }
    }
}

fn clear_events_restart_rx()
{
    NRF_RADIO.SHORTS = m_radio_shorts_common;
    update_rf_payload_format(m_config_local.payload_length);
    NRF_RADIO.PACKETPTR = (u32)m_rx_payload_buffer;
    NRF_RADIO.EVENTS_DISABLED = 0;
    NRF_RADIO.TASKS_DISABLE = 1;

    while (NRF_RADIO.EVENTS_DISABLED == 0);

    NRF_RADIO.EVENTS_DISABLED = 0;
    NRF_RADIO.SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_TXEN_Msk;

    NRF_RADIO.TASKS_RXEN = 1;
}

fn on_radio_disabled_rx()
{
    let mut ack            = false;
    let  mut retransmit_payload  = false;
   let mut send_rx_event       = true;
    pipe_info_t *   p_pipe_info;

    if (NRF_RADIO.CRCSTATUS == 0)
    {
        clear_events_restart_rx();
        return;
    }

    if (m_rx_fifo.count >= NRF_ESB_RX_FIFO_SIZE)
    {
        clear_events_restart_rx();
        return;
    }

    p_pipe_info = &m_rx_pipe_info[NRF_RADIO.RXMATCH];
    if (NRF_RADIO.RXCRC             == p_pipe_info.crc &&
        (m_rx_payload_buffer[1] >> 1) == p_pipe_info.pid
       )
    {
        retransmit_payload = true;
        send_rx_event = false;
    }

    p_pipe_info.pid = m_rx_payload_buffer[1] >> 1;
    p_pipe_info.crc = NRF_RADIO.RXCRC;

    if ((m_config_local.selective_auto_ack == false) || ((m_rx_payload_buffer[1] & 0x01) == 1))
    {
        ack = true;
    }

    if (ack)
    {
        NRF_RADIO.SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_RXEN_Msk;

        match m_config_local.protocol
        {
            NRF_ESB_PROTOCOL_ESB_DPL =>
                {
                    if (m_tx_fifo.count > 0 && m_ack_pl_container_entry_point_pr_pipe[NRF_RADIO.RXMATCH] != 0)
                    {
                        mp_current_payload = m_ack_pl_container_entry_point_pr_pipe[NRF_RADIO.RXMATCH].p_payload;

                        // Pipe stays in ACK with payload until TX FIFO is empty
                        // Do not report TX success on first ack payload or retransmit
                        if (p_pipe_info.ack_payload == true && !retransmit_payload)
                        {
                            u32 pipe = NRF_RADIO.RXMATCH;
                            m_ack_pl_container_entry_point_pr_pipe[pipe].in_use = false;
                            m_ack_pl_container_entry_point_pr_pipe[pipe] = (nrf_esb_payload_random_access_buf_wrapper_t *)m_ack_pl_container_entry_point_pr_pipe[pipe].p_next;
                            m_tx_fifo.count -= 1;
                            if (m_tx_fifo.count > 0 && m_ack_pl_container_entry_point_pr_pipe[pipe] != 0)
                            {
                                 mp_current_payload = m_ack_pl_container_entry_point_pr_pipe[pipe].p_payload;
                            }
                            else mp_current_payload = 0;

                            // ACK payloads also require TX_DS
                            // (page 40 of the 'nRF24LE1_Product_Specification_rev1_6.pdf').
                            m_interrupt_flags |= NRF_ESB_INT_TX_SUCCESS_MSK;
                        }

                        if(mp_current_payload != 0)
                        {
                            p_pipe_info.ack_payload = true;
                            update_rf_payload_format(mp_current_payload.length);
                            m_tx_payload_buffer[0] = mp_current_payload.length;
                            memcpy(&m_tx_payload_buffer[2],
                                   mp_current_payload.data,
                                   mp_current_payload.length);
                        }
                        else
                        {
                            p_pipe_info.ack_payload = false;
                            update_rf_payload_format(0);
                            m_tx_payload_buffer[0] = 0;
                        }
                    }
                    else
                    {
                        p_pipe_info.ack_payload = false;
                        update_rf_payload_format(0);
                        m_tx_payload_buffer[0] = 0;
                    }

                    m_tx_payload_buffer[1] = m_rx_payload_buffer[1];
                }

            NRF_ESB_PROTOCOL_ESB =>
                {
                    update_rf_payload_format(0);
                    m_tx_payload_buffer[0] = m_rx_payload_buffer[0];
                    m_tx_payload_buffer[1] = 0;
                }
        }

        m_nrf_esb_mainstate = NRF_ESB_STATE_PRX_SEND_ACK;
        NRF_RADIO.TXADDRESS = NRF_RADIO.RXMATCH;
        NRF_RADIO.PACKETPTR = (u32)m_tx_payload_buffer;
        on_radio_disabled = on_radio_disabled_rx_ack;
    }
    else
    {
        clear_events_restart_rx();
    }

    if (send_rx_event)
    {
        // Push the new packet to the RX buffer and trigger a received event if the operation was
        // successful.
        if (rx_fifo_push_rfbuf(NRF_RADIO.RXMATCH, p_pipe_info.pid))
        {
            m_interrupt_flags |= NRF_ESB_INT_RX_DATA_RECEIVED_MSK;
            NVIC_SetPendingIRQ(ESB_EVT_IRQ);
        }
    }
}


fn on_radio_disabled_rx_ack()
{
    NRF_RADIO.SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_TXEN_Msk;
    update_rf_payload_format(m_config_local.payload_length);

    NRF_RADIO.PACKETPTR = (u32)m_rx_payload_buffer;
    on_radio_disabled = on_radio_disabled_rx;

    m_nrf_esb_mainstate = NRF_ESB_STATE_PRX;
}


/**@brief Function for clearing pending interrupts.
 *
 * @param[in,out]   p_interrupts        Pointer to the value that holds the current interrupts.
 *
 * @retval  NRF_SUCCESS                     If the interrupts were cleared successfully.
 * @retval  NRF_ERROR_NULL                  If the required parameter was NULL.
 * @retval  NRF_INVALID_STATE               If the module is not initialized.
 */
fn nrf_esb_get_clear_interrupts(p_interrupts: *u32) -> u32
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    VERIFY_PARAM_NOT_NULL(p_interrupts);

    unsafe { NVIC::mask(pac::Interrupt::RADIO) }

    *p_interrupts = m_interrupt_flags;
    m_interrupt_flags = 0;

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}


fn RADIO_IRQHandler()
{
    if (NRF_RADIO.EVENTS_READY && (NRF_RADIO.INTENSET & RADIO_INTENSET_READY_Msk))
    {
        NRF_RADIO.EVENTS_READY = 0;
        DEBUG_PIN_SET(DEBUGPIN1);
    }

    if (NRF_RADIO.EVENTS_END && (NRF_RADIO.INTENSET & RADIO_INTENSET_END_Msk))
    {
        NRF_RADIO.EVENTS_END = 0;
        DEBUG_PIN_SET(DEBUGPIN2);

        // Call the correct on_radio_end function, depending on the current protocol state
        if (on_radio_end)
        {
            on_radio_end();
        }
    }

    if (NRF_RADIO.EVENTS_DISABLED && (NRF_RADIO.INTENSET & RADIO_INTENSET_DISABLED_Msk))
    {
        NRF_RADIO.EVENTS_DISABLED = 0;
        DEBUG_PIN_SET(DEBUGPIN3);

        // Call the correct on_radio_disable function, depending on the current protocol state
        if (on_radio_disabled)
        {
            on_radio_disabled();
        }
    }

    DEBUG_PIN_CLR(DEBUGPIN1);
    DEBUG_PIN_CLR(DEBUGPIN2);
    DEBUG_PIN_CLR(DEBUGPIN3);
    DEBUG_PIN_CLR(DEBUGPIN4);
}

/**@brief Function for initializing the Enhanced ShockBurst module.
 *
 * @param  p_config     Parameters for initializing the module.
 *
 * @retval  NRF_SUCCESS             If initialization was successful.
 * @retval  NRF_ERROR_NULL          If the @p p_config argument was NULL.
 * @retval  NRF_ERROR_BUSY          If the function failed because the radio is busy.
 */
fn nrf_esb_init(p_config: &[nrf_esb_config_t]) -> u32
{
    let err_code;

    VERIFY_PARAM_NOT_NULL(p_config);

    if (m_esb_initialized)
    {
        err_code = nrf_esb_disable();
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    m_event_handler = p_config.event_handler;

    memcpy(&m_config_local, p_config, sizeof(nrf_esb_config_t));

    m_interrupt_flags    = 0;

    memset(m_rx_pipe_info, 0, sizeof(m_rx_pipe_info));
    memset(m_pids, 0, sizeof(m_pids));

    VERIFY_TRUE(update_radio_parameters(), NRF_ERROR_INVALID_PARAM);

    // Configure radio address registers according to ESB default values
    NRF_RADIO.BASE0   = 0xE7E7E7E7;
    NRF_RADIO.BASE1   = 0x43434343;
    NRF_RADIO.PREFIX0 = 0x23C343E7;
    NRF_RADIO.PREFIX1 = 0x13E363A3;

    initialize_fifos();

    sys_timer_init();

    ppi_init();

    NVIC_SetPriority(RADIO_IRQn, m_config_local.radio_irq_priority & ESB_IRQ_PRIORITY_MSK);
    NVIC_SetPriority(ESB_EVT_IRQ, m_config_local.event_irq_priority & ESB_IRQ_PRIORITY_MSK);
    NVIC_EnableIRQ(ESB_EVT_IRQ);

    m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
    m_esb_initialized = true;



    #[cfg(feature = "52832")]
    {
        if ((NRF_FICR.INFO.VARIANT & 0x0000FF00) == 0x00004500) //Check if the device is an nRF52832 Rev. 2.
            //Workaround for nRF52832 rev 2 errata 182
            * (volatile
        u32 *) 0x4000173C |= (1 << 10);
    }

    return NRF_SUCCESS;
}


/**@brief Function for suspending the Enhanced ShockBurst module.
 *
 * Calling this function stops ongoing communications without changing the queues.
 *
 * @retval  NRF_SUCCESS             If Enhanced ShockBurst was suspended.
 * @retval  NRF_ERROR_BUSY          If the function failed because the radio is busy.
 */
fn nrf_esb_suspend() -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    // Clear PPI
    NRF_PPI.CHENCLR = (1 << NRF_ESB_PPI_TIMER_START) |
                       (1 << NRF_ESB_PPI_TIMER_STOP)  |
                       (1 << NRF_ESB_PPI_RX_TIMEOUT)  |
                       (1 << NRF_ESB_PPI_TX_START);

    m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;

    return NRF_SUCCESS;
}


/**@brief Function for disabling the Enhanced ShockBurst module.
 *
 * Calling this function disables the Enhanced ShockBurst module immediately.
 * Doing so might stop ongoing communications.
 *
 * @note All queues are flushed by this function.
 *
 * @retval  NRF_SUCCESS             If Enhanced ShockBurst was disabled.
 */
fn nrf_esb_disable() -> u32
{
    // Clear PPI
    NRF_PPI.CHENCLR = (1 << NRF_ESB_PPI_TIMER_START) |
                       (1 << NRF_ESB_PPI_TIMER_STOP)  |
                       (1 << NRF_ESB_PPI_RX_TIMEOUT)  |
                       (1 << NRF_ESB_PPI_TX_START);

    m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
    m_esb_initialized = false;

    reset_fifos();

    memset(m_rx_pipe_info, 0, sizeof(m_rx_pipe_info));
    memset(m_pids, 0, sizeof(m_pids));

    // Disable the radio
    NVIC_DisableIRQ(ESB_EVT_IRQ);
    NRF_RADIO.SHORTS = RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos |
                        RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos;

    return NRF_SUCCESS;
}


/**@brief Function for checking if the Enhanced ShockBurst module is idle.
 *
 * @retval true                     If the module is idle.
 * @retval false                    If the module is busy.
 */
fn nrf_esb_is_idle() -> bool
{
    return m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE;
}


fn ESB_EVT_IRQHandler()
{
       let err_code;
          let interrupts;
       let event;

    event.tx_attempts = m_last_tx_attempts;

    err_code = nrf_esb_get_clear_interrupts(&interrupts);
    if (err_code == NRF_SUCCESS && m_event_handler != 0)
    {
        if (interrupts & NRF_ESB_INT_TX_SUCCESS_MSK)
        {
            event.evt_id = NRF_ESB_EVENT_TX_SUCCESS;
            m_event_handler(&event);
        }
        if (interrupts & NRF_ESB_INT_TX_FAILED_MSK)
        {
            event.evt_id = NRF_ESB_EVENT_TX_FAILED;
            m_event_handler(&event);
        }
        if (interrupts & NRF_ESB_INT_RX_DATA_RECEIVED_MSK)
        {
            event.evt_id = NRF_ESB_EVENT_RX_RECEIVED;
            m_event_handler(&event);
        }
    }
}


fn find_free_payload_cont() -> *nrf_esb_payload_random_access_buf_wrapper_t
{
    for (let i = 0; i < NRF_ESB_TX_FIFO_SIZE; i += 1)
    {
        if(!m_ack_pl_container[i].in_use) return &m_ack_pl_container[i];
    }
    return 0;
}


/**@brief Function for writing a payload for transmission or acknowledgement.
 *
 * This function writes a payload that is added to the queue. When the module is in PTX mode, the
 * payload is queued for a regular transmission. When the module is in PRX mode, the payload
 * is queued for when a packet is received that requires an acknowledgement with payload.
 *
 * @param[in]   p_payload     Pointer to the structure that contains information and state of the payload.
 *
 * @retval  NRF_SUCCESS                     If the payload was successfully queued for writing.
 * @retval  NRF_ERROR_NULL                  If the required parameter was NULL.
 * @retval  NRF_INVALID_STATE               If the module is not initialized.
 * @retval  NRF_ERROR_NO_MEM                If the TX FIFO is full.
 * @retval  NRF_ERROR_INVALID_LENGTH        If the payload length was invalid (zero or larger than the allowed maximum).
 */
fn nrf_esb_write_payload(p_payload: &nrf_esb_payload_t) -> u32
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    VERIFY_PARAM_NOT_NULL(p_payload);
    VERIFY_PAYLOAD_LENGTH(p_payload);
    VERIFY_FALSE(m_tx_fifo.count >= NRF_ESB_TX_FIFO_SIZE, NRF_ERROR_NO_MEM);
    VERIFY_TRUE(p_payload.pipe < NRF_ESB_PIPE_COUNT, NRF_ERROR_INVALID_PARAM);

    DISABLE_RF_IRQ();

    if (m_config_local.mode == NRF_ESB_MODE_PTX)
    {
        memcpy(m_tx_fifo.p_payload[m_tx_fifo.entry_point], p_payload, sizeof(nrf_esb_payload_t));

        m_pids[p_payload.pipe] = (m_pids[p_payload.pipe] + 1) % (NRF_ESB_PID_MAX + 1);
        m_tx_fifo.p_payload[m_tx_fifo.entry_point].pid = m_pids[p_payload.pipe];

        if ((m_tx_fifo.entry_point += 1) >= NRF_ESB_TX_FIFO_SIZE)
        {
            m_tx_fifo.entry_point = 0;
        }

        m_tx_fifo.count += 1;
    }
    else
    {
        nrf_esb_payload_random_access_buf_wrapper_t *new_ack_payload;
        if((new_ack_payload = find_free_payload_cont()) != 0)
        {
            new_ack_payload.in_use = true;
            new_ack_payload.p_next = 0;
            memcpy(new_ack_payload.p_payload, p_payload, sizeof(nrf_esb_payload_t));

            m_pids[p_payload.pipe] = (m_pids[p_payload.pipe] + 1) % (NRF_ESB_PID_MAX + 1);
            new_ack_payload.p_payload.pid = m_pids[p_payload.pipe];

            if(m_ack_pl_container_entry_point_pr_pipe[p_payload.pipe] == 0)
            {
                m_ack_pl_container_entry_point_pr_pipe[p_payload.pipe] = new_ack_payload;
            }
            else
            {
                nrf_esb_payload_random_access_buf_wrapper_t *list_iterator = m_ack_pl_container_entry_point_pr_pipe[p_payload.pipe];
                while(list_iterator.p_next != 0)
                {
                    list_iterator = (nrf_esb_payload_random_access_buf_wrapper_t *)list_iterator.p_next;
                }
                list_iterator.p_next = (struct nrf_esb_payload_random_access_buf_wrapper_t *)new_ack_payload;
            }
            m_tx_fifo.count += 1;
        }
    }

    ENABLE_RF_IRQ();


    if (m_config_local.mode == NRF_ESB_MODE_PTX &&
        m_config_local.tx_mode == NRF_ESB_TXMODE_AUTO &&
        m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE)
    {
        start_tx_transaction();
    }

    return NRF_SUCCESS;
}


/**@brief Function for reading an RX payload.
 *
 * @param[in,out]   p_payload   Pointer to the structure that contains information and state of the payload.
 *
 * @retval  NRF_SUCCESS                     If the data was read successfully.
 * @retval  NRF_ERROR_NULL                  If the required parameter was NULL.
 * @retval  NRF_INVALID_STATE               If the module is not initialized.
 */
fn nrf_esb_read_rx_payload(p_payload: &nrf_esb_payload_t) -> u32 {
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    VERIFY_PARAM_NOT_NULL(p_payload);

    if (m_rx_fifo.count == 0)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    DISABLE_RF_IRQ();

    p_payload.length = m_rx_fifo.p_payload[m_rx_fifo.exit_point].length;
    p_payload.pipe   = m_rx_fifo.p_payload[m_rx_fifo.exit_point].pipe;
    p_payload.rssi   = m_rx_fifo.p_payload[m_rx_fifo.exit_point].rssi;
    p_payload.pid    = m_rx_fifo.p_payload[m_rx_fifo.exit_point].pid;
    p_payload.noack  = m_rx_fifo.p_payload[m_rx_fifo.exit_point].noack;
    memcpy(p_payload.data, m_rx_fifo.p_payload[m_rx_fifo.exit_point].data, p_payload.length);

    if (m_rx_fifo.exit_point += 1 >= NRF_ESB_RX_FIFO_SIZE)
    {
        m_rx_fifo.exit_point = 0;
    }

    m_rx_fifo.count -= 1;

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}


/**@brief Function for starting transmission.
 *
 * @retval  NRF_SUCCESS                     If the TX started successfully.
 * @retval  NRF_ERROR_BUFFER_EMPTY          If the TX did not start because the FIFO buffer is empty.
 * @retval  NRF_ERROR_BUSY                  If the function failed because the radio is busy.
 */
fn nrf_esb_start_tx() -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    if (m_tx_fifo.count == 0)
    {
        return NRF_ERROR_BUFFER_EMPTY;
    }

    start_tx_transaction();

    return NRF_SUCCESS;
}


/**@brief Function for starting to transmit data from the FIFO buffer.
 *
 * @retval  NRF_SUCCESS                     If the transmission was started successfully.
 * @retval  NRF_ERROR_BUSY                  If the function failed because the radio is busy.
 */
fn nrf_esb_start_rx() -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    NRF_RADIO.INTENCLR = 0xFFFFFFFF;
    NRF_RADIO.EVENTS_DISABLED = 0;
    on_radio_disabled = on_radio_disabled_rx;

    NRF_RADIO.SHORTS      = m_radio_shorts_common | RADIO_SHORTS_DISABLED_TXEN_Msk;
    NRF_RADIO.INTENSET    = RADIO_INTENSET_DISABLED_Msk;
    m_nrf_esb_mainstate    = NRF_ESB_STATE_PRX;

    NRF_RADIO.RXADDRESSES  = m_esb_addr.rx_pipes_enabled;
    NRF_RADIO.FREQUENCY    = m_esb_addr.rf_channel;
    NRF_RADIO.PACKETPTR    = (u32)m_rx_payload_buffer;

    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    NRF_RADIO.EVENTS_ADDRESS = 0;
    NRF_RADIO.EVENTS_PAYLOAD = 0;
    NRF_RADIO.EVENTS_DISABLED = 0;

    NRF_RADIO.TASKS_RXEN  = 1;

    return NRF_SUCCESS;
}


/** @brief Function for stopping data reception.
 *
 * @retval  NRF_SUCCESS                     If data reception was stopped successfully.
 * @retval  NRF_ESB_ERROR_NOT_IN_RX_MODE    If the function failed because the module is not in RX mode.
 */
fn nrf_esb_stop_rx() -> u32
{
    if (m_nrf_esb_mainstate == NRF_ESB_STATE_PRX ||
        m_nrf_esb_mainstate == NRF_ESB_STATE_PRX_SEND_ACK)
    {
        NRF_RADIO.SHORTS = 0;
        NRF_RADIO.INTENCLR = 0xFFFFFFFF;
        on_radio_disabled = NULL;
        NRF_RADIO.EVENTS_DISABLED = 0;
        NRF_RADIO.TASKS_DISABLE = 1;
        while (NRF_RADIO.EVENTS_DISABLED == 0);
        m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;

        return NRF_SUCCESS;
    }

    return NRF_ESB_ERROR_NOT_IN_RX_MODE;
}


/**@brief Function for removing remaining items from the TX buffer.
 *
 * This function clears the TX FIFO buffer.
 *
 * @retval  NRF_SUCCESS                     If pending items in the TX buffer were successfully cleared.
 * @retval  NRF_INVALID_STATE               If the module is not initialized.
 */
fn nrf_esb_flush_tx() -> u32
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);

    DISABLE_RF_IRQ();

    m_tx_fifo.count = 0;
    m_tx_fifo.entry_point = 0;
    m_tx_fifo.exit_point = 0;

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}


/**@brief Function for removing the newest entry from the TX buffer.
 *
 * This function will remove the most recently added element from the FIFO queue.
 *
 * @retval  NRF_SUCCESS                     If the operation completed successfully.
 * @retval  NRF_INVALID_STATE               If the module is not initialized.
 * @retval  NRF_ERROR_BUFFER_EMPTY          If there are no items in the queue to remove.
 */
fn nrf_esb_pop_tx() -> u32
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    VERIFY_TRUE(m_tx_fifo.count > 0, NRF_ERROR_BUFFER_EMPTY);

    DISABLE_RF_IRQ();

    if (m_tx_fifo.entry_point == 0)
    {
        m_tx_fifo.entry_point = (NRF_ESB_TX_FIFO_SIZE-1);
    }
    else
    {
        m_tx_fifo.entry_point -= 1;
    }
    m_tx_fifo.count -= 1;

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}


/**@brief Function for removing remaining items from the RX buffer.
 *
 * @retval  NRF_SUCCESS                     If the pending items in the RX buffer were successfully cleared.
 * @retval  NRF_INVALID_STATE               If the module is not initialized.
 */
fn nrf_esb_flush_rx() -> u32
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);

    DISABLE_RF_IRQ();

    m_rx_fifo.count = 0;
    m_rx_fifo.entry_point = 0;
    m_rx_fifo.exit_point = 0;

    memset(m_rx_pipe_info, 0, sizeof(m_rx_pipe_info));

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}


/**@brief Function for setting the length of the address.
 *
 * @param[in]       length              Length of the ESB address (in bytes).
 *
 * @retval  NRF_SUCCESS                      If the address length was set successfully.
 * @retval  NRF_ERROR_INVALID_PARAM          If the address length was invalid.
 * @retval  NRF_ERROR_BUSY                   If the function failed because the radio is busy.
 */
fn nrf_esb_set_address_length(length: u8) -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(length > 2 && length < 6, NRF_ERROR_INVALID_PARAM);

    cfg_if! {
        if #[cfg(feature = "52832")] {
            let base_address_mask = if length == 5 { 0xFFFF0000 } else { 0xFF000000 };
            if ((NRF_FICR.INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
            {
                /*
                Workaround for nRF52832 Rev 1 Errata 107
                Check if pipe 0 or pipe 1-7 has a 'zero address'.
                Avoid using access addresses in the following pattern (where X is don't care):
                ADDRLEN=5
                BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00
                BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX
                BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX
                BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX
                BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00
                BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX
                BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX
                BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX

                ADDRLEN=4
                BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00
                BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX
                BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX
                BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX
                BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00
                BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX
                BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX
                BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
                */
                if ((NRF_RADIO.BASE0 & base_address_mask) == 0 && (NRF_RADIO.PREFIX0 & 0x000000FF) == 0)
                {
                    return NRF_ERROR_INVALID_PARAM;
                }
                if ((NRF_RADIO.BASE1 & base_address_mask) == 0 && ((NRF_RADIO.PREFIX0 & 0x0000FF00) == 0 ||(NRF_RADIO.PREFIX0 & 0x00FF0000) == 0 || (NRF_RADIO.PREFIX0 & 0xFF000000) == 0 ||
                   (NRF_RADIO.PREFIX1 & 0xFF000000) == 0 || (NRF_RADIO.PREFIX1 & 0x00FF0000) == 0 ||(NRF_RADIO.PREFIX1 & 0x0000FF00) == 0 || (NRF_RADIO.PREFIX1 & 0x000000FF) == 0))
                {
                    return NRF_ERROR_INVALID_PARAM;
                }
            }
        }
    }

    m_esb_addr.addr_length = length;

    update_rf_payload_format(m_config_local.payload_length);

#ifdef NRF52832_XXAA
    if ((NRF_FICR.INFO.VARIANT & 0x0000FF00) == 0x00004500)  //Check if the device is an nRF52832 Rev. 2.
    {
        return apply_address_workarounds();
    }
    else
    {
        return NRF_SUCCESS;
    }
#else
    return NRF_SUCCESS;
#endif
}


/**@brief Function for setting the base address for pipe 0.
 *
 * @param[in]       p_addr      Pointer to the address data.
 *
 * @retval  NRF_SUCCESS                     If the base address was set successfully.
 * @retval  NRF_ERROR_BUSY                  If the function failed because the radio is busy.
 * @retval  NRF_ERROR_INVALID_PARAM         If the function failed because the address given was too close to a zero address.
 * @retval  NRF_ERROR_NULL                  If the required parameter was NULL.
 */
fn nrf_esb_set_base_address_0(p_addr: *const u8) -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_PARAM_NOT_NULL(p_addr);

    cfg_if! {
        if #[cfg(feature = "52832")] {
            if ((NRF_FICR.INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
            {
                /*
                Workaround for nRF52832 Rev 1 Errata 107
                Check if pipe 0 or pipe 1-7 has a 'zero address'.
                Avoid using access addresses in the following pattern (where X is don't care):
                ADDRLEN=5
                BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00
                BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX
                BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX
                BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX
                BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00
                BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX
                BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX
                BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX

                ADDRLEN=4
                BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00
                BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX
                BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX
                BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX
                BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00
                BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX
                BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX
                BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
                */
                u32 base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;
                if ((addr_conv(p_addr) & base_address_mask) == 0 && (NRF_RADIO.PREFIX0 & 0x000000FF) == 0)
                {
                    return NRF_ERROR_INVALID_PARAM;
                }
            }
        }
    }

    memcpy(m_esb_addr.base_addr_p0, p_addr, 4);

    update_radio_addresses(NRF_ESB_ADDR_UPDATE_MASK_BASE0);
    cfg_if! {
        if #[cfg(feature = "52832")] {
            return apply_address_workarounds();
        } else {
            return NRF_SUCCESS;
        }
    }
}


/**@brief Function for setting the base address for pipe 1 to pipe 7.
 *
 * @param[in]       p_addr      Pointer to the address data.
 *
 * @retval  NRF_SUCCESS                     If the base address was set successfully.
 * @retval  NRF_ERROR_BUSY                  If the function failed because the radio is busy.
 * @retval  NRF_ERROR_INVALID_PARAM         If the function failed because the address given was too close to a zero address.
 * @retval  NRF_ERROR_NULL                  If the required parameter was NULL.
 */
fn nrf_esb_set_base_address_1(p_addr: *const u8) -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_PARAM_NOT_NULL(p_addr);

    cfg_if! {
        if #[cfg(feature = "52832")] {
            if ((NRF_FICR.INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
            {
                /*
                Workaround for nRF52832 Rev 1 Errata 107
                Check if pipe 0 or pipe 1-7 has a 'zero address'.
                Avoid using access addresses in the following pattern (where X is don't care):
                ADDRLEN=5
                BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00
                BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX
                BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX
                BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX
                BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00
                BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX
                BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX
                BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX

                ADDRLEN=4
                BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00
                BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX
                BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX
                BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX
                BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00
                BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX
                BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX
                BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
                */
                let base_address_mask = if m_esb_addr.addr_length == 5 { 0xFFFF0000 } else { 0xFF000000 };
                if ((addr_conv(p_addr) & base_address_mask) == 0 &&
                    ((NRF_RADIO.PREFIX0 & 0x0000FF00) == 0 ||(NRF_RADIO.PREFIX0 & 0x00FF0000) == 0 ||
                    (NRF_RADIO.PREFIX0 & 0xFF000000) == 0 || (NRF_RADIO.PREFIX1 & 0xFF000000) == 0 ||
                    (NRF_RADIO.PREFIX1 & 0x00FF0000) == 0 ||(NRF_RADIO.PREFIX1 & 0x0000FF00) == 0 ||
                    (NRF_RADIO.PREFIX1 & 0x000000FF) == 0))
                {
                    return NRF_ERROR_INVALID_PARAM;
                }
            }
        }
    }
}



    memcpy(m_esb_addr.base_addr_p1, p_addr, 4);

    update_radio_addresses(NRF_ESB_ADDR_UPDATE_MASK_BASE1);

    cfg_if! {
        if #[cfg(feature = "52832")] {
            return apply_address_workarounds();
        } else {
            return NRF_SUCCESS;
        }
    }
}


/**@brief Function for setting the number of pipes and the pipe prefix addresses.
 *
 * This function configures the number of available pipes, enables the pipes,
 * and sets their prefix addresses.
 *
 * @param[in]   p_prefixes      Pointer to a char array that contains the prefix for each pipe.
 * @param[in]   num_pipes       Number of pipes. Must be less than or equal to @ref NRF_ESB_PIPE_COUNT.
 *
 * @retval  NRF_SUCCESS                     If the prefix addresses were set successfully.
 * @retval  NRF_ERROR_BUSY                  If the function failed because the radio is busy.
 * @retval  NRF_ERROR_NULL                  If a required parameter was NULL.
 * @retval  NRF_ERROR_INVALID_PARAM         If an invalid number of pipes was given or if the address given was too close to a zero address.
 */
fn nrf_esb_set_prefixes(p_prefixes: &[u8], num_pipes: u8) ->u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_PARAM_NOT_NULL(p_prefixes);
    VERIFY_TRUE(num_pipes <= NRF_ESB_PIPE_COUNT, NRF_ERROR_INVALID_PARAM);

#ifdef NRF52832_XXAA
    if ((NRF_FICR.INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
    {
        /*
        Workaround for nRF52832 Rev 1 Errata 107
        Check if pipe 0 or pipe 1-7 has a 'zero address'.
        Avoid using access addresses in the following pattern (where X is don't care):
        ADDRLEN=5
        BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX

        ADDRLEN=4
        BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
        */
        u32 base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;
        if (num_pipes >= 1 && (NRF_RADIO.BASE0 & base_address_mask) == 0 && p_prefixes[0] == 0)
        {
            return NRF_ERROR_INVALID_PARAM;
        }

        if ((NRF_RADIO.BASE1 & base_address_mask) == 0)
        {
            for (u8 i = 1; i < num_pipes; i += 1)
            {
                if (p_prefixes[i] == 0)
                {
                    return NRF_ERROR_INVALID_PARAM;
                }
            }
        }
    }
#endif

    memcpy(m_esb_addr.pipe_prefixes, p_prefixes, num_pipes);
    m_esb_addr.num_pipes = num_pipes;
    m_esb_addr.rx_pipes_enabled = BIT_MASK_UINT_8(num_pipes);

    update_radio_addresses(NRF_ESB_ADDR_UPDATE_MASK_PREFIX);

cfg_if! {
    if #[cfg(feature = "52832")] {
        return apply_address_workarounds();
    } else {
        return NRF_SUCCESS;
    }
}

/**@brief Function for updating the prefix for a pipe.
 *
 * @param   pipe    Pipe for which to set the prefix.
 * @param   prefix  Prefix to set for the pipe.
 *
 * @retval  NRF_SUCCESS                         If the operation completed successfully.
 * @retval  NRF_ERROR_BUSY                      If the function failed because the radio is busy.
 * @retval  NRF_ERROR_INVALID_PARAM             If the given pipe number was invalid or if the address given was too close to a zero address.
 */
fn nrf_esb_update_prefix(pipe: u8, prefix: u8) -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(pipe < NRF_ESB_PIPE_COUNT, NRF_ERROR_INVALID_PARAM);

    cfg_if! {
        if #[cfg(feature = "52832")] {
            if ((NRF_FICR.INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
            {
                /*
                Workaround for nRF52832 Rev 1 Errata 107
                Check if pipe 0 or pipe 1-7 has a 'zero address'.
                Avoid using access addresses in the following pattern (where X is don't care):
                ADDRLEN=5
                BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00
                BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX
                BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX
                BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX
                BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00
                BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX
                BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX
                BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX

                ADDRLEN=4
                BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00
                BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX
                BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX
                BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX
                BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00
                BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX
                BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX
                BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
                */
                u32 base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;
                if (pipe == 0)
                {
                    if ((NRF_RADIO.BASE0 & base_address_mask) == 0 && prefix == 0)
                    {
                        return NRF_ERROR_INVALID_PARAM;
                    }
                }
                else
                {
                    if ((NRF_RADIO.BASE1 & base_address_mask) == 0 && prefix == 0)
                    {
                        return NRF_ERROR_INVALID_PARAM;
                    }
                }
            }
        }
    }
    m_esb_addr.pipe_prefixes[pipe] = prefix;

    update_radio_addresses(NRF_ESB_ADDR_UPDATE_MASK_PREFIX);

    cfg_if! {
        if #[cfg(feature = "52832")] {
            return apply_address_workarounds();
        } else {
            return NRF_SUCCESS;
        }
    }
}


/**@brief Function for enabling pipes.
 *
 * The @p enable_mask parameter must contain the same number of pipes as has been configured
 * with @ref nrf_esb_set_prefixes. This number may not be greater than the number defined by
 * @ref NRF_ESB_PIPE_COUNT
 *
 * @param   enable_mask         Bitfield mask to enable or disable pipes. Setting a bit to
 *                              0 disables the pipe. Setting a bit to 1 enables the pipe.
 *
 * @retval  NRF_SUCCESS                     If the pipes were enabled and disabled successfully.
 * @retval  NRF_ERROR_BUSY                  If the function failed because the radio is busy.
 * @retval  NRF_ERROR_INVALID_PARAM         If the function failed because the address given was too close to a zero address.
 */
fn nrf_esb_enable_pipes(enable_mask: u8) -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE((enable_mask | BIT_MASK_UINT_8(NRF_ESB_PIPE_COUNT)) == BIT_MASK_UINT_8(NRF_ESB_PIPE_COUNT), NRF_ERROR_INVALID_PARAM);

    m_esb_addr.rx_pipes_enabled = enable_mask;

    cfg_if! {
    if #[cfg(feature = "52832")] {
        return apply_address_workarounds();
    } else {
        return NRF_SUCCESS;
    }
}
}

/** @brief Function for setting the channel to use for the radio.
 *
 * The module must be in an idle state to call this function. As a PTX, the
 * application must wait for an idle state and as a PRX, the application must stop RX
 * before changing the channel. After changing the channel, operation can be resumed.
 *
 * @param[in]   channel                         Channel to use for radio.
 *
 * @retval  NRF_SUCCESS                         If the operation completed successfully.
 * @retval  NRF_INVALID_STATE                   If the module is not initialized.
 * @retval  NRF_ERROR_BUSY                      If the module was not in idle state.
 * @retval  NRF_ERROR_INVALID_PARAM             If the channel is invalid (larger than 100).
 */
fn nrf_esb_set_rf_channel(channel: u32) -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(channel <= 100, NRF_ERROR_INVALID_PARAM);

    m_esb_addr.rf_channel = channel;

    return NRF_SUCCESS;
}


/**@brief Function for getting the current radio channel.
 *
 * @param[in, out] p_channel    Pointer to the channel data.
 *
 * @retval  NRF_SUCCESS                         If the operation completed successfully.
 * @retval  NRF_ERROR_NULL                      If the required parameter was NULL.
 */
fn nrf_esb_get_rf_channel(p_channel: &mut [u32]) -> u32
{
    VERIFY_PARAM_NOT_NULL(p_channel);

    p_channel = m_esb_addr.rf_channel;

    return NRF_SUCCESS;
}


/**@brief Function for setting the radio output power.
 *
 * @param[in]   tx_output_power    Output power.
 *
 * @retval  NRF_SUCCESS                         If the operation completed successfully.
 * @retval  NRF_ERROR_BUSY                      If the function failed because the radio is busy.
 */
fn nrf_esb_set_tx_power(tx_output_power: nrf_esb_tx_power_t) -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    if ( m_config_local.tx_output_power != tx_output_power )
    {
        m_config_local.tx_output_power = tx_output_power;
        update_radio_tx_power();
    }

    return NRF_SUCCESS;
}


/**@brief Function for setting the packet retransmit delay.
 *
 * @param[in]   delay                           Delay between retransmissions.
 *
 * @retval  NRF_SUCCESS                         If the operation completed successfully.
 * @retval  NRF_ERROR_BUSY                      If the function failed because the radio is busy.
 */
fn nrf_esb_set_retransmit_delay(delay: u16) -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(delay >= m_wait_for_ack_timeout_us + RETRANSMIT_DELAY_US_OFFSET, NRF_ERROR_INVALID_PARAM);

    m_config_local.retransmit_delay = delay;
    return NRF_SUCCESS;
}


/**@brief Function for setting the number of retransmission attempts.
 *
 * @param[in]   count                           Number of retransmissions.
 *
 * @retval  NRF_SUCCESS                         If the operation completed successfully.
 * @retval  NRF_ERROR_BUSY                      If the function failed because the radio is busy.
 */
fn nrf_esb_set_retransmit_count(count: u16) -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    m_config_local.retransmit_count = count;
    return NRF_SUCCESS;
}


/**@brief Function for setting the radio bitrate.
 *
 * @param[in]   bitrate                         Radio bitrate.
 *
 * @retval  NRF_SUCCESS                         If the operation completed successfully.
 * @retval  NRF_ERROR_BUSY                      If the function failed because the radio is busy.
 */
fn nrf_esb_set_bitrate(bitrate: nrf_esb_bitrate_t) -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    m_config_local.bitrate = bitrate;
    return if update_radio_bitrate() { NRF_SUCCESS } else { NRF_ERROR_INVALID_PARAM };
}


/**@brief Function for reusing a packet ID for a specific pipe.
 *
 * The ESB protocol uses a 2-bit sequence number (packet ID) to identify
 * retransmitted packets. By default, the packet ID is incremented for every
 * uploaded packet. Use this function to prevent this and send two different
 * packets with the same packet ID.
 *
 * @param[in]   pipe                            Pipe.
 *
 * @retval  NRF_SUCCESS                         If the operation completed successfully.
 * @retval  NRF_ERROR_BUSY                      If the function failed because the radio is busy.
 */
fn nrf_esb_reuse_pid(pipe: u8) -> u32
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(pipe < NRF_ESB_PIPE_COUNT, NRF_ERROR_INVALID_PARAM);

    m_pids[pipe] = (m_pids[pipe] + NRF_ESB_PID_MAX) % (NRF_ESB_PID_MAX + 1);
    return NRF_SUCCESS;
}


// Workaround neccessary on nRF52832 Rev. 1.
#[cfg(feature = "nrf52832")]
fn NRF_ESB_BUGFIX_TIMER_IRQHandler()
{
    if (NRF_ESB_BUGFIX_TIMER.EVENTS_COMPARE[0])
    {
        NRF_ESB_BUGFIX_TIMER.EVENTS_COMPARE[0] = 0;

        // If the timeout timer fires and we are in the PTX receive ACK state, disable the radio
        if (m_nrf_esb_mainstate == NRF_ESB_STATE_PTX_RX_ACK)
        {
            NRF_RADIO.TASKS_DISABLE = 1;
        }
    }
}
