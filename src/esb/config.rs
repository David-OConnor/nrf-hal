//! This file mirrors `nrf_esb.h`, `nrf_esb_error_codes.h`, and `nrf_esb_resources.h`.

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


use cfg_if::cfg_if;

/** @defgroup nrf_esb Enhanced ShockBurst
* @{
* @ingroup proprietary_api
*
* @brief Enhanced ShockBurst (ESB) is a basic protocol that supports two-way data
*        packet communication including packet buffering, packet acknowledgment,
*        and automatic retransmission of lost packets.
 */

/** @name Debug pins
* @{
* @brief If NRF_ESB_DEBUG is defined, these GPIO pins can be used for debug timing.
 */

cfg_if! {
    if #[cfg(feature = "52840")] {
        pub(crate) const DEBUGPIN1: u8 =   12; //!< If NRF_ESB_DEBUG is defined, this GPIO pin is set with every radio interrupt.
        pub(crate) const DEBUGPIN2: u8 =    13; //!< If NRF_ESB_DEBUG is defined, this GPIO pin is set with every NRF_RADIO->EVENTS_END.
        pub(crate) const DEBUGPIN3: u8 =    14; //!< If NRF_ESB_DEBUG is defined, this GPIO pin is set with every NRF_RADIO->EVENTS_DISABLED.
        pub(crate) const DEBUGPIN4: u8 =    15; //!< If NRF_ESB_DEBUG is defined, this GPIO pin is set when the radio is set to start transmission.
    } else {
        pub(crate) const DEBUGPIN1: u8 =    24; //!< If NRF_ESB_DEBUG is defined, this GPIO pin is set with every radio interrupt.
        pub(crate) const DEBUGPIN2: u8 =    25; //!< If NRF_ESB_DEBUG is defined, this GPIO pin is set with every NRF_RADIO->EVENTS_END.
        pub(crate) const DEBUGPIN3: u8 =   26; //!< If NRF_ESB_DEBUG is defined, this GPIO pin is set with every NRF_RADIO->EVENTS_DISABLED.
        pub(crate) const DEBUGPIN4: u8 =    27; //!< If NRF_ESB_DEBUG is defined, this GPIO pin is set when the radio is set to start transmission.
    }
}

if NRF_ESB_DEBUG {
    pub(crate) const DEBUG_PIN_SET(a)    (NRF_GPIO->OUTSET = (1 < < (a))) //!< Used internally to set debug pins.
    pub(crate) const DEBUG_PIN_CLR(a)    (NRF_GPIO->OUTCLR = (1 < < (a))) //!< Used internally to clear debug pins.
} else {
    pub ( crate ) const DEBUG_PIN_SET(a) //!< Used internally to set debug pins.
    pub( crate ) const DEBUG_PIN_CLR(a) //!< Used internally to clear debug pins.
}

  /** @} */

// Hardcoded parameters - change if necessary
// #ifndef NRF_ESB_MAX_PAYLOAD_LENGTH
pub(crate) const     NRF_ESB_MAX_PAYLOAD_LENGTH: u8 =           32;                  //!< The maximum size of the payload. Valid values are 1 to 252.
// #endif

pub(crate) const     NRF_ESB_TX_FIFO_SIZE: u8 =                  8;                   //!< The size of the transmission first-in, first-out buffer.
pub(crate) const     NRF_ESB_RX_FIFO_SIZE: u8 =                  8;                   //!< The size of the reception first-in, first-out buffer.

// 252 is the largest possible payload size according to the nRF5 architecture.
assert!(NRF_ESB_MAX_PAYLOAD_LENGTH <= 252);

pub(crate) const     NRF_ESB_SYS_TIMER: u8 =                     NRF_TIMER2 ;         //!< The timer that is used by the module.
pub(crate) const     NRF_ESB_SYS_TIMER_IRQ_Handler: u8 =         TIMER2_IRQHandler;   //!< The handler that is used by @ref NRF_ESB_SYS_TIMER.

pub(crate) const     NRF_ESB_PPI_TIMER_START: u8 =               10 ;                 //!< The PPI channel used for starting the timer.
pub(crate) const     NRF_ESB_PPI_TIMER_STOP : u8 =               11 ;                 //!< The PPI channel used for stopping the timer.
pub(crate) const     NRF_ESB_PPI_RX_TIMEOUT  : u8 =              12  ;                //!< The PPI channel used for RX time-out.
pub(crate) const     NRF_ESB_PPI_TX_START  : u8 =                13  ;                //!< The PPI channel used for starting TX.

#ifndef NRF_ESB_PIPE_COUNT
pub(crate) const     NRF_ESB_PIPE_COUNT     : u8 =               8  ;                 //!< The maximum number of pipes allowed in the API, can be used if you need to restrict the number of pipes used. Must be 8 or lower because of architectural limitations.
#endif
assert!(NRF_ESB_PIPE_COUNT <= 8);

/**@cond NO_DOXYGEN */
// These bugfixes are for nRF-52833 only.
// nRF52 address fix timer and PPI defines
pub(crate) const     NRF_ESB_PPI_BUGFIX1   : u8 =                9;
pub(crate) const     NRF_ESB_PPI_BUGFIX2   : u8 =                8;
pub(crate) const     NRF_ESB_PPI_BUGFIX3   : u8 =                7;

pub(crate) const     NRF_ESB_BUGFIX_TIMER      : u8 =            NRF_TIMER3;
pub(crate) const     NRF_ESB_BUGFIX_TIMER_IRQn  : u8 =           TIMER3_IRQn;
pub(crate) const     NRF_ESB_BUGFIX_TIMER_IRQHandler : u8 =      TIMER3_IRQHandler;

/** @endcond */

// Interrupt flags
pub(crate) const     NRF_ESB_INT_TX_SUCCESS_MSK  : u8 =          0x01  ;              //!< The flag used to indicate a success since the last event.
pub(crate) const     NRF_ESB_INT_TX_FAILED_MSK   : u8 =          0x02  ;              //!< The flag used to indicate a failure since the last event.
pub(crate) const     NRF_ESB_INT_RX_DR_MSK     : u8 =            0x04   ;             //!< The flag used to indicate that a packet was received since the last event.

pub(crate) const     NRF_ESB_PID_RESET_VALUE  : u8 =             0xFF   ;             //!< Invalid PID value that is guaranteed to not collide with any valid PID value.
pub(crate) const     NRF_ESB_PID_MAX    : u8 =                   3      ;             //!< The maximum value for PID.
pub(crate) const     NRF_ESB_CRC_RESET_VALUE   : u8 =            0xFFFF  ;            //!< The CRC reset value.

pub(crate) const     ESB_EVT_IRQ    : u8 =                       SWI0_IRQn    ;       //!< The ESB event IRQ number when running on an nRF5 device.
pub(crate) const     ESB_EVT_IRQHandler     : u8 =               SWI0_IRQHandler   ; //!< The handler for @ref ESB_EVT_IRQ when running on an nRF5 device.

pub(crate) const ESB_IRQ_PRIORITY_MSK      : u8 =                0x07  ;              //!< The mask used to enforce a valid IRQ priority.


/** @brief Default address configuration for ESB.
*  @details Roughly equal to the nRF24Lxx default (except for the number of pipes, because more pipes are supported). */
pub(crate) const NRF_ESB_ADDR_DEFAULT                                                    \
{                                                                               \
    .base_addr_p0       = { 0xE7, 0xE7, 0xE7, 0xE7 },                           \
    .base_addr_p1       = { 0xC2, 0xC2, 0xC2, 0xC2 },                           \
    .pipe_prefixes      = { 0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 },   \
    .addr_length        = 5,                                                    \
    .num_pipes          = NRF_ESB_PIPE_COUNT,                                   \
    .rf_channel         = 2,                                                    \
    .rx_pipes_enabled   = 0xFF                                                  \
}


/** @brief Default radio parameters.
*  @details Roughly equal to the nRF24Lxx default parameters (except for CRC, which is set to 16 bit, and protocol, which is set to DPL). */
pub(crate) const NRF_ESB_DEFAULT_CONFIG {.protocol               = NRF_ESB_PROTOCOL_ESB_DPL,         \
                                .mode                   = NRF_ESB_MODE_PTX,                 \
                                .event_handler          = 0,                                \
                                .bitrate                = NRF_ESB_BITRATE_2MBPS,            \
                                .crc                    = NRF_ESB_CRC_16BIT,                \
                                .tx_output_power        = NRF_ESB_TX_POWER_0DBM,            \
                                .retransmit_delay       = 250,                              \
                                .retransmit_count       = 3,                                \
                                .tx_mode                = NRF_ESB_TXMODE_AUTO,              \
                                .radio_irq_priority     = 1,                                \
                                .event_irq_priority     = 2,                                \
                                .payload_length         = 32,                               \
                                .selective_auto_ack     = false                             \
}


/** @brief Default legacy radio parameters. Identical to the nRF24Lxx defaults. */
pub(crate) const NRF_ESB_LEGACY_CONFIG  {.protocol               = NRF_ESB_PROTOCOL_ESB,             \
                                .mode                   = NRF_ESB_MODE_PTX,                 \
                                .event_handler          = 0,                                \
                                .bitrate                = NRF_ESB_BITRATE_2MBPS,            \
                                .crc                    = NRF_ESB_CRC_8BIT,                 \
                                .tx_output_power        = NRF_ESB_TX_POWER_0DBM,            \
                                .retransmit_delay       = 600,                              \
                                .retransmit_count       = 3,                                \
                                .tx_mode                = NRF_ESB_TXMODE_AUTO,              \
                                .radio_irq_priority     = 1,                                \
                                .event_irq_priority     = 2,                                \
                                .payload_length         = 32,                               \
                                .selective_auto_ack     = false                             \
}


/** @brief Macro to create an initializer for a TX data packet.
*
* @details This macro generates an initializer. Using the initializer is more efficient
*          than setting the individual parameters dynamically.
*
* @param[in]   _pipe   The pipe to use for the data packet.
* @param[in]   ...     Comma separated list of character data to put in the TX buffer.
*                      Supported values consist of 1 to 63 characters.
*
* @return  Initializer that sets up the pipe, length, and byte array for content of the TX data.
 */
pub(crate) const NRF_ESB_CREATE_PAYLOAD(_pipe, ...)                                                  \
        {.pipe = _pipe, .length = NUM_VA_ARGS(__VA_ARGS__), .data = {__VA_ARGS__}};         \
        assert!(NUM_VA_ARGS(__VA_ARGS__) > 0 && NUM_VA_ARGS(__VA_ARGS__) <= 63)


/**@brief Enhanced ShockBurst protocols. */
typedef enum {
    NRF_ESB_PROTOCOL_ESB,      /**< Enhanced ShockBurst with fixed payload length.                                            */
    NRF_ESB_PROTOCOL_ESB_DPL   /**< Enhanced ShockBurst with dynamic payload length.                                          */
} nrf_esb_protocol_t;


/**@brief Enhanced ShockBurst modes. */
typedef enum {
    NRF_ESB_MODE_PTX,          /**< Primary transmitter mode. */
    NRF_ESB_MODE_PRX           /**< Primary receiver mode.    */
} nrf_esb_mode_t;


/**@brief Enhanced ShockBurst bitrate modes. */
typedef enum {
    NRF_ESB_BITRATE_2MBPS     = RADIO_MODE_MODE_Nrf_2Mbit,      /**< 2 Mb radio mode.                                                */
    NRF_ESB_BITRATE_1MBPS     = RADIO_MODE_MODE_Nrf_1Mbit,      /**< 1 Mb radio mode.                                                */
#if defined(RADIO_MODE_MODE_Nrf_250Kbit)
    NRF_ESB_BITRATE_250KBPS   = RADIO_MODE_MODE_Nrf_250Kbit,    /**< 250 Kb radio mode.                                              */
#endif //!( defined(NRF52840_XXAA) || defined(NRF52810_XXAA) || defined(NRF52811_XXAA) )
    NRF_ESB_BITRATE_1MBPS_BLE = RADIO_MODE_MODE_Ble_1Mbit,      /**< 1 Mb radio mode using @e Bluetooth low energy radio parameters. */
#if defined(RADIO_MODE_MODE_Ble_2Mbit)
    NRF_ESB_BITRATE_2MBPS_BLE = RADIO_MODE_MODE_Ble_2Mbit       /**< 2 Mb radio mode using @e Bluetooth low energy radio parameters. */
#endif
} nrf_esb_bitrate_t;


/**@brief Enhanced ShockBurst CRC modes. */
typedef enum {
    NRF_ESB_CRC_16BIT = RADIO_CRCCNF_LEN_Two,                   /**< Use two-byte CRC. */
    NRF_ESB_CRC_8BIT  = RADIO_CRCCNF_LEN_One,                   /**< Use one-byte CRC. */
    NRF_ESB_CRC_OFF   = RADIO_CRCCNF_LEN_Disabled               /**< Disable CRC.      */
} nrf_esb_crc_t;


/**@brief Enhanced ShockBurst radio transmission power modes. */
typedef enum {
#if defined(RADIO_TXPOWER_TXPOWER_Pos8dBm)
    NRF_ESB_TX_POWER_8DBM     = RADIO_TXPOWER_TXPOWER_Pos8dBm,  /**< 8 dBm radio transmit power.   */
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos7dBm)
    NRF_ESB_TX_POWER_7DBM     = RADIO_TXPOWER_TXPOWER_Pos7dBm,  /**< 7 dBm radio transmit power.   */
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos6dBm)
    NRF_ESB_TX_POWER_6DBM     = RADIO_TXPOWER_TXPOWER_Pos6dBm,  /**< 6 dBm radio transmit power.   */
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos5dBm)
    NRF_ESB_TX_POWER_5DBM     = RADIO_TXPOWER_TXPOWER_Pos5dBm,  /**< 5 dBm radio transmit power.   */
#endif
    NRF_ESB_TX_POWER_4DBM     = RADIO_TXPOWER_TXPOWER_Pos4dBm,  /**< 4 dBm radio transmit power.   */
#if defined(RADIO_TXPOWER_TXPOWER_Pos3dBm)
    NRF_ESB_TX_POWER_3DBM     = RADIO_TXPOWER_TXPOWER_Pos3dBm,  /**< 3 dBm radio transmit power.   */
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos2dBm)
    NRF_ESB_TX_POWER_2DBM     = RADIO_TXPOWER_TXPOWER_Pos2dBm,  /**< 2 dBm radio transmit power.   */
#endif
    NRF_ESB_TX_POWER_0DBM     = RADIO_TXPOWER_TXPOWER_0dBm,     /**< 0 dBm radio transmit power.   */
    NRF_ESB_TX_POWER_NEG4DBM  = RADIO_TXPOWER_TXPOWER_Neg4dBm,  /**< -4 dBm radio transmit power.  */
    NRF_ESB_TX_POWER_NEG8DBM  = RADIO_TXPOWER_TXPOWER_Neg8dBm,  /**< -8 dBm radio transmit power.  */
    NRF_ESB_TX_POWER_NEG12DBM = RADIO_TXPOWER_TXPOWER_Neg12dBm, /**< -12 dBm radio transmit power. */
    NRF_ESB_TX_POWER_NEG16DBM = RADIO_TXPOWER_TXPOWER_Neg16dBm, /**< -16 dBm radio transmit power. */
    NRF_ESB_TX_POWER_NEG20DBM = RADIO_TXPOWER_TXPOWER_Neg20dBm, /**< -20 dBm radio transmit power. */
    NRF_ESB_TX_POWER_NEG30DBM = RADIO_TXPOWER_TXPOWER_Neg30dBm, /**< -30 dBm radio transmit power. */
    NRF_ESB_TX_POWER_NEG40DBM = RADIO_TXPOWER_TXPOWER_Neg40dBm  /**< -40 dBm radio transmit power. */
} nrf_esb_tx_power_t;


/**@brief Enhanced ShockBurst transmission modes. */
pub(crate) enum nrf_esb_tx_mode_t {
    NRF_ESB_TXMODE_AUTO,        /**< Automatic TX mode: When the TX FIFO contains packets and the radio is idle, packets are sent automatically. */
    NRF_ESB_TXMODE_MANUAL,      /**< Manual TX mode: Packets are not sent until @ref nrf_esb_start_tx is called. This mode can be used to ensure consistent packet timing. */
    NRF_ESB_TXMODE_MANUAL_START /**< Manual start TX mode: Packets are not sent until @ref nrf_esb_start_tx is called. Then, transmission continues automatically until the TX FIFO is empty. */
}


/**@brief Enhanced ShockBurst event IDs used to indicate the type of the event. */
pub(crate) enum nrf_esb_evt_id_t
{
    NRF_ESB_EVENT_TX_SUCCESS,   /**< Event triggered on TX success.     */
    NRF_ESB_EVENT_TX_FAILED,    /**< Event triggered on TX failure.     */
    NRF_ESB_EVENT_RX_RECEIVED   /**< Event triggered on RX received.    */
}


/**@brief Enhanced ShockBurst payload.
 *
 * @details The payload is used both for transmissions and for acknowledging a
 *          received packet with a payload.
 */
pub(crate) struct nrf_esb_payload_t
{
    u8 lengthL,                                 //!< Length of the packet (maximum value is @ref NRF_ESB_MAX_PAYLOAD_LENGTH).
    u8 pipe,                                   //!< Pipe used for this payload.
    int8_t  rssi,                                   //!< RSSI for the received packet.
    u8 noack,                                  //!< Flag indicating that this packet will not be acknowledgement. Flag is ignored when selective auto ack is enabled.
    u8 pid,                                    //!< PID assigned during communication.
    u8 data[NRF_ESB_MAX_PAYLOAD_LENGTH],       //!< The payload data.
} 


/**@brief Enhanced ShockBurst event. */
pub(crate) struct nrf_esb_evt_t
{
    evt_id: nrf_esb_evt_id_t    ,                    //!< Enhanced ShockBurst event ID.
             tx_attempts: u32  ,              //!< Number of TX retransmission attempts.
}


/**@brief Definition of the event handler for the module. */
typedef void (* nrf_esb_event_handler_t)(nrf_esb_evt_t const * p_event);


/**@brief Main configuration structure for the module. */
pub(crate) struct nrf_esb_config_t
{
         protocol:      nrf_esb_protocol_t,           //!< Enhanced ShockBurst protocol.
              mode: nrf_esb_mode_t   ,                //!< Enhanced ShockBurst mode.
     event_handler: nrf_esb_event_handler_t,          //!< Enhanced ShockBurst event handler.

    // General RF parameters
          bitrate: nrf_esb_bitrate_t ,                //!< Enhanced ShockBurst bitrate mode.
               crc: nrf_esb_crc_t,                    //!< Enhanced ShockBurst CRC mode.

          tx_output_power: nrf_esb_tx_power_t,        //!< Enhanced ShockBurst radio transmission power mode.

                   retransmit_delay: uint16_t ,       //!< The delay between each retransmission of unacknowledged packets.
    uint16_t                retransmit_count,       //!< The number of retransmission attempts before transmission fail.

    // Control settings
           tx_mode: nrf_esb_tx_mode_t,                //!< Enhanced ShockBurst transmission mode.

                     radio_irq_priority: u8,     //!< nRF radio interrupt priority.
                 event_irq_priority: u8,     //!< ESB event interrupt priority.
               payload_length: u8,         //!< Length of the payload (maximum length depends on the platforms that are used on each side).

                    selective_auto_ack: bool,     //!< Enable or disable selective auto acknowledgement. When this feature is disabled, all packets will be acknowledged ignoring the noack field.
} ;


// Code below is similar to `nrf_esb_error_codes.h`.


pub(crate) const NRF_ERROR_BUFFER_EMPTY: u8 = 0x0100;

pub(crate) const NRF_ESB_ERROR_NOT_IN_RX_MODE: u8 = 0x0101;


// Code below is similar to `nrf_esb_resources.h`.
// if ESB_ALTERNATIVE_RESOURCES {
pub(crate) const ESB_PPI_CHANNELS_USED: u8 = 0x00000007; /**< PPI channels used by ESB (not available to the application). */
pub(crate) const ESB_TIMERS_USED: u8 = 0x00000004; /**< Timers used by ESB. */
pub(crate) const ESB_SWI_USED: u8 = 0x00000001; /**< Software interrupts used by ESB. */
// } else {
// eg `if ESB_ALTERNATIVE_RESOURCES` {`
pub(crate) const ESB_PPI_CHANNELS_USED_ALTERNATIVE: u8 = 0x00000700; /**< PPI channels used by ESB (not available to the application). */
pub(crate) const ESB_TIMERS_USED_ALTERNATIVE: u8 = 0x00000001; /**< Timers used by ESB. */
pub(crate) const ESB_SWI_USED_ALTERNATIVE: u8 = 0x00000002; /**< Software interrupts used by ESB. */
// }