/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "app_pwm.h"
#include "nrf_drv_gpiote.h"

#include "nrf_fstorage_sd.h"
#include "nrf_fstorage.h"

#include "oled.h"
#include "nrf_drv_saadc.h"


#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */
#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256     /**< UART RX buffer size. */
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

nrf_ppi_channel_t my_ppi_channel=NRF_PPI_CHANNEL0 ;
nrf_ppi_channel_t my_ppi_channel2=NRF_PPI_CHANNEL1;
//ID对应Timer的ID，如NRF_DRV_TIMER_INSTANCE(0)对应Timer0
const nrf_drv_timer_t timer0 = NRF_DRV_TIMER_INSTANCE(0);
const nrf_drv_timer_t timer1 = NRF_DRV_TIMER_INSTANCE(1);
const nrf_drv_timer_t timer2 = NRF_DRV_TIMER_INSTANCE(2);

#define INPUT  30		//输入捕获引脚
#define OUTPUT  31	//PWM输出引脚

static uint16_t count=0;	//脉冲计数值
static uint16_t fre=0;		//测量所得频率


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
		
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

/////////////////////////////////////// 外设部分
//saadc回调函数
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
}
//saadc初始化
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
            NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);   //采集通道P0.02
	//初始化saadc
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

	//加入通道配置
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
}

//定时器2定时1s触发频率计算
void timer2_event_handler(nrf_timer_event_t event_type, void* p_context)
{
		uint16_t time=1;		//定时1s
		
		fre=count/time;							//计算频率
		printf("Current fre: %d Hz\n",fre);
	
		nrfx_timer_clear(&timer0);	//清除定时器0的计数值
}
//定时器2定时功能初始化
void tim2_init(void){
    uint32_t time_ms = 1000; //定时器2定时时间
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
	
		    //配置定时器相关事件
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&timer2, &timer_cfg, timer2_event_handler);
    APP_ERROR_CHECK(err_code);
    //计算CC寄存器里存的值
    time_ticks = nrf_drv_timer_ms_to_ticks(&timer2, time_ms);
   //触发定时器比较
    nrf_drv_timer_extended_compare(
         &timer2, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}

APP_PWM_INSTANCE(PWM1,1);                   // 创建一个使用定时器1产生PWM波的实例

static volatile bool ready_flag;            // 使用一个标志位表示PWM状态

void pwm_ready_callback(uint32_t pwm_id)    // PWM回调功能
{
    ready_flag = true;
}

//PWM1脉冲输出
void PWM_OUT(uint32_t value)
{

  ret_code_t err_code;
    
    /* 1个频道的 PWM, 50KHz（1000 000/20=50 000). */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(20L, OUTPUT);
    
   
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    
    /* 初始化和使能PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);//使能PWM

    while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY);  
		//PWM1的通道0即为P0.02
}

//gpiote输入作为count事件，即记录定时器2定时时间内的PWM脉冲数量
static void gpio_init(void)
{
    ret_code_t err_code;
    //GPIOE驱动初始化
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
  
    //配置设置GPIOE输入参数
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(1);//采集就是占空比变化的次数
    in_config.pull = NRF_GPIO_PIN_PULLUP;
     //GPIOE输入初始化，设置触发输入中断
    err_code = nrf_drv_gpiote_in_init(INPUT, &in_config, NULL);
    APP_ERROR_CHECK(err_code);
    //设置GPIOE输入事件使能
    nrf_drv_gpiote_in_event_enable(INPUT, 1);
}

//Timer事件回调函数
void my_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{

}
	
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
//定时器0输入捕获功能初始化
 void timer0_init(void)
{
//    NRF_TIMER0->MODE    = TIMER_MODE_MODE_Counter;      // Set the timer in counter Mode.
//    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_24Bit;  // 24-bit mode.
	  uint32_t err_code = NRF_SUCCESS;

	
	  //定义定时器配置结构体，并使用默认配置参数初始化结构体
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;	
	  //Timer0配置为计数模式
    timer_cfg.mode = NRF_TIMER_MODE_COUNTER;


	  //初始化定时器，定时器工作于计数模式时，没有事件，所以无需回调函数
    err_code = nrfx_timer_init(&timer0, &timer_cfg, my_timer_event_handler);
	  //err_code = nrfx_timer_init(&TIMER_COUNTER, &timer_cfg, NULL);
    APP_ERROR_CHECK(err_code);
}

/** @brief 初始化PPI外设.
*/
static void ppi_init(void)
{
    uint32_t err_code = NRF_SUCCESS;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);


    APP_ERROR_CHECK(err_code);

    // Configure 2nd available PPI channel to start timer0 counter at TIMER2 COMPARE[0] match, which is every odd number of seconds.
    err_code = nrf_drv_ppi_channel_alloc(&my_ppi_channel);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(my_ppi_channel,
                                           nrf_drv_gpiote_in_event_addr_get(INPUT),
                                           nrf_drv_timer_task_address_get(&timer0, NRF_TIMER_TASK_COUNT));
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_enable(my_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

//oled屏幕初始化
void oled_init(void)
{
  nrf_gpio_cfg_output(LCD_SCL);
	nrf_gpio_cfg_output(LCD_SDA);
	nrf_gpio_cfg_output(LCD_RST);
	nrf_gpio_cfg_output(LCD_DC);
	nrf_gpio_cfg_output(LCD_CS);
	nrf_gpio_cfg_output(LCD_POWER_EN);
	nrf_gpio_pin_clear(LCD_CS);
	nrf_gpio_pin_set(LCD_POWER_EN);
  LCD_Init(); //oled 初始化  
}

//flash的fstorage功能
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x2c000,
    .end_addr   = 0x2ffff,
};

/* Dummy data to write to flash. */
static uint32_t m_data          = 0xBAD;
static uint32_t m_data2;  

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
//        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
//            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
//                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
//            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
//                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
       sd_app_evt_wait();
    }
}
#define FLASH_ADDR 0x2c000 //确定删除页的地址
static uint32_t pages_to_erase = 1;//确定删除页的个数

void flash_init(void){
    ret_code_t rc;

    nrf_fstorage_api_t * p_fs_api;
	  p_fs_api = &nrf_fstorage_sd;//处理操作类型设置
	
	  rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);//flash处理的开始地址和结束地址初始化
    APP_ERROR_CHECK(rc);
	
	  (void) nrf5_flash_end_addr_get();//获取地址，判断为可写地址大小
		

   rc = nrf_fstorage_erase(
                            &fstorage, /* 前面声明的fstorage实例  */
                            FLASH_ADDR, /*要擦除的flash页面的地址。*/
                            pages_to_erase, /* 要删除的页数。*/
                            NULL /*可选参数*/
                            );
   APP_ERROR_CHECK(rc);

}
void flash_test(void)
{
    ret_code_t rc;

    rc = nrf_fstorage_write(&fstorage, 0x2c000, &m_data, sizeof(m_data), NULL);//向0x3e000写入数据m_data数据
    APP_ERROR_CHECK(rc);
    wait_for_flash_ready(&fstorage);//等待写完
		printf("write:%x",m_data);

	  nrf_fstorage_read(&fstorage, 0x2c000,
                              &m_data2,
                             sizeof(m_data2));//从0x3e000地址读取数据
		printf("read:%x",m_data2);

}
//flash写入函数
void flash_write(float *va,int *q){
		ret_code_t rc;
			
		m_data=(uint32_t)(*va*1000);
		rc = nrf_fstorage_write(&fstorage, FLASH_ADDR+*q*100,&m_data, sizeof(m_data2), NULL);
		APP_ERROR_CHECK(rc);
		wait_for_flash_ready(&fstorage);//等待写完
	  //printf("write:%d\r\n",m_data2);	
}
//flash读出函数
void flash_read(){
		int i=0;
		for(;i<10;i++){
		nrf_fstorage_read(&fstorage, FLASH_ADDR+100*i,
                              &m_data2,
                             sizeof(m_data2));//从0x3e000地址读取数据
		nrf_delay_us(2000);
		printf("read%d:%fV\r\n",i,(float)m_data2/1000);	
		}
}
//void adc_sample(float *va,int *p,int *q){
//		nrf_saadc_value_t  saadc_val; //adc采集值
//	  float  val;  //保存SAADC采样数据计算的实际电压值

//						       //启动一次ADC采样。
//		nrf_drv_saadc_sample_convert(0,&saadc_val);
//			  //串口输出ADC采样值。
//		val = saadc_val * 3.6 /1024;	//参考电压0.6v，增益1/6，精确度10位
//			  //延时300ms，方便观察SAADC采样数据
//		if(*p<2000){
//			*va=val+*va;
//		}
//		else{
//			*va=*va/2000;
//			printf("val:%fv\r\n",*va);	
//	  	
//			if(*q<=10){
//			flash_write(va,q);
//				*q=*q+1;
//			}
//			else
//				*q=0;
//			
//			flash_read();
//			*p=0;
//			*va=0;
//		}
//		*p=*p+1;
//}
		
void adc_sample(float *va,int *p,int *q){
		nrf_saadc_value_t  saadc_val; //adc采集值
	  float  val;  //保存SAADC采样数据计算的实际电压值

						       //启动一次ADC采样。
		nrf_drv_saadc_sample_convert(0,&saadc_val);
			  //串口输出ADC采样值。
		val = saadc_val * 3.6 /1024;	//参考电压0.6v，增益1/6，精确度10位
			  //延时300ms，方便观察SAADC采样数据
		if(*p<2000){
			*va=val+*va;
		}
		else{
			*va=*va/2000;
			printf("val:%fv\r\n",*va);	
	  	
			if(*q<=10){						//依次将10次采集到的电压值写入flash中
				flash_write(va,q);	//*va是计算得到的平均电压值，*q是写入的次数
				*q=*q+1;
			}
			else
				*q=0;
			
			flash_read();		//读取写入flash中的数据
			*p=0;
			*va=0;
		}
		*p=*p+1;
}
//void find_max(float *va,float *va_ago,uint8_t *duty,uint8_t *duty_ago){
//		if(*va> *va_ago){
//			if(*duty>*duty_ago){
//				*duty_ago=*duty;
//				*duty=*duty+1;
//			}
//			else if(*duty<*duty_ago){
//				*duty_ago=*duty;
//				*duty=*duty-1;
//			}
//			else
//				*duty=*duty+1;
//		}
//		else if(*va<*va_ago){ 
//			if(*duty>*duty_ago){
//				*duty_ago=*duty;
//				*duty=*duty-1;
//			}
//			else if(*duty<*duty_ago){
//				*duty_ago=*duty;
//				*duty=*duty+1;
//			}
//			else
//				*duty=*duty+1;
//		}
//		
//		if(*duty<1){
//			*duty=1;
//		}
//		if(*duty>99){
//			*duty=99;
//		}
//		
//		app_pwm_channel_duty_set(&PWM1, 0, *duty);
//		printf("duty:%d %\r\n",*duty);	
//		nrf_delay_us(50000);//100000
//}

/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;
		int i=0;
		int j=0;
		float va=0;
		float va_ago=0;
		uint8_t duty=50;
		uint8_t duty_ago=0;
		
    // Initialize.
    uart_init();
//    log_init();
//    timers_init();
////    buttons_leds_init(&erase_bonds);
////		power_management_init();
//    ble_stack_init();
//		
//    gap_params_init();
//    gatt_init();
//    services_init();
//    advertising_init();
//    conn_params_init();
//		advertising_start();

		//外设部分	
    timer0_init(); // Timer used to blink the LEDs.
	  gpio_init();
    ppi_init();    // PPI to redirect the event to timer start/stop tasks.
    PWM_OUT(50);	//10为占空比
		tim2_init();
		flash_init();
		oled_init();
		saadc_init();
			//启动定时器
		nrf_drv_timer_enable(&timer0);
		nrf_drv_timer_enable(&timer2);
		LCD_P8x16Str(2,4,"PWM "); 	//2，4是在OLED屏中的像素坐标x，y 
		
//	flash_test();

    // Enter main loop.
    while(1)
    {	
			//idle_state_handle();
			//adc_sample(&va,&i,&j);	
			adc_sample(&va,&i,&j);
			//find_max(&va,&va_ago,&duty,&duty_ago);
			count=(int)nrf_drv_timer_capture(&timer0,NRF_TIMER_CC_CHANNEL1);	
			//定时器0的通道1即P0.03进行输入捕获功能

    }
}

