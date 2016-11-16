#include "non-lorawan_passthrough.h"


/*!
 * Tx output powers table definition
 */
const int8_t M100A_TxPowers[]    = { 20, 14, 11,  8,  5,  2 };

/*!
 * Data rates table definition
 */
const uint8_t M100A_Datarates[]  = { 12, 11, 10,  9,  8,  7,  7, 50 };

static TimerEvent_t tx_delay_timer;
static TimerEvent_t rx_window_timer1;
static TimerEvent_t rx_window_timer2;

static RadioEvents_t radio_events;

struct m100a_config_t_ m100a_config_malloc = {0};
m100a_config_t m100a_config = &m100a_config_malloc;

static uint8_t rf_buffer[255] = {0};
static uint8_t rf_buffer_size = 0;

static enum e_lofi_mac_state
{
    MAC_IDLE          = 0x00,
    MAC_TX_RUNNING    = 0x01,
    MAC_RX            = 0x02,
    MAC_ACK_REQ       = 0x04,
    MAC_ACK_RETRY     = 0x08,
    MAC_TX_DELAYED    = 0x10,
}lofi_mac_state;

static enum e_device_state
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_RECV,
    DEVICE_STATE_SLEEP
}device_state;

void set_rf_buffer(uint8_t *buffer, uint8_t size)
{
	rf_buffer_size = size;
	memcpy1(rf_buffer, buffer, rf_buffer_size);
}

uint8_t get_com_baud(void)
{
	return m100a_config->com_baud;
}

uint8_t get_com_parity(void)
{
	return m100a_config->com_parity;
}

uint8_t get_com_data_bits(void)
{
	return m100a_config->com_data_bits;
}

uint8_t get_com_stop_bits(void)
{
	return m100a_config->com_stop_bits;
}

void non_lorawan_use_config_param(uint8_t *param)   // param length must is CONFIG_PARAM_SIZE
{
    uint8_t index = 0;
    
    index = 0;
    memcpyr((uint8_t *)&m100a_config->rf_frequency, param+index, 4);
    index += 4;
    m100a_config->rf_tx_power = param[index];
    index += 1;
    m100a_config->rf_datarate = param[index];
    index += 1;
    m100a_config->rf_bandwidth = param[index];
    index += 1;
    m100a_config->rf_syncword = param[index];
    index += 1;
    memcpy1((uint8_t *)&m100a_config->deveui, param+index, 8);
    index += 8;
    m100a_config->EnableDebugPrintf = param[index];
    index += 1;
    
    m100a_config->rf_preamble_len = param[index];
    index += 1;
    m100a_config->rf_header_mode = param[index];
    index += 1;
    m100a_config->rf_coderate = param[index];
    index += 1;
    m100a_config->rf_crc_enable = param[index]?true:false;
    index += 1;
    m100a_config->com_baud = param[index];
    index += 1;
    m100a_config->com_parity = param[index];
    index += 1;
    m100a_config->com_data_bits = param[index];
    index += 1;
    m100a_config->com_stop_bits = param[index];
		index += 1;
    m100a_config->rf_rx_iq_signal = param[index]?true:false;
    index += 1;
    m100a_config->rf_tx_iq_signal = param[index]?true:false;
}

void show_non_lorawan_config_param(void)
{
	DBG("======================================================================\r\n");					
	print_message("DevEui:", m100a_config->deveui, 8, 'X');
	DBG("Freq:%d\r\n", m100a_config->rf_frequency);
	DBG("Channel Datarate Index:%d\r\n", m100a_config->rf_datarate);
	DBG("Channel TxPower Index:%d\r\n", m100a_config->rf_tx_power);
	DBG("Bandwidth:%d\r\n", m100a_config->rf_bandwidth);
	DBG("Syncword:%02X\r\n", m100a_config->rf_syncword);
	DBG("Preamble_Length:%d\r\n", m100a_config->rf_preamble_len);
	DBG("Header Mode:%d\r\n", m100a_config->rf_header_mode);
	DBG("Code Rate:4/%d\r\n", m100a_config->rf_coderate+5);
	DBG("CRC Endble:%d\r\n", m100a_config->rf_crc_enable);
	DBG("RX IQ Signal:%d\r\n", m100a_config->rf_rx_iq_signal);
	DBG("TX IQ Signal:%d\r\n", m100a_config->rf_tx_iq_signal);
	DBG("Debug Switch:%d\r\n", m100a_config->EnableDebugPrintf);
	DBG("======================================================================\r\n");
}

//param: RFLR_MODEMCONFIG1_IMPLICITHEADER_ON or RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF
void set_lora_header_mode(uint8_t mode)
{
    Radio.SetModem(MODEM_LORA);
    
    if(mode == RFLR_MODEMCONFIG1_IMPLICITHEADER_ON)
    {
      Radio.Write( REG_LR_MODEMCONFIG1, 
                         ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_ON );
    }
    else
    {
      Radio.Write( REG_LR_MODEMCONFIG1, 
                         ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF );
    }
}

void set_lora_syncword(uint8_t syncword)
{
    Radio.SetModem(MODEM_LORA);

    Radio.Write(REG_LR_SYNCWORD, syncword);
}

lofi_mac_status_t send_frame_on_channel(uint32_t freq, uint32_t datarate, uint32_t bandwidth, uint8_t tx_power)
{
    Radio.SetChannel( freq );
    Radio.SetMaxPayloadLength( MODEM_LORA, rf_buffer_size );
    Radio.SetTxConfig( MODEM_LORA, tx_power, 0, m100a_config->rf_bandwidth, 
                      datarate, m100a_config->rf_coderate, m100a_config->rf_preamble_len, 
                      false, m100a_config->rf_crc_enable, false, 0, m100a_config->rf_tx_iq_signal, 10e6 );	// 10e6 for send 252 byte can't timeout.
    m100a_config->tx_time_on_air = Radio.TimeOnAir( MODEM_LORA, rf_buffer_size );

    Radio.Send( rf_buffer, rf_buffer_size );

    lofi_mac_state |= MAC_TX_RUNNING;
    
    DBG("Freq:%d, SF%dBW%d, Power:%d ", (uint16_t)(freq / 100000), (uint8_t)datarate, (uint8_t)bandwidth, tx_power);
    
    print_message("Send Frame:", rf_buffer, rf_buffer_size, 'X');

    return LOFIMAC_STATUS_OK;
}

void rx_window_setup(uint32_t freq, uint32_t datarate)
{

    if(Radio.GetStatus() == RF_IDLE)
    {
        Radio.SetChannel(freq);

        Radio.SetRxConfig( MODEM_LORA, m100a_config->rf_bandwidth, datarate, m100a_config->rf_coderate, 
                          0, m100a_config->rf_preamble_len, 5, false, 0, 
                          m100a_config->rf_crc_enable, false, 0, m100a_config->rf_rx_iq_signal, true );
			
        Radio.SetMaxPayloadLength( MODEM_LORA, LOFIMAC_MAXPAYLOAD );
        
        Radio.Rx(NON_LORAWAN_MAX_RX_WINDOW);
        
        switch(m100a_config->rf_bandwidth)
        {
        case 0:
            DBG("[%d, SF%dBW125K, SyncWord:0x%02X]Start RF Recv...\r\n", freq, (uint8_t)datarate, m100a_config->rf_syncword);
            break;
        case 1:
            DBG("[%d, SF%dBW250K, SyncWord:0x%02X]Start RF Recv...\r\n", freq, (uint8_t)datarate, m100a_config->rf_syncword);
            break;
        case 2:
            DBG("[%d, SF%dBW500K, SyncWord:0x%02X]Start RF Recv...\r\n", freq, (uint8_t)datarate, m100a_config->rf_syncword);
            break;
        default:
            DBG("[%d, SF%dBW%d, SyncWord:0x%02X]Start RF Recv...\r\n", freq, (uint8_t)datarate, m100a_config->rf_bandwidth, m100a_config->rf_syncword);
            break;
        }
    }
}

uint32_t set_next_channel(void)
{
    uint32_t next_tx_delay = 0;

    // ???????????

    return next_tx_delay;
}

lofi_mac_status_t schedule_tx(void)
{
    uint32_t duty_cycle_time_off = 0;

    duty_cycle_time_off = set_next_channel();

    if(duty_cycle_time_off)
    {
        lofi_mac_state |= MAC_TX_DELAYED;
        TimerSetValue(&tx_delay_timer, duty_cycle_time_off);
        TimerStart(&tx_delay_timer);
        return LOFIMAC_STATUS_OK;
    }
    else
    {
        return send_frame_on_channel(m100a_config->rf_frequency, M100A_Datarates[m100a_config->rf_datarate], m100a_config->rf_bandwidth, M100A_TxPowers[m100a_config->rf_tx_power]);
    }
}

void on_radio_tx_done(void)
{
    DBG("On Tx Done\r\n");
    rf_send_ok_ack();
    device_state = DEVICE_STATE_RECV;
}

void on_radio_rx_done( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    TimerStop(&rx_window_timer1);
    TimerStop(&rx_window_timer2);
    
    Radio.Sleep( );
    
    DBG("On Rx Done [%dB, RSSI:%d, SNR:%d]\r\n", size, rssi, snr);
    
    print_message("Received Frame:", payload, size, 'X');

    send_rf_data(payload, size );

    device_state = DEVICE_STATE_RECV;
}

void on_radio_tx_timeout(void)
{	
    Radio.Sleep();
    
    DBG("On Tx Timeout\r\n");
    rf_send_fail_ack();
    device_state = DEVICE_STATE_RECV;
}

void on_radio_rx_error(void)
{
    Radio.Sleep();
    
    DBG("On Rx Error\r\n");

    device_state = DEVICE_STATE_RECV;
}

void on_radio_rx_timeout(void)
{
    Radio.Sleep();
    
    DBG("On Rx Timeout\r\n");

    device_state = DEVICE_STATE_RECV;
}

void on_tx_delay_timer_event(void)
{
    TimerStop(&tx_delay_timer);
    lofi_mac_state &= ~MAC_TX_DELAYED;

    schedule_tx();
}

void on_rx_window1_timer_event(void)
{
    TimerStop(&rx_window_timer1);
    //rx window setup and rx runing

    m100a_config->rx_slot = 0;
    rx_window_setup(m100a_config->rf_frequency, M100A_Datarates[m100a_config->rf_datarate]);
}

void on_rx_window2_timer_event(void)
{
    TimerStop(&rx_window_timer2);
    //rx window setup and rx runing

    m100a_config->rx_slot = 1;
    rx_window_setup(m100a_config->rf_frequency, M100A_Datarates[m100a_config->rf_datarate]);
}

void non_lorawan_mac_initialization(void)
{  
		passthrough_init();
		
		DBG("Firmware version: v1.0.0\r\n");
																				 
		check_run_work_mode();

    TimerInit(&tx_delay_timer, on_tx_delay_timer_event);
    TimerInit(&rx_window_timer1, on_rx_window1_timer_event);
    TimerInit(&rx_window_timer2, on_rx_window2_timer_event);

    radio_events.TxDone = on_radio_tx_done;
    radio_events.RxDone = on_radio_rx_done;
    radio_events.RxError = on_radio_rx_error;
    radio_events.TxTimeout = on_radio_tx_timeout;
    radio_events.RxTimeout = on_radio_rx_timeout;
    Radio.Init(&radio_events);
    
    set_lora_header_mode(m100a_config->rf_header_mode);
    set_lora_syncword(m100a_config->rf_syncword);
    
    Radio.Sleep();

    lofi_mac_state = MAC_IDLE;
    
    DBG("LoRa MAC Layer Init OK\r\n");
}

void non_lorawan_usart_passthrough_process(void)
{
    device_state = DEVICE_STATE_INIT;
    
    while(1)
    {
        switch(device_state)
        {
        case DEVICE_STATE_INIT:
            non_lorawan_mac_initialization();
            device_state = DEVICE_STATE_RECV;
            break;
        case DEVICE_STATE_SEND:
            schedule_tx();
						clear_usart_received_data_flag();
            device_state = DEVICE_STATE_SLEEP;
            break;
        case DEVICE_STATE_RECV:
						enable_usart_state(true);
            rx_window_setup(m100a_config->rf_frequency, M100A_Datarates[m100a_config->rf_datarate]);
            device_state = DEVICE_STATE_SLEEP;
            break;
        case DEVICE_STATE_SLEEP:
            TimerLowPowerHandler( );
            break;
        default:
            device_state = DEVICE_STATE_INIT;
            break;
        }
				if( check_usart_received_data_flag( ) == true )
				{
						enable_usart_state(false);
						device_state = DEVICE_STATE_SEND;
				}
    }
}
