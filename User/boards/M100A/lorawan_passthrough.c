/*
2016 Winext

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Hellogz
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "board.h"


/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           10000000  // 10 [s] value in us

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

#if defined( USE_BAND_868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

/*!
 * User application data buffer size
 */
#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )

#define LORAWAN_APP_DATA_SIZE                       16

#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )

#define LORAWAN_APP_DATA_SIZE                       11

#endif


/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Device states
 */
static enum eDevicState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
} DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
} ComplianceTest;

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    switch ( port )
    {
    case 222:
        AppDataSize = copy_usart_received_data(AppData);
        break;
    case 224:
        if ( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch ( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if ( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = DR_0;
    }
    else
    {
        if ( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = get_channel_data_rate();//DR_0;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.nbRetries = 8;
            mcpsReq.Req.Confirmed.Datarate = get_channel_data_rate();//DR_0;
        }
    }
    if ( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if ( status == LORAMAC_STATUS_OK )
    {
        if ( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
            enable_usart_state(true);
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] McpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *McpsConfirm )
{
    show_lora_mac_status("In McpsConfirm", McpsConfirm->Status);

    enable_usart_state(true);

    if ( McpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
				rf_send_ok_ack();
        switch ( McpsConfirm->McpsRequest )
        {
        case MCPS_UNCONFIRMED:
        {
            // Check Datarate
            // Check TxPower
            int8_t TxPowers[]    = { 20, 14, 11,  8,  5,  2 };
            DBG("MCPS_UNCONFIRMED[Datarate:%d, TxPower:%ddBm]\r\n", McpsConfirm->Datarate, TxPowers[McpsConfirm->TxPower]);
            break;
        }
        case MCPS_CONFIRMED:
        {
            // Check Datarate
            // Check TxPower
            // Check AckReceived
            // Check NbRetries
            DBG("MCPS_CONFIRMED[Datarate:%d, TxPower:%ddB, AckReceived:%d, NbRetries:%d]\r\n",
                McpsConfirm->Datarate,
                McpsConfirm->TxPower,
                McpsConfirm->AckReceived,
                McpsConfirm->NbRetries);
            break;
        }
        case MCPS_PROPRIETARY:
        {
            DBG("MCPS_PROPRIETARY\r\n");
            break;
        }
        default:
            break;
        }
    }
    else
    {
        rf_send_fail_ack();
    }
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] McpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *McpsIndication )
{
    show_lora_mac_status("In McpsIndication", McpsIndication->Status);
    if ( McpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch ( McpsIndication->McpsIndication )
    {
    case MCPS_UNCONFIRMED:
    {
        DBG("MCPS_UNCONFIRMED\r\n");
        break;
    }
    case MCPS_CONFIRMED:
    {
        DBG("MCPS_CONFIRMED\r\n");
        break;
    }
    case MCPS_PROPRIETARY:
    {
        DBG("MCPS_PROPRIETARY\r\n");
        break;
    }
    case MCPS_MULTICAST:
    {
        DBG("MCPS_MULTICAST\r\n");
        break;
    }
    default:
        break;
    }

    // Check Multicast
    DBG("Check Multicast:%d\r\n", McpsIndication->Multicast);
    // Check Port
    DBG("Check Port:%d\r\n", McpsIndication->Port);
    // Check Datarate
    DBG("Check RxDatarate:%d\r\n", McpsIndication->RxDatarate);
    // Check FramePending
    DBG("Check FramePending:%d\r\n", McpsIndication->FramePending);
    // Check Buffer
    // Check BufferSize
    print_message("Check Buffer:", McpsIndication->Buffer, McpsIndication->BufferSize, 'X');
    // Check Rssi
    DBG("Check Rssi:%d\r\n", McpsIndication->Rssi);
    // Check Snr
    DBG("Check Snr:%d\r\n", McpsIndication->Snr);
    // Check RxSlot
    DBG("Check RxSlot:%d\r\n", McpsIndication->RxSlot);

    if ( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if ( McpsIndication->RxData == true )
    {
        switch ( McpsIndication->Port )
        {
				case 222:
						send_rf_data(McpsIndication->Buffer, McpsIndication->BufferSize);
						break;
        case 224:
            if ( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if ( ( McpsIndication->BufferSize == 4 ) &&
                        ( McpsIndication->Buffer[0] == 0x01 ) &&
                        ( McpsIndication->Buffer[1] == 0x01 ) &&
                        ( McpsIndication->Buffer[2] == 0x01 ) &&
                        ( McpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                }
            }
            else
            {
                ComplianceTest.State = McpsIndication->Buffer[0];
                switch ( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = McpsIndication->BufferSize;

                    AppData[0] = 4;
                    for ( uint8_t i = 1; i < AppDataSize; i++ )
                    {
                        AppData[i] = McpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                {
                    MlmeReq_t mlmeReq;
                    mlmeReq.Type = MLME_LINK_CHECK;
                    LoRaMacMlmeRequest( &mlmeReq );
                }
                break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] MlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *MlmeConfirm )
{
    show_lora_mac_status("In MlmeConfirm", MlmeConfirm->Status);
    if ( MlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch ( MlmeConfirm->MlmeRequest )
        {
        case MLME_JOIN:
        {
            // Status is OK, node has joined the network
            DBG("Status is OK, node has joined the network\r\n");
            break;
        }
        case MLME_LINK_CHECK:
        {
            // Check DemodMargin
            // Check NbGateways
            DBG("MLME_LINK_CHECK\r\n");
            if ( ComplianceTest.Running == true )
            {
                DBG("ComplianceTest.LinkCheck = true\r\n");
                ComplianceTest.LinkCheck = true;
                ComplianceTest.DemodMargin = MlmeConfirm->DemodMargin;
                ComplianceTest.NbGateways = MlmeConfirm->NbGateways;
            }
            break;
        }
        default:
            DBG("MlmeConfirm->MlmeRequest:%d\r\n", MlmeConfirm->MlmeRequest);
            break;
        }
    }
		else
		{
				OnTxNextPacketTimerEvent();
		}
}

/**
 * Main application entry point.
 */
void lorawan_usart_passthrough_process( void )
{
		uint8_t join_count = 0;
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;

    passthrough_init( );
	
		DBG("Firmware version: v1.0.0\r\n");

    check_run_work_mode();

    m100a_self_test();

    DeviceState = DEVICE_STATE_INIT;

    while ( 1 )
    {
        switch ( DeviceState )
        {
        case DEVICE_STATE_INIT:
        {
            LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
            LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
            LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
            LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );

            TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

            config_initialization();

#if defined( USE_BAND_868 )
            LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
            DeviceState = DEVICE_STATE_JOIN;
            break;
        }
        case DEVICE_STATE_JOIN:
        {
            if (check_activation())
            {
                if ( NextTx == true )
                {
                    ota_join_req();
										DBG("NextTx = true, Join Count:%d\r\n", ++join_count);
                }
								else
								{
										DBG("NextTx = false, Join Request\r\n");
								}

                // Schedule next packet transmission
                TxDutyCycleTime = OVER_THE_AIR_ACTIVATION_DUTYCYCLE;
                DeviceState = DEVICE_STATE_CYCLE;
            }
            else
            {
                DeviceState = DEVICE_STATE_SEND;
            }
            break;
        }
        case DEVICE_STATE_SEND:
        {
            if ( NextTx == true )
            {
                set_app_port( &AppPort );

                PrepareTxFrame( AppPort );

                NextTx = false;
                SendFrame( );
                DBG("Send RF Frame\r\n");
                clear_usart_received_data_flag();
            }
            DeviceState = DEVICE_STATE_SLEEP;
            break;
        }
        case DEVICE_STATE_CYCLE:
        {
            // Schedule next packet transmission
            TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
            TimerStart( &TxNextPacketTimer );
            DeviceState = DEVICE_STATE_SLEEP;
            break;
        }
        case DEVICE_STATE_SLEEP:
        {
            // Wake up through events
            TimerLowPowerHandler( );
            break;
        }
        default:
        {
            DeviceState = DEVICE_STATE_INIT;
            break;
        }
        }
        if ( check_usart_received_data_flag( ) == true )
        {
            if (check_join_network())
            {
                NextTx = true;
                enable_usart_state(false);
                TimerStop( &TxNextPacketTimer );
                DeviceState = DEVICE_STATE_SEND;
            }
        }
    }
}
