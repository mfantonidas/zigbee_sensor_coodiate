/****************************************************************************
 *
 * MODULE:			   WSN - Router
 *
 * COMPONENT:          $RCSfile: WSN_Router.c,v $
 *
 * VERSION:            $Name:  $
 *
 * REVISION:           $Revision: 1.7 $
 *
 * DATED:              $Date: 2007/07/12 11:03:03 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:             IDM
 *
 * DESCRIPTION:
 *
 * Implements a Wireless Sensor Network Router using the Jennic Zigbee stack.
 * Reads temperature, humidity and battery voltage and transmits these to
 * network coordinator. Assumes code is running on a evaluation kit sensor
 * board.
 *
 *
 * Update history
 * $Log: WSN_Router.c,v $
 * Revision 1.7  2007/07/12 11:03:03  ndani
 * Add simple descriptor after network has started
 *
 * Revision 1.6  2007/07/12 10:03:34  ndani
 *
 *
 * LAST MODIFIED BY:   $Author: ndani $
 *                     $Modtime: $
 *
 ****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on each
 * copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd 2005, 2006, 2007. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <ALSdriver.h>
#include <HTSdriver.h>
#include <LedControl.h>
#include <AppHardwareApi.h>
#include <JZ_Api.h>
#include <gdb.h>
#include <AppApi.h>
#include "WSN_Profile.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* Timing values */
#define APP_TICK_PERIOD_ms		  600
#define APP_TICK_PERIOD     	  (APP_TICK_PERIOD_ms * 32)

#define APP_DATA_SEND_PERIOD_ms	  1000
#define APP_DATA_SEND_PERIOD	  (APP_DATA_SEND_PERIOD_ms / APP_TICK_PERIOD_ms)

#define DTIME 10

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/* Battery reading state definitions */
typedef enum
{
	E_STATE_READ_BATT_VOLT_IDLE,
	E_STATE_READ_BATT_VOLTS_ADC_CONVERTING,
	E_STATE_READ_BATT_VOLTS_COMPLETE,
    E_STATE_READ_BATT_VOLTS_READY
}teStateReadBattVolt;

/* Temperature/Humidity Sensor - reading state definitions */
typedef enum
{
	E_STATE_READ_TEMP_HUMID_IDLE,
	E_STATE_READ_HUMID_RUNNING,
	E_STATE_READ_TEMP_HUMID_COMPLETE,
	E_STATE_READ_TEMP_START,
	E_STATE_READ_TEMP_HUMID_RUNNING,
	E_STATE_READ_TEMP_COMPLETE,
	E_STATE_READ_TEMP_HUMID_READY
}teStateReadTempHumidity;

typedef enum
{
    multicatch = 10,
    singlecatch = 20
}idleCoefficient;

typedef enum
{
    UNSTARTED,
    WAITING_MODE,
    SINGLE_MODE,
    MULTI_MODE
}dataType;

/* Battery measurement data */
typedef struct
{
	uint16 					u16Reading;
	teStateReadBattVolt 	eState;
}tsBattSensor;

/* Temp/Humidity measurement data */
typedef struct
{
	uint16 					u16TempReading;
	uint16 					u16HumidReading;
	teStateReadTempHumidity eState;
}tsTempHumiditySensor;

typedef struct
{
    MAC_ExtAddr_s sExtAddr;
    int sensorID;
}tsSensorMac;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vInit(void);
PRIVATE void vSendData(void);
PRIVATE void vSendState(void);
PRIVATE void vInitSensors(void);
PRIVATE void vReadTempHumidity(void);
PRIVATE void vReadBatteryVoltage(void);
PRIVATE void vAppTick(void *pvMsg, uint8 u8Param);

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE uint8 u8AppTicks = 0;
PRIVATE tsBattSensor sBattSensor;
PRIVATE tsTempHumiditySensor sTempHumiditySensor;
PRIVATE tsSensorMac sSensorMac;
PRIVATE bool_t bAppTimerStarted = FALSE;
PRIVATE bool_t bNwkJoined = FALSE;
PRIVATE bool_t isStarted = TRUE;
PRIVATE idleCoefficient coefficient = multicatch;
PRIVATE int tempdt;
PRIVATE dataType datatype;
/****************************************************************************
 *
 * NAME: AppColdStart
 *
 * DESCRIPTION:
 * Entry point for application. Initialises system, starts scan then
 * processes interrupts.
 *
 * RETURNS:
 * void, never returns
 *
 ****************************************************************************/
PUBLIC void AppColdStart(void)
{
	/* Debug hooks: include these regardless of whether debugging or not */
	HAL_GDB_INIT();
    HAL_BREAKPOINT();

    /* General initialisation: reset hardware */
    JZS_sConfig.u32Channel 	= WSN_CHANNEL;
    JZS_sConfig.u16PanId 	= WSN_PAN_ID;

    /* General initialisation: reset hardware */
    vInit();

    /* No return from the above function call */
}

/****************************************************************************
 *
 * NAME: AppWarmStart
 *
 * DESCRIPTION:
 * Entry point for application from boot loader. Simply jumps to AppColdStart
 * as, in this instance, application will never warm start.
 *
 * RETURNS:
 * Never returns.
 *
 ****************************************************************************/
PUBLIC void AppWarmStart(void)
{
    AppColdStart();
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: vInit
 *
 * DESCRIPTION:
 * Initialises Zigbee stack and hardware. Final action is to start BOS, from
 * which there is no return. Subsequent application actions occur in the
 * functions defined above.
 *
 * RETURNS:
 * No return from this function
 *
 ****************************************************************************/
PRIVATE void vInit(void)
{

    /* Initialise Zigbee stack */
    JZS_u32InitSystem(TRUE);

    /* Set DIO for LEDs */
    vLedInitRfd();
    vLedControl(0,0);
    vLedControl(1,0);

    /* Set sensors */
    vInitSensors();
    datatype = WAITING_MODE;
    /* Start BOS */
    (void)bBosRun(TRUE);

    /* No return from the above function call */
}

/****************************************************************************
 *
 * NAME: vInitSensors
 *
 * DESCRIPTION:
 * Initialise the temperature/humidity sensor and set the ADC to measure the
 * supply voltage.
 *
 ****************************************************************************/
PRIVATE void vInitSensors(void)
{
    /* Initialise temp/humidity sensor interface */
    vHTSreset();
    sTempHumiditySensor.eState = E_STATE_READ_TEMP_HUMID_IDLE;

    /* Initialise ADC for internal battery voltage measurement */
	sBattSensor.eState = E_STATE_READ_BATT_VOLT_IDLE;

	vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE,
	                 E_AHI_AP_INT_DISABLE,
	                 E_AHI_AP_SAMPLE_2,
	                 E_AHI_AP_CLOCKDIV_2MHZ,
	                 E_AHI_AP_INTREF);

    /* Wait until the analogue peripheral regulator has come up before setting
       the ADC. */
    while(!bAHI_APRegulatorEnabled());

    vAHI_AdcEnable(E_AHI_ADC_CONVERT_DISABLE,
                   E_AHI_AP_INPUT_RANGE_2,
                   E_AHI_ADC_SRC_VOLT);

    //sSensorMac.sExtAddr.u32L = *(uint32 *)(0xf0000004);
    //sSensorMac.sExtAddr.u32H = *(uint32 *)(0xf0000000);

}

/****************************************************************************
 *
 * NAME: vAppTick
 *
 * DESCRIPTION:
 *
 * Called by a BOS timer expiry. Reads sensor data and if complete transmits
 * to coordinator.
 *
 ****************************************************************************/
PRIVATE void vAppTick(void *pvMsg, uint8 u8Param)
{
    uint8 u8Msg;
    uint8 u8TimerId;
    static bool_t bToggle;

	/* Read sensor data */
	vReadTempHumidity();
	vReadBatteryVoltage();

	if (u8AppTicks++ > APP_DATA_SEND_PERIOD)
	{
	    /* If sensor reads are compete */
	    if ((sBattSensor.eState == E_STATE_READ_BATT_VOLTS_READY) &&
            (sTempHumiditySensor.eState == E_STATE_READ_TEMP_HUMID_READY))
        {
		    /* Toggle LED1 to show we are alive */
		    if (bToggle)
		    {
		    	vLedControl(0,0);
		    }
		    else
		    {
		    	vLedControl(0,1);
		    }
		    bToggle = !bToggle;

		    u8AppTicks = 0;

            /* Transmit data to coordinator */
		    if(datatype == WAITING_MODE)
                vSendState();
            else if(datatype == SINGLE_MODE || datatype == MULTI_MODE)
                vSendData();

           	sBattSensor.eState = E_STATE_READ_BATT_VOLT_IDLE;
            sTempHumiditySensor.eState = E_STATE_READ_TEMP_HUMID_IDLE;
		}
	}
    (void)bBosCreateTimer(vAppTick, &u8Msg, 0, (APP_TICK_PERIOD_ms / coefficient), &u8TimerId);
}

/****************************************************************************
 *
 * NAME: vReadBatteryVoltage
 *
 * DESCRIPTION:
 *
 * Uses ADC to read supply voltage. Measurement is performed using a state
 * machine to ensure that it never blocks.
 *
 ****************************************************************************/
PRIVATE void vReadBatteryVoltage(void)
{
    uint16 u16AdcReading;

	switch(sBattSensor.eState)
	{
		case E_STATE_READ_BATT_VOLT_IDLE:
	    	vAHI_AdcStartSample();
	    	sBattSensor.eState = E_STATE_READ_BATT_VOLTS_ADC_CONVERTING;
			break;

		case E_STATE_READ_BATT_VOLTS_ADC_CONVERTING:
	    	if (!bAHI_AdcPoll())
	    	{
	    	    sBattSensor.eState = E_STATE_READ_BATT_VOLTS_COMPLETE;
	    	}
			break;

		case E_STATE_READ_BATT_VOLTS_COMPLETE:

		    u16AdcReading = u16AHI_AdcRead();

		    /* Input range is 0 to 2.4V. ADC has full scale range of 12 bits.
		       Therefore a 1 bit change represents a voltage of approx 586uV */
		    sBattSensor.u16Reading = ((uint32)((uint32)(u16AdcReading * 586) +
                                     ((uint32)(u16AdcReading * 586) >> 1)))  /
                                     1000;

	    	sBattSensor.eState = E_STATE_READ_BATT_VOLTS_READY;
			break;

		case E_STATE_READ_BATT_VOLTS_READY:
			break;

		default:
			break;
	}
}

/****************************************************************************
 *
 * NAME: vReadTempHumidity
 *
 * DESCRIPTION:
 *
 * Read temperature/humidity sensor. Reading is performed using a state machine
 * to ensure that it never blocks.
 *
 ****************************************************************************/
PRIVATE void vReadTempHumidity(void)
{
    switch(sTempHumiditySensor.eState)
	{
		case E_STATE_READ_TEMP_HUMID_IDLE:
		    vHTSstartReadHumidity();
			sTempHumiditySensor.eState = E_STATE_READ_HUMID_RUNNING;
		break;

		case E_STATE_READ_HUMID_RUNNING:
			if ((u32AHI_DioReadInput() & HTS_DATA_DIO_BIT_MASK) == 0)
			{
				sTempHumiditySensor.eState = E_STATE_READ_TEMP_HUMID_COMPLETE;
			}
			break;

		case E_STATE_READ_TEMP_HUMID_COMPLETE:
			sTempHumiditySensor.u16HumidReading = u16HTSreadHumidityResult();
			sTempHumiditySensor.eState = E_STATE_READ_TEMP_START;
			break;

		case E_STATE_READ_TEMP_START:
		    vHTSstartReadTemp();
			sTempHumiditySensor.eState = E_STATE_READ_TEMP_HUMID_RUNNING;
			break;

		case E_STATE_READ_TEMP_HUMID_RUNNING:
			if ((u32AHI_DioReadInput() & HTS_DATA_DIO_BIT_MASK) == 0)
			{
				sTempHumiditySensor.eState = E_STATE_READ_TEMP_COMPLETE;
			}
			break;

		case E_STATE_READ_TEMP_COMPLETE:
			sTempHumiditySensor.u16TempReading = u16HTSreadTempResult();
			sTempHumiditySensor.eState = E_STATE_READ_TEMP_HUMID_READY;
			break;

		case E_STATE_READ_TEMP_HUMID_READY:
			break;

		default:
			break;
	}
}

/****************************************************************************
 *
 * NAME: vSendData
 *
 * DESCRIPTION:
 *
 * Transmit sensor data to coordinator.
 *
 ****************************************************************************/
PRIVATE void vSendData(void)
{
    int i;
    uint8  MacAddress[8];
    void   *pu8ExtAdr;
    AF_Transaction_s asTransaction[1];


    asTransaction[0].u8SequenceNum = u8AfGetTransactionSequence(TRUE);
    asTransaction[0].uFrame.sMsg.u8TransactionDataLen = 16;

    asTransaction[0].uFrame.sMsg.au8TransactionData[0] = datatype;
    asTransaction[0].uFrame.sMsg.au8TransactionData[1] = sBattSensor.u16Reading;
	asTransaction[0].uFrame.sMsg.au8TransactionData[2] = sBattSensor.u16Reading >> 8;
    asTransaction[0].uFrame.sMsg.au8TransactionData[3] = sTempHumiditySensor.u16TempReading;
	asTransaction[0].uFrame.sMsg.au8TransactionData[4] = sTempHumiditySensor.u16TempReading >> 8;
    asTransaction[0].uFrame.sMsg.au8TransactionData[5] = sTempHumiditySensor.u16HumidReading;
	asTransaction[0].uFrame.sMsg.au8TransactionData[6] = sTempHumiditySensor.u16HumidReading >> 8;
	pu8ExtAdr = pvAppApiGetMacAddrLocation();
     /* Load extended address into frame payload */
    for (i = 0; i < 8; i++)
    {
        MacAddress[i] = *( (uint8*)pu8ExtAdr + i);
    }
	asTransaction[0].uFrame.sMsg.au8TransactionData[7] = MacAddress[0];
	asTransaction[0].uFrame.sMsg.au8TransactionData[8] = MacAddress[1];
	asTransaction[0].uFrame.sMsg.au8TransactionData[9] = MacAddress[2];
	asTransaction[0].uFrame.sMsg.au8TransactionData[10] = MacAddress[3];
	asTransaction[0].uFrame.sMsg.au8TransactionData[11] = MacAddress[4];
	asTransaction[0].uFrame.sMsg.au8TransactionData[12] = MacAddress[5];
	asTransaction[0].uFrame.sMsg.au8TransactionData[13] = MacAddress[6];
	asTransaction[0].uFrame.sMsg.au8TransactionData[14] = MacAddress[7];
	asTransaction[0].uFrame.sMsg.au8TransactionData[15] = sSensorMac.sensorID;

    //if(isStarted)
    //{
        (void)afdeDataRequest(APS_ADDRMODE_SHORT,       /* Address type */
                          0x0000,                   /* Destination address */
                          WSN_DATA_SINK_ENDPOINT,   /* destination endpoint */
                          WSN_DATA_SOURCE_ENDPOINT, /* Source endpoint */
                          WSN_PROFILE_ID,           /* Profile ID */
                          WSN_CID_SENSOR_READINGS,  /* Cluster ID */
                          AF_MSG,                   /* Frame type */
                          1,                        /* Transactions */
                          asTransaction,            /* Transaction data */
                          APS_TXOPTION_NONE,        /* Transmit options */
                          SUPPRESS_ROUTE_DISCOVERY, /* Route discovery mode */
                          0);                       /* Radius count */
    //}
}

PRIVATE void vSendState(void)
{
    int i;
    uint8  MacAddress[8];
    void   *pu8ExtAdr;
    AF_Transaction_s asTransaction[1];

    asTransaction[0].u8SequenceNum = u8AfGetTransactionSequence(TRUE);
    asTransaction[0].uFrame.sMsg.u8TransactionDataLen = 16;
    pu8ExtAdr = pvAppApiGetMacAddrLocation();
    for (i = 0; i < 8; i++)
    {
        MacAddress[i] = *( (uint8*)pu8ExtAdr + i);
    }
    asTransaction[0].uFrame.sMsg.au8TransactionData[0] = CONTROL_CMD_GET_INFO;
	asTransaction[0].uFrame.sMsg.au8TransactionData[1] = MacAddress[0];
	asTransaction[0].uFrame.sMsg.au8TransactionData[2] = MacAddress[1];
	asTransaction[0].uFrame.sMsg.au8TransactionData[3] = MacAddress[2];
	asTransaction[0].uFrame.sMsg.au8TransactionData[4] = MacAddress[3];
	asTransaction[0].uFrame.sMsg.au8TransactionData[5] = MacAddress[4];
	asTransaction[0].uFrame.sMsg.au8TransactionData[6] = MacAddress[5];
	asTransaction[0].uFrame.sMsg.au8TransactionData[7] = MacAddress[6];
	asTransaction[0].uFrame.sMsg.au8TransactionData[8] = MacAddress[7];

    (void)afdeDataRequest(APS_ADDRMODE_SHORT,       /* Address type */
                          0x0000,                   /* Destination address */
                          WSN_DATA_SINK_ENDPOINT,   /* destination endpoint */
                          WSN_DATA_SOURCE_ENDPOINT, /* Source endpoint */
                          WSN_PROFILE_ID,           /* Profile ID */
                          WSN_CID_SENSOR_READINGS,  /* Cluster ID */
                          AF_MSG,                   /* Frame type */
                          1,                        /* Transactions */
                          asTransaction,            /* Transaction data */
                          APS_TXOPTION_NONE,        /* Transmit options */
                          SUPPRESS_ROUTE_DISCOVERY, /* Route discovery mode */
                          0);                       /* Radius count */
}
/****************************************************************************/
/***               Functions called by the stack                          ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: JZA_vZdpResponse
 *
 * DESCRIPTION:
 * Called when a ZDP response frame has been received. In this application no
 * action is taken as no ZDP responses are anticipated.
 *
 * PARAMETERS:      Name           RW  Usage
 *                  u8Type         R   ZDP response type
 *                  pu8Payload     R   Payload buffer
 *                  u8PayloadLen   R   Length of payload
 *
 ****************************************************************************/
PUBLIC void JZA_vZdpResponse(uint8  u8Type,
                             uint8  u8LQI,
                             uint8 *pu8Payload,
                             uint8  u8PayloadLen)

{
}

/****************************************************************************
 *
 * NAME: JZA_pu8AfMsgObject
 *
 * DESCRIPTION:
 * Called when a MSG transaction has been received with a matching endpoint.
 * In this application no action is taken as no MSG transactions are expected.
 *
 * PARAMETERS:      Name           RW  Usage
 *                  afSrcAddr      R   Address of sender device
 *                  dstEndPoint    R   Endpoint at receiver
 *                  clusterID      R   Pointer to cluster ID
 *                  afduLength     R   Pointer to length of data
 *                  pAfdu          R   Data array
 *
 * RETURNS:
 * NULL
 *
 ****************************************************************************/
PUBLIC bool_t JZA_bAfMsgObject(APS_Addrmode_e eAddrMode,
                               uint16 u16AddrSrc,
                               uint8 u8SrcEP,
                               uint8 u8LQI,
                               uint8 u8DstEP,
                               uint8 u8ClusterID,
                               uint8 *pu8ClusterIDRsp,
                               AF_Transaction_s *puTransactionInd,
                               AF_Transaction_s *puTransactionRsp)
{
    uint8 data;
    if ((eAddrMode == APS_ADDRMODE_SHORT) && (u8DstEP == WSN_DATA_SOURCE_ENDPOINT))
    {

        if(u8ClusterID == WSN_CID_SENSOR_READINGS)
        {
            data = puTransactionInd->uFrame.sMsg.au8TransactionData[0];
            tempdt = puTransactionInd->uFrame.sMsg.au8TransactionData[0];
            if(data == CONTROL_CMD_GET_INFO)
            {
                datatype = WAITING_MODE;
                coefficient = multicatch;
            }
            else if(data == CONTROL_CMD_GET_DATA_S)
            {
                datatype = SINGLE_MODE;
                coefficient = singlecatch;
            }
            else if(data == CONTROL_CMD_GET_DATA_M)
            {
                datatype = MULTI_MODE;
                coefficient = multicatch;
            }
            else
                return;
        }
    }
    return 0;
}

/****************************************************************************
 *
 * NAME: JZA_vAfKvpResponse
 *
 * DESCRIPTION:
 * Called after a KVP transaction with acknowledgement request, when the
 * acknowledgement arrives. In this application no action is taken as no
 * KVP transaction acknowledgements are expected.
 *
 * PARAMETERS:      Name                   RW  Usage
 *                  srcAddressMod          R   Address of sender device
 *                  transactionSequenceNum R   KVP transaction number
 *                  commandTypeIdentifier  R   KVP command type
 *                  dstEndPoint            R   Endpoint at receiver
 *                  clusterID              R   Cluster ID
 *                  attributeIdentifier    R   KVP attribute ID
 *                  errorCode              R   Result code
 *                  afduLength             R   Length of payload data
 *                  pAfdu                  R   Payload data array
 *
 ****************************************************************************/
PUBLIC void JZA_vAfKvpResponse(APS_Addrmode_e eAddrMode,
                               uint16 u16AddrSrc,
                               uint8 u8SrcEP,
                               uint8 u8LQI,
                               uint8 u8DstEP,
                               uint8 u8ClusterID,
                               AF_Transaction_s *puTransactionInd)
{
}

/****************************************************************************
 *
 * NAME: JZA_eAfKvpObject
 *
 * DESCRIPTION:
 * Called when a KVP transaction has been received with a matching endpoint.
 *
 * PARAMETERS:      Name           RW  Usage
 *                  afSrcAddr      R   Address of sender device
 *                  u8DstEndpoint  R   Endpoint at receiver
 *                  pu8ClusterId   R   Pointer to cluster ID
 *                  eCommandTypeId R   KVP command type
 *                  u16AttributeId R   KVP attribute ID
 *                  pu8AfduLength  R   Pointer to length of data
 *                  pu8Afdu        R   Data array
 *
 * RETURNS:
 * AF_ERROR_CODE
 *
 ****************************************************************************/
PUBLIC bool_t JZA_bAfKvpObject(APS_Addrmode_e eAddrMode,
                               uint16 u16AddrSrc,
                               uint8 u8SrcEP,
                               uint8 u8LQI,
                               uint8 u8DstEP,
                               uint8 u8ClusterId,
                               uint8 *pu8ClusterIDRsp,
                               AF_Transaction_s *puTransactionInd,
                               AF_Transaction_s *puTransactionRsp)
{
    return KVP_SUCCESS;
}

/****************************************************************************
 *
 * NAME: JZA_vAppDefineTasks
 *
 * DESCRIPTION:
 * Called by Zigbee stack during initialisation to allow the application to
 * initialise any tasks that it requires. This application requires none.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void JZA_vAppDefineTasks(void)
{
}

/****************************************************************************
 *
 * NAME: JZA_vPeripheralEvent
 *
 * DESCRIPTION:
 * Called when a hardware event causes an interrupt. This function is called
 * from within the interrupt context so should be brief. In this case, the
 * information is placed on a simple FIFO queue to be processed later.
 *
 * PARAMETERS: Name          RW  Usage
 *             u32Device     R   Peripheral generating interrupt
 *             u32ItemBitmap R   Bitmap of interrupt sources within peripheral
 *
 ****************************************************************************/
PUBLIC void JZA_vPeripheralEvent(uint32 u32Device, uint32 u32ItemBitmap)
{
}

/****************************************************************************
 *
 * NAME: JZA_vAppEventHandler
 *
 * DESCRIPTION:
 * Called regularly by the task scheduler. This function reads the hardware
 * event queue and processes the events therein. It is important that this
 * function exits after a relatively short time so that the other tasks are
 * not adversely affected.
 *
 ****************************************************************************/
PUBLIC void JZA_vAppEventHandler(void)
{
    uint8 u8Msg;
    uint8 u8TimerId;

    if (!bAppTimerStarted)
    {
        if (bNwkJoined)
        {
            bAppTimerStarted = TRUE;
            (void)bBosCreateTimer(vAppTick, &u8Msg, 0, (APP_TICK_PERIOD_ms / 10), &u8TimerId);
        }
    }
}

/****************************************************************************
 *
 * NAME: JZA_boAppStart
 *
 * DESCRIPTION:
 * Called by Zigbee stack during initialisation.
 *
 * RETURNS:
 * TRUE
 *
 ****************************************************************************/
PUBLIC bool_t JZA_boAppStart(void)
{
    JZS_vStartStack();
    return TRUE;
}

/****************************************************************************
 *
 * NAME: JZA_vStackEvent
 *
 * DESCRIPTION:
 * Called by Zigbee stack to pass an event up to the application.
 *
 * RETURNS:
 * TRUE
 *
 ****************************************************************************/
PUBLIC void JZA_vStackEvent(teJZS_EventIdentifier eEventId,
                            tuJZS_StackEvent *puStackEvent)
{
    if (eEventId == JZS_EVENT_NWK_JOINED_AS_ROUTER)
    {
        uint8 u8InputClusterCnt      = 1;
		uint8 au8InputClusterList[]  = {WSN_CID_SENSOR_READINGS};
		uint8 u8OutputClusterCnt     = 1;
		uint8 au8OutputClusterList[] = {WSN_CID_SENSOR_READINGS};

		(void)afmeAddSimpleDesc(WSN_DATA_SOURCE_ENDPOINT,
								WSN_PROFILE_ID,
								0x0000,
								0x00,
								0x00,
								u8InputClusterCnt,
								au8InputClusterList,
								u8OutputClusterCnt,
								au8OutputClusterList);

        bNwkJoined = TRUE;
    }
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
