/****************************************************************************
 *
 * MODULE:			   WSN - Coordinator
 *
 * COMPONENT:          $RCSfile: WSN_Coordinator.c,v $
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
 * Implements a Wireless Sensor Network Coordinator Node using Jennic Zigbee
 * stack. Receives data from compatible nodes via the radio and retransmits to
 * to host using UART.
 *
 * Update history
 * $Log: WSN_Coordinator.c,v $
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
#include <Utilities.h>
#include <JZ_Api.h>
#include <AppApi.h>
#include <gdb.h>
#include "..\..\..\Chip\Common\Include\Printf.h"


#include "WSN_Profile.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* Timing values */
#define APP_TICK_PERIOD_ms			 1500
#define APP_TICK_PERIOD     	  (APP_TICK_PERIOD_ms * 6)

#define APP_DATA_SEND_PERIOD_ms	  1000
#define APP_DATA_SEND_PERIOD	  (APP_DATA_SEND_PERIOD_ms / APP_TICK_PERIOD_ms)
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
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

/*fill state of sensor data*/
typedef enum
{
    E_FILL_COMPLETE,
    E_FILL_UNCOMPLETE
}teStateDataFill;

typedef enum
{
    UNSTARTED,
    WAITING_MODE,
    SINGLE_MODE,
    MULTI_MODE
}dataType;

typedef enum
{
    IS_NULL,
    NOT_NULL
}isAvailable;

/* Temp/Humidity measurement data */
typedef struct
{
	uint16 					u16TempReading;
	uint16 					u16HumidReading;
	teStateReadTempHumidity eState;
}tsTempHumiditySensor;

typedef struct
{
    uint num;
    bool_t enable;
    teStateDataFill estate;
}sensorsState;

typedef struct
{
    uint16 tempdata;
    uint16 humidata;
    bool_t isValiable;
    int sensorID;
    MAC_ExtAddr_s sExtAddr;
    isAvailable available;
    uint8 templength;
    uint8 humilength;
}multiData;

/*typedef struct
{
    multiData multidata[6];

};*/

typedef struct
{
    uint16 tempdata[6];
    uint16 humidata[6];
    int sensorID;
    bool_t isFilled;
    sensorsState estate;
    uint8 templength[6];
    uint8 humilength[6];
}singleData;

typedef struct
{
    bool_t isValiable;
    int sensorID;
    uint32 mach, macl;
}sensorInfo;

typedef struct
{
    int sensorID;
    sensorInfo sensorinfo[6];
    dataType datatype;
    int sensorCount;
    union {
        singleData sd;
        multiData md[6];
    }data;
}sensorData;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vInit(void);
PRIVATE void vToggleLed(void *pvMsg, uint8 u8Dummy);
PRIVATE void vTxSerialDataFrame(uint16 count,
                                uint16 u16Humidity,
                                uint16 u16Temperature,
                                uint32 sensorMach,
                                uint32 sensorMacl);
PRIVATE void vInitSensors(void);
PRIVATE void vReadTempHumidity(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE uint8 u8AppTicks = 0;
PRIVATE bool_t bNwkStarted = FALSE;
PRIVATE bool_t bAppTimerStarted = FALSE;
PRIVATE tsTempHumiditySensor sTempHumiditySensor;
PRIVATE sensorsState sensorstate[3];
PRIVATE bool_t armStarted = 0;
PRIVATE uint8 armCmdRsv;
PRIVATE singleData singledata;
PRIVATE multiData multidata[6];
PRIVATE MAC_ExtAddr_s sExtAddr;
PRIVATE bool_t commandChanged = FALSE;
PRIVATE dataType runmode = WAITING_MODE;
PRIVATE sensorData sensordata;
PRIVATE bool_t getinfo_ok = FALSE;
PRIVATE enum {start = 103, single = 49, mulit = 50, stop = 115,reflash = 114}armCmd = reflash;
/****************************************************************************
 *
 * NAME: AppColdStart
 *
 * DESCRIPTION:
 * Entry point for application from boot loader. Initialises system and runs
 * main loop.
 *
 * RETURNS:
 * Never returns.
 *
 ****************************************************************************/
PUBLIC void AppColdStart(void)
{
	/* Debug hooks: include these regardless of whether debugging or not */
	HAL_GDB_INIT();
    HAL_BREAKPOINT();

	/* Set network information */
	JZS_sConfig.u32Channel = WSN_CHANNEL;
	JZS_sConfig.u16PanId   = WSN_PAN_ID;

    /* General initialisation */
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
    int i;
    int machh, machl, maclh, macll;
    uint8  MacAddress[8];
    void   *pu8ExtAdr;
    /* Initialise Zigbee stack */
    JZS_u32InitSystem(TRUE);

    /* Set DIO for LEDs */
    vLedInitFfd();

    vLedControl(0,0);
    vLedControl(1,0);
    vLedControl(2,0);
    vLedControl(3,0);

    /* Intialise serial comms unless debug mode*/
    #ifndef GDB
        vUART_printInit();

    #endif

    /* Set sensors */
    vInitSensors();
    singledata.estate.estate = E_FILL_UNCOMPLETE;
    for(i = 0; i < 6; i++)
    {
        sensordata.data.sd.tempdata[i] = 0;
        sensordata.data.md[i].tempdata = 0;
        sensordata.data.sd.humidata[i] = 0;
        sensordata.data.md[i].humidata = 0;
        sensordata.sensorinfo[i].sensorID = 0;
        sensordata.sensorinfo[i].isValiable = FALSE;
        sensordata.sensorCount = 0;
        sensordata.datatype = UNSTARTED;
    }

    //sExtAddr.u32L = *(uint32 *)(0xf0000004);
    //sExtAddr.u32H = *(uint32 *)(0xf0000000);
    //vPrintf("%x, %x \n", sExtAddr.u32H, sExtAddr.u32L);



  /* Set pointer to point to location in internal RAM where extended address is stored */
      // pu8ExtAdr = pvAppApiGetMacAddrLocation();
     /* Load extended address into frame payload */
      // for (i = 0; i < 8; i++)
      //  {
      //   MacAddress[i] = *( (uint8*)pu8ExtAdr + i);
      //   vPrintf(" %x",MacAddress[i] );
      //  }
      //  vPrintf("\n");

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
}



/****************************************************************************
 *
 * NAME: vTxSerialDataFrame
 *
 * DESCRIPTION:
 * Transmits node data (address and sensor readings) to host via serial port.
 *
 * PARAMETERS: Name           RW  Usage
 *             u16NodeId      R   Short address of node that generated the data
 *             u16Humidity    R   Reading from humidity sensor (%)
 *             u16Temperature R   Reading from temperature sensor (degrees C)
 *             u16BattVoltage R   ADC reading of supply voltage (mv)
 *
 ****************************************************************************/
PRIVATE void vTxSerialDataFrame(uint16 count,
                                uint16 u16Humidity,
                                uint16 u16Temperature,
                                uint32 sensorMach,
                                uint32 sensorMacl)
{
    #ifndef GDB
        vPrintf("\n\rS%d,", count);
        vPrintf("\n\rH%d,", u16Humidity);
        vPrintf("\n\rT%d,", u16Temperature);
        vPrintf("\n\rMac:%x %x,", sensorMach, sensorMacl);
	#endif
}

/****************************************************************************
 *
 * NAME: vToggleLed
 *
 * DESCRIPTION:
 * Gets called by a BOS timer. Toggles LED1 to indicate we are alive.
 *
 ****************************************************************************/
PRIVATE void vToggleLed(void *pvMsg, uint8 u8Dummy)
{
    uint8 u8Msg;
    uint8 u8TimerId;
    static int count = 3;
    static bool_t bToggle;
    static int fillcount = 0;
    MAC_ExtAddr_s sExtAddr, *psExtAddr;

    int i;
    AF_Transaction_s asTransaction[1];


    asTransaction[0].u8SequenceNum = u8AfGetTransactionSequence(TRUE);
    asTransaction[0].uFrame.sMsg.u8TransactionDataLen = 1;

    asTransaction[0].uFrame.sMsg.au8TransactionData[0] = 9;
    //static int sensorID[3] = {0, };

    vReadTempHumidity();

    if (bToggle)
    {
        vLedControl(0,0);
    }
    else
    {
        vLedControl(0,1);
    }
    bToggle = !bToggle;
    /* if(afdeDataRequest(APS_ADDRMODE_SHORT,
                          0x0001,
                          WSN_DATA_SOURCE_ENDPOINT,
                          WSN_DATA_SINK_ENDPOINT,
                          WSN_PROFILE_ID,
                          WSN_CID_SENSOR_READINGS,
                          AF_MSG,
                          1,
                          asTransaction,
                          APS_TXOPTION_NONE,
                          SUPPRESS_ROUTE_DISCOVERY,
                          0)){;}
        armCmd = single;*/
    if(commandChanged == TRUE)
    {
        //vPrintf("command changed!\n");
        commandChanged = FALSE;
        switch(armCmd)
        {
            case reflash:
                //vPrintf("reflashed\n");
                sensordata.datatype = WAITING_MODE;
                for(i = 0; i < 6; i++)
                {
                    if(sensordata.sensorinfo[i].isValiable == TRUE)
                    {
                        asTransaction[0].uFrame.sMsg.au8TransactionData[0] = WAITING_MODE;
                        afdeDataRequest(APS_ADDRMODE_SHORT,
                            sensordata.sensorinfo[i].sensorID,
                            WSN_DATA_SOURCE_ENDPOINT,
                            WSN_DATA_SINK_ENDPOINT,
                            WSN_PROFILE_ID,
                            WSN_CID_SENSOR_READINGS,
                            AF_MSG,
                            1,
                            asTransaction,
                            APS_TXOPTION_NONE,
                            SUPPRESS_ROUTE_DISCOVERY,
                            0);
                    }
                }
                break;
            case single:
                //vPrintf("single mode send\n");
                sensordata.sensorID = 0x0001;
                for(i = 0; i < 6; i++)
                {
                    if(sensordata.sensorID != sensordata.sensorinfo[i].sensorID)
                    {
                        if(sensordata.sensorinfo[i].isValiable == TRUE)
                        {
                            sExtAddr.u32L = sensordata.sensorinfo[i].macl;
                            sExtAddr.u32H = sensordata.sensorinfo[i].mach;

                            JZS_vRemoveNode(&sExtAddr, TRUE);
                            //bNwkRemoveDevice(u16AddrSrc);
                            vPrintf("sensor: %d remove success!x\n", sensordata.sensorinfo[i].sensorID);
                            sensordata.sensorinfo[i].isValiable = FALSE;
                        }
                    }
                }
                asTransaction[0].uFrame.sMsg.au8TransactionData[0] = CONTROL_CMD_GET_DATA_S;
                afdeDataRequest(APS_ADDRMODE_SHORT,
                        sensordata.sensorID,
                        WSN_DATA_SOURCE_ENDPOINT,
                        WSN_DATA_SINK_ENDPOINT,
                        WSN_PROFILE_ID,
                        WSN_CID_SENSOR_READINGS,
                        AF_MSG,
                        1,
                        asTransaction,
                        APS_TXOPTION_NONE,
                        SUPPRESS_ROUTE_DISCOVERY,
                        0);
                /*singledata.estate.num++;
                if(singledata.estate.num > 5)
                {
                    singledata.estate.num = 0;
                    singledata.estate.estate = E_FILL_COMPLETE;
                }else
                    singledata.estate.estate = E_FILL_UNCOMPLETE;
                singledata.tempdata[singledata.estate.num] = sTempHumiditySensor.u16TempReading;
                singledata.humidata[singledata.estate.num] = sTempHumiditySensor.u16HumidReading;
                if(singledata.tempdata[singledata.estate.num] > 100)
                {
                    singledata.tempdata[singledata.estate.num] = 100;
                    singledata.templength[singledata.estate.num] = 3;
                }else if(singledata.tempdata[singledata.estate.num] <= 0)
                {
                    singledata.tempdata[singledata.estate.num] = 0;
                    singledata.templength[singledata.estate.num] = 1;
                }else
                {
                    singledata.templength[singledata.estate.num] = 2;
                }

                if(singledata.humidata[singledata.estate.num] > 100)
                {
                    singledata.humidata[singledata.estate.num] = 100;
                    singledata.humilength[singledata.estate.num] = 3;
                }else if(singledata.humidata[singledata.estate.num] <= 0)
                {
                    singledata.humidata[singledata.estate.num] = 0;
                    singledata.humilength[singledata.estate.num] = 1;
                }else
                {
                    singledata.humilength[singledata.estate.num] = 2;
                }

                if(singledata.estate.estate == E_FILL_COMPLETE)
                    vPrintf("S%d,T[%d]%d:%d,H[%d]%d:%d,T[%d]%d:%d,H[%d]%d:%d,T[%d]%d:%d,H[%d]%d:%d,T[%d]%d:%d,H[%d]%d:%d,T[%d]%d:%d,H[%d]%d:%d,T[%d]%d:%d,H[%d]%d:%d\n",
                                singledata.sensorID, 0, singledata.templength[0], singledata.tempdata[0], 0, singledata.humilength[0], singledata.humidata[0],
                                1, singledata.templength[1], singledata.tempdata[1], 1, singledata.humilength[1], singledata.humidata[1],
                                2, singledata.templength[2], singledata.tempdata[2], 2, singledata.humilength[2], singledata.humidata[2],
                                3, singledata.templength[3], singledata.tempdata[3], 3, singledata.humilength[3], singledata.humidata[3],
                                4, singledata.templength[4], singledata.tempdata[4], 4, singledata.humilength[4], singledata.humidata[4],
                                5, singledata.templength[5], singledata.tempdata[5], 5, singledata.humilength[5], singledata.humidata[5]);
                    //vTxSerialDataFrame(1, sTempHumiditySensor.u16HumidReading,sTempHumiditySensor.u16TempReading);
                sTempHumiditySensor.eState = E_STATE_READ_TEMP_HUMID_IDLE;*/
                break;
            case mulit:
                for(i = 0; i < 6; i++)
                {
                    asTransaction[0].uFrame.sMsg.au8TransactionData[0] = CONTROL_CMD_GET_DATA_M;
                    if(sensordata.sensorinfo[i].isValiable == TRUE)
                        afdeDataRequest(APS_ADDRMODE_SHORT,
                        sensordata.sensorinfo[i].sensorID,
                        WSN_DATA_SOURCE_ENDPOINT,
                        WSN_DATA_SINK_ENDPOINT,
                        WSN_PROFILE_ID,
                        WSN_CID_SENSOR_READINGS,
                        AF_MSG,
                        1,
                        asTransaction,
                        APS_TXOPTION_NONE,
                        SUPPRESS_ROUTE_DISCOVERY,
                        0);
                    /*if(multidata[i].available == NOT_NULL)
                    {
                        multidata[i].tempdata = sTempHumiditySensor.u16TempReading;
                        multidata[i].humidata = sTempHumiditySensor.u16HumidReading;
                        vPrintf("S:%d,T:%d,H:%d\n",
                                multidata[i].sensorID, multidata[i].tempdata, multidata[i].humidata);
                    }*/
                }
                //sTempHumiditySensor.eState = E_STATE_READ_TEMP_HUMID_IDLE;
                break;
            case stop:
                //vPrintf("stopped!\n");
                break;
            default:
                    //vPrintf("wait for start!\n");
                break;
        }
    }
    if(sensordata.datatype == WAITING_MODE)
    {
        for(i = 0; i < 6; i++)
        {
            if(sensordata.sensorinfo[i].isValiable == TRUE)
                vPrintf("%d is avaliable x\n", sensordata.sensorinfo[i].sensorID);
        }
    }
    else if(sensordata.datatype == SINGLE_MODE)
    {
        if(fillcount == 6)
            fillcount = 0;

        vPrintf("S%dT%dL%d%dH%dL%d%dx\n", sensordata.data.sd.sensorID,
                                         fillcount, sensordata.data.sd.templength[fillcount], sensordata.data.sd.tempdata[fillcount],
                                         fillcount, sensordata.data.sd.humilength[fillcount], sensordata.data.sd.humidata[fillcount]);
        fillcount++;
    }
    else if(sensordata.datatype == MULTI_MODE)
    {
        for(i = 0; i < 6; i++)
        {
            if(sensordata.data.md[i].isValiable == TRUE)
                vPrintf("S%dT%dL%d%dH%dL%d%dx\n", sensordata.data.md[i].sensorID,
                                                 i, sensordata.data.md[i].templength, sensordata.data.md[i].tempdata,
                                                 i, sensordata.data.md[i].humilength, sensordata.data.md[i].humidata);
        }
    }
    else if(sensordata.datatype == UNSTARTED)
    {
        //vPrintf("wait for start\n");
    }

    (void)bBosCreateTimer(vToggleLed, &u8Msg, 0, (APP_TICK_PERIOD_ms / 10), &u8TimerId);
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

/****************************************************************************/
/***               Functions called by the stack                          ***/
/****************************************************************************/

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
void JZA_vAppEventHandler(void)
{
    uint8 u8Msg;
    uint8 u8TimerId;

    if (!bAppTimerStarted)
    {
        if (bNwkStarted)
        {
            bAppTimerStarted = TRUE;
            (void)bBosCreateTimer(vToggleLed, &u8Msg, 0, (APP_TICK_PERIOD_ms / 20), &u8TimerId);
        }
    }
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
    if(u32Device == E_AHI_DEVICE_UART0)
    {
        if((u32ItemBitmap & 0x000000ff) == E_AHI_UART_INT_RXDATA)
            armCmdRsv = ((u32ItemBitmap & 0x0000ff00) >> 8);
    }
    if(armCmdRsv==start||armCmdRsv==single||armCmdRsv==mulit||armCmdRsv==stop||armCmdRsv==reflash)
    {
        if(armCmd != armCmdRsv)
        {
            armCmd = armCmdRsv;
            commandChanged = TRUE;
            //vPrintf("command changed!\n");
        }
    }
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
 * NAME: JZA_boAppStart
 *
 * DESCRIPTION:
 * Called by Zigbee stack during initialisation. Sets up the profile
 * information and starts the networking activity
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
 * NAME: JZA_pu8AfMsgObject
 *
 * DESCRIPTION:
 * Called when a MSG transaction has been received with a matching endpoint.
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
	uint16 u16Humidity;
	uint16 u16BattVoltage;
	uint16 u16Temperature;
	uint32 mach, macl;
	static uint16 count, count1 = 0, count_s = 0;
	MAC_ExtAddr_s sExtAddr, *psExtAddr;
	uint16 command;
	int i;

    if ((eAddrMode == APS_ADDRMODE_SHORT) && (u8DstEP == WSN_DATA_SINK_ENDPOINT))
    {
        if(u8ClusterID == WSN_CID_SENSOR_READINGS)
        {
            command = puTransactionInd[0].uFrame.sMsg.au8TransactionData[0];
            if(command == CONTROL_CMD_GET_INFO)
            {
                //vPrintf("CONTROL_CMD_GET_INFO\n");
                count1++;
                vPrintf("count1: %dx\n", count1);
                if(count1 > 10 || getinfo_ok)
                {
                    vPrintf("get info okx\n");
                    getinfo_ok = TRUE;
                    count1 = 0;
                    return;
                }
                sensordata.datatype = WAITING_MODE;
                for(i = 0; i < 10; i++)
                {
                    if(sensordata.sensorinfo[i].sensorID == u16AddrSrc)
                        break;
                    sensordata.sensorinfo[i].sensorID = u16AddrSrc;
                    sensordata.sensorinfo[i].isValiable = TRUE;
                    sensordata.sensorinfo[i].mach = (puTransactionInd[0].uFrame.sMsg.au8TransactionData[1] & 0xff);
                    sensordata.sensorinfo[i].mach = sensordata.sensorinfo[i].mach << 24;
                    sensordata.sensorinfo[i].mach |= ((puTransactionInd[0].uFrame.sMsg.au8TransactionData[2] << 16) & 0xffff0000);
                    sensordata.sensorinfo[i].mach |= ((puTransactionInd[0].uFrame.sMsg.au8TransactionData[3] << 8) & 0xffffff00);
                    sensordata.sensorinfo[i].mach |= ((puTransactionInd[0].uFrame.sMsg.au8TransactionData[4]) & 0xffffffff);

                    sensordata.sensorinfo[i].macl = (puTransactionInd[0].uFrame.sMsg.au8TransactionData[5] & 0xff);
                    sensordata.sensorinfo[i].macl = sensordata.sensorinfo[i].macl << 24;
                    sensordata.sensorinfo[i].macl |= ((puTransactionInd[0].uFrame.sMsg.au8TransactionData[6] << 16) & 0xffff0000);
                    sensordata.sensorinfo[i].macl |= ((puTransactionInd[0].uFrame.sMsg.au8TransactionData[7] << 8) & 0xffffff00);
                    sensordata.sensorinfo[i].macl |= ((puTransactionInd[0].uFrame.sMsg.au8TransactionData[8]) & 0xffffffff);
                    break;
                }
            }
            else if(command == CONTROL_CMD_GET_DATA_S)
            {
                sensordata.datatype = SINGLE_MODE;
                //vPrintf("CONTROL_CMD_GET_DATA_S");
                if(count_s == 6)
                    count_s = 0;
                sensordata.datatype = SINGLE_MODE;
                sensordata.data.sd.sensorID = u16AddrSrc;
                sensordata.data.sd.tempdata[count_s] = puTransactionInd[0].uFrame.sMsg.au8TransactionData[4];
                sensordata.data.sd.tempdata[count_s] = sensordata.data.sd.tempdata[count_s]<<8;
                sensordata.data.sd.tempdata[count_s] |= puTransactionInd[0].uFrame.sMsg.au8TransactionData[3];
                if(sensordata.data.sd.tempdata[count_s] > 99)
                    sensordata.data.sd.templength[count_s] = 3;
                else if(sensordata.data.sd.tempdata[count_s] <10)
                    sensordata.data.sd.templength[count_s] = 1;
                else
                    sensordata.data.sd.templength[count_s] = 2;

                sensordata.data.sd.humidata[count_s] = puTransactionInd[0].uFrame.sMsg.au8TransactionData[6];
                sensordata.data.sd.humidata[count_s] = sensordata.data.sd.humidata[count_s]<<8;
                sensordata.data.sd.humidata[count_s] |= puTransactionInd[0].uFrame.sMsg.au8TransactionData[5];
                if(sensordata.data.sd.humidata[count_s] > 99)
                    sensordata.data.sd.humilength[count_s] = 3;
                else if(sensordata.data.sd.humidata[count_s] <10)
                    sensordata.data.sd.humilength[count_s] = 1;
                else
                    sensordata.data.sd.humilength[count_s] = 2;

                count_s++;
            }
            else if(command == CONTROL_CMD_GET_DATA_M)
            {
                //vPrintf("CONTROL_CMD_GET_DATA_M");
                sensordata.datatype = MULTI_MODE;
                for(i = 0; i < 6; i++)
                {
                    if(u16AddrSrc == sensordata.sensorinfo[i].sensorID)
                    {
                        sensordata.data.md[i].sensorID = u16AddrSrc;
                        sensordata.data.md[i].isValiable = TRUE;
                        sensordata.data.md[i].tempdata = puTransactionInd[0].uFrame.sMsg.au8TransactionData[4];
                        sensordata.data.md[i].tempdata = sensordata.data.md[i].tempdata<<8;
                        sensordata.data.md[i].tempdata |= puTransactionInd[0].uFrame.sMsg.au8TransactionData[3];
                        if(sensordata.data.md[i].tempdata > 99)
                            sensordata.data.md[i].templength = 3;
                        else if(sensordata.data.md[i].tempdata <10)
                            sensordata.data.md[i].templength = 1;
                        else
                            sensordata.data.md[i].templength = 2;

                        sensordata.data.md[i].humidata = puTransactionInd[0].uFrame.sMsg.au8TransactionData[6];
                        sensordata.data.md[i].humidata = sensordata.data.md[i].tempdata<<8;
                        sensordata.data.md[i].humidata |= puTransactionInd[0].uFrame.sMsg.au8TransactionData[5];
                        if(sensordata.data.md[i].humidata > 99)
                            sensordata.data.md[i].humilength = 3;
                        else if(sensordata.data.md[i].humidata <10)
                            sensordata.data.md[i].humilength = 1;
                        else
                            sensordata.data.md[i].humilength = 2;
                    }
                }
            }
            /*count++;
            u16BattVoltage  = puTransactionInd[0].uFrame.sMsg.au8TransactionData[2];
            u16BattVoltage  = u16BattVoltage << 8;
            u16BattVoltage |= puTransactionInd[0].uFrame.sMsg.au8TransactionData[1];

            u16Temperature  = puTransactionInd[0].uFrame.sMsg.au8TransactionData[4];
            u16Temperature  = u16Temperature << 8;
            u16Temperature |= puTransactionInd[0].uFrame.sMsg.au8TransactionData[3];

            u16Humidity  = puTransactionInd[0].uFrame.sMsg.au8TransactionData[6];
            u16Humidity  = u16Humidity << 8;
            u16Humidity |= puTransactionInd[0].uFrame.sMsg.au8TransactionData[5];

            for(i = 6; i < 14; i++)
                vPrintf("%x ", puTransactionInd[0].uFrame.sMsg.au8TransactionData[i]);
            mach = (puTransactionInd[0].uFrame.sMsg.au8TransactionData[7] & 0xff);
            mach = mach << 24;
            mach |= ((puTransactionInd[0].uFrame.sMsg.au8TransactionData[8] << 16) & 0xffff0000);
            mach |= ((puTransactionInd[0].uFrame.sMsg.au8TransactionData[9] << 8) & 0xffffff00);
            mach |= ((puTransactionInd[0].uFrame.sMsg.au8TransactionData[10]) & 0xffffffff);

            macl = (puTransactionInd[0].uFrame.sMsg.au8TransactionData[11] & 0xff);
            macl = macl << 24;
            macl |= ((puTransactionInd[0].uFrame.sMsg.au8TransactionData[12] << 16) & 0xffff0000);
            macl |= ((puTransactionInd[0].uFrame.sMsg.au8TransactionData[13] << 8) & 0xffffff00);
            macl |= ((puTransactionInd[0].uFrame.sMsg.au8TransactionData[14]) & 0xffffffff);

            sExtAddr.u32L = macl;
            sExtAddr.u32H = mach;

            if(count > 10)
            {
                count = 0;
                JZS_vRemoveNode(&sExtAddr, TRUE);
                //bNwkRemoveDevice(u16AddrSrc);
                vPrintf("remove success!\n");
            }
            vPrintf("%d\n", puTransactionInd[0].uFrame.sMsg.au8TransactionData[0]);//
            vPrintf("%d\n", puTransactionInd[1].uFrame.sMsg.au8TransactionData[0]);
            vTxSerialDataFrame(count, u16Humidity, u16Temperature, mach, macl);*/
        }
    }
    return 0;
}

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
    if (eEventId == JZS_EVENT_NWK_STARTED)
    {

		// load the simple descriptor now that the network has started
		uint8 u8InputClusterCnt      = 1;
		uint8 au8InputClusterList[]  = {WSN_CID_SENSOR_READINGS};
		uint8 u8OutputClusterCnt     = 1;
		uint8 au8OutputClusterList[] = {WSN_CID_SENSOR_READINGS};

		(void)afmeAddSimpleDesc(WSN_DATA_SINK_ENDPOINT,
								WSN_PROFILE_ID,
								0x0000,
								0x00,
								0x00,
								u8InputClusterCnt,
								au8InputClusterList,
								u8OutputClusterCnt,
								au8OutputClusterList);

        bNwkStarted = TRUE;
    }
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
