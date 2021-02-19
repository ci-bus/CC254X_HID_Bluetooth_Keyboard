/**************************************************************************************************
Filename:       hidKbdMouse.c

Description:    This file contains the HID emulated keyboard sample application
for use with the CC2540 Bluetooth Low Energy Protocol Stack.

Author:         CONG NGUYEN
Last modified:  8/10/2014
**************************************************************************************************/

/*********************************************************************
* INCLUDES
*/

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "hidkbmservice.h"
#include "devinfoservice.h"
#include "hiddev.h"

#include "osal_snv.h"

#include "hidKbdMouse.h"
#include <string.h>
#include "KBD_Report.h"
#include "KBD_Batt.h"

/*********************************************************************
* MACROS
*/

//version string to identify module/capabilities
#define VERSION_STRING "hidkbdmousev2\r\n"

// UART
#define KEEP_CONNECTION_ALIVE_50s       50000

// Selected HID LED bitmaps
#define LED_NUM_LOCK                8
#define LED_CAPS_LOCK               9
#define LED_SCROLL_LOCK             10

// Selected HID mouse button values
#define MOUSE_BUTTON_1              0x01
#define MOUSE_BUTTON_NONE           0x00

// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN     8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN         2

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN        5

// HID consumer input report length
#define HID_CONSUMER_IN_RPT_LEN     3

/*********************************************************************
* CONSTANTS
*/

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT              60000

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         50

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         10

// Default passcode
#define DEFAULT_PASSCODE                      0

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

//#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE
//#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_KEYBOARD_DISPLAY

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6

#define SNV_ID_DEVICE_NAME              0x80
#define SNV_ID_DEVICE_NAME_LENGTH       0x81
#define SNV_ID_DEVICE_NAME_CRC          0x82

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/

// Task ID
uint8 hidKbdMouseTaskId;
// Notificate key actions
static uint8 notificateActionKeys = 0;

//UART test variable
static uint8 len, i, rxBufferIndex = 0;
static uint8 buffer[16], rxBuffer[24];

/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/

/*********************************************************************
* LOCAL VARIABLES
*/

halUARTCfg_t uartConfig;

uint8 *device_name_crc;
uint8 *device_name;
uint8 *device_name_length;

// GAP Profile - Name attribute for SCAN RSP data - Name shown up when scanned
static uint8 default_scanData[] =
{
  0x0d,                             // length of this data => (name's size = 20 bytes) + 1
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,   // AD Type = Complete local name
  'H',
  'I',
  'D',
  ' ',
  'K',
  'e',
  'y',
  'b',
  'o',
  'a',
  'r',
  'd'
};

// Advertising data
static uint8 advData[] =
{
  // flags
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // appearance
  0x03,   // length of this data
  GAP_ADTYPE_APPEARANCE,
  LO_UINT16(GAP_APPEARE_HID_KEYBOARD),
  HI_UINT16(GAP_APPEARE_HID_MOUSE),

  // service UUIDs
  0x05,   // length of this data
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(HID_SERV_UUID),
  HI_UINT16(HID_SERV_UUID),
  LO_UINT16(BATT_SERV_UUID),
  HI_UINT16(BATT_SERV_UUID)
};

static CONST uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "HID Keyboard";

// HID Dev configuration
static hidDevCfg_t hidKbdMouseCfg =
{
  DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
  HID_KBD_FLAGS               // HID feature flags
};

/*********************************************************************
* LOCAL FUNCTIONS
*/

/* after calling sleepMode(), the host MCU must wake
HM-xx before sending characters */
static void sleepMode(void);
static void activeMode(void);

static void hidKbdMouseSendReport( uint8 modifier, uint8 keycode );
static void hidKbdMouseSendMouseReport(uint8 buttons, int8 dx, int8 dy, int8 dz);
static void hidKbdMouseSendConsumerReport(uint8 first, uint8 second);
static uint8 hidKbdMouseRcvReport( uint8 len, uint8 *pData );
static uint8 hidKbdMouseRptCB( uint8 id, uint8 type, uint16 uuid,
                              uint8 oper, uint8 *pLen, uint8 *pData );
static void hidKbdMouseEvtCB( uint8 evt );

// UART functions
static void setupUART(void);
static void uartCallback(uint8 port, uint8 event);
static void performPeriodicTask(void);

//Keyboard functions
static void processCommands(void);


//UART send string
static void uartSend(uint8 *str) {
    #if (HAL_UART_ISR == 1)
      HalUARTWrite(HAL_UART_PORT_0, str, strlen((const char*)str));
    #else
      HalUARTWrite(HAL_UART_PORT_1, str, strlen((const char*)str));
    #endif
}
//UART send data like binary
/*
static void uartSendLikeBinary(uint8 *pData) {
    uartSend("\r\n");
    for (int i = 0; i < sizeof(pData)*8; i ++) {
      if ((pData[i / 8] >> (i % 8)) & 1) {
        uartSend("0");
      } else {
        uartSend("1");
      }
    }
    uartSend("\r\n");
}
*/

//Pololu's CRC functions with minimal changes
uint8 CRCPoly = 0x89;  // the value of our CRC-7 polynomial
uint8 CRCTable[256];

void GenerateCRCTable()
{
  int i, j;

  // generate a table value for all 256 possible byte values
  for (i = 0; i < 256; i++)
  {
    CRCTable[i] = (i & 0x80) ? i ^ CRCPoly : i;
    for (j = 1; j < 8; j++)
    {
      CRCTable[i] <<= 1;
      if (CRCTable[i] & 0x80)
        CRCTable[i] ^= CRCPoly;
    }
  }
}


// adds a message byte to the current CRC-7 to get a the new CRC-7
uint8 CRCAdd(uint8 CRC, uint8 message_byte)
{
  return CRCTable[(CRC << 1) ^ message_byte];
}


// returns the CRC-7 for a message of "length" bytes
uint8 getCRC(uint8 message[], uint8 length)
{
  uint8 i;
  uint8 CRC = 0;

  for (i = 0; i < length; i++)
    CRC = CRCAdd(CRC, message[i]);

  return CRC;
}

/*********************************************************************
* PROFILE CALLBACKS
*/

static hidDevCB_t hidKbdMouseHidCBs =
{
  hidKbdMouseRptCB,
  hidKbdMouseEvtCB,
  NULL
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      HidKbdMouse_Init
*
* @brief   Initialization function for the HidKbdMouse App Task.
*          This is called during initialization and should contain
*          any application specific initialization (ie. hardware
*          initialization/setup, table initialization, power up
*          notificaiton ... ).
*
* @param   task_id - the ID assigned by OSAL.  This ID should be
*                    used to send messages and set timers.
*
* @return  none
*/
void HidKbdMouse_Init( uint8 task_id )
{
  setupUART();
  GenerateCRCTable();
  device_name = osal_mem_alloc(20);
  device_name_length = osal_mem_alloc(1);
  device_name_crc = osal_mem_alloc(1);
  HalLedInit();

  hidKbdMouseTaskId = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = TRUE; //previously FALSE

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advData ), advData );

    //name change - 21/12/2014
    osal_snv_read(SNV_ID_DEVICE_NAME_CRC, 1, device_name_crc);
    osal_snv_read(SNV_ID_DEVICE_NAME_LENGTH, 1, device_name_length);
    osal_snv_read(SNV_ID_DEVICE_NAME, 20, device_name);

    if(*device_name_crc != getCRC(device_name, *device_name_length)) {
      uartSend("Using default scan response name\r\n");
      GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( default_scanData ), default_scanData );
    } else {
      //make changes directly to the default_scanData. Since this variable is set at start-up, it should not matter
      uint8 len = *device_name_length;
      //uint8 default_name_length = default_scanData[0];
      default_scanData[0] = len + 1;
      uint8 i;
      for(i = 0; i < len; i++) {
        default_scanData[i+2] = device_name[i];
      }
      GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( default_scanData ), default_scanData );
    }

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  if(*device_name_crc != getCRC(device_name, *device_name_length)) {
    //    printf("Using default device name");
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (void *) attDeviceName );
  } else {
    //    printf("Using stored device name");
    //    printf("%s\r\n", device_name);
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, *device_name_length + 1, (void *) device_name );
  }
  //Allow device to change name
  uint8 devNamePermission = GATT_PERMIT_READ|GATT_PERMIT_WRITE;
  GGS_SetParameter( GGS_W_PERMIT_DEVICE_NAME_ATT, sizeof ( uint8 ), &devNamePermission );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }

  // Setup Battery Characteristic Values
  {
    uint8 critical = DEFAULT_BATT_CRITICAL_LEVEL;
    Batt_SetParameter( BATT_PARAM_CRITICAL_LEVEL, sizeof (uint8), &critical );
  }

  // Set up HID keyboard service
  HidKbM_AddService( );

  // Register for HID Dev callback
  HidDev_Register( &hidKbdMouseCfg, &hidKbdMouseHidCBs );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( hidKbdMouseTaskId );

#if defined( CC2540_MINIDK )
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2 | HAL_LED_3), HAL_LED_MODE_OFF );

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
  // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0xFF;   // All pins on port 1 to high
  P2 = 0;   // All pins on port 2 to low

#endif // #if defined( CC2540_MINIDK )
  
  // Turn on an LED
  //init keyboard report manager
  KBD_Report_Init();

  // Setup a delayed profile startup
  osal_set_event( hidKbdMouseTaskId, START_DEVICE_EVT );
}

/*********************************************************************
* @fn      HidKbdMouse_ProcessEvent
*
* @brief   HidKbdMouse Application Task event processor.  This function
*          include timers, messages and any other user defined events.
*          is called to process all events for the task.  Events
*
* @param   task_id  - The OSAL assigned task ID.
* @param   events - events to process.  This is a bit map and can
*                   contain more than one event.
*
* @return  events not processed
*/
uint16 HidKbdMouse_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( hidKbdMouseTaskId )) != NULL )
    {
      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Set timer for first periodic event
    osal_start_timerEx( hidKbdMouseTaskId, SBP_PERIODIC_EVT, KEEP_CONNECTION_ALIVE_50s );

    return ( events ^ START_DEVICE_EVT );
  }

  //Added to test UART
  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( KEEP_CONNECTION_ALIVE_50s )
    {
      osal_start_timerEx( hidKbdMouseTaskId, SBP_PERIODIC_EVT, KEEP_CONNECTION_ALIVE_50s );
    }

    // Perform periodic application task
    performPeriodicTask();

    return (events ^ SBP_PERIODIC_EVT);
  }

  return 0;
}

/*********************************************************************
* @fn      hidKbdMouseSendReport
*
* @brief   Build and send a HID keyboard report.
*
* @param   keycode - HID keycode.
*
* @return  none
*/
static void hidKbdMouseSendReport( uint8 modifier, uint8 keycode )
{
  uint8 buf[HID_KEYBOARD_IN_RPT_LEN];

  buf[0] = modifier;  // Modifier keys
  buf[1] = 0;         // Reserved
  buf[2] = keycode;   // Keycode 1
  buf[3] = 0;         // Keycode 2
  buf[4] = 0;         // Keycode 3
  buf[5] = 0;         // Keycode 4
  buf[6] = 0;         // Keycode 5
  buf[7] = 0;         // Keycode 6

  HidDev_Report( HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT,
                HID_KEYBOARD_IN_RPT_LEN, buf );
}

/*********************************************************************
* @fn      hidKbdMouseSendMouseReport
*
* @brief   Build and send a HID mouse report.
*
* @param   buttons - Mouse button code
*
* @return  none
*/
static void hidKbdMouseSendMouseReport(uint8 buttons, int8 dx, int8 dy, int8 dz)
{
  uint8 buf[HID_MOUSE_IN_RPT_LEN];

  buf[0] = buttons;   // Buttons
  buf[1] = dx;         // X
  buf[2] = dy;         // Y
  buf[3] = dz;         // Wheel
  buf[4] = 0;         // AC Pan

  HidDev_Report( HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                HID_MOUSE_IN_RPT_LEN, buf );
}

/*********************************************************************
* @fn      hidKbdMouseSendConsumerReport
*
* @brief   Build and send a HID consumer report.
*
* @param   Action code in two 8 bit numbers
*
* @return  none
*/
static void hidKbdMouseSendConsumerReport(uint8 byteLow, uint8 byteHigh)
{
  uint8 buf[HID_CONSUMER_IN_RPT_LEN];

  buf[0] = byteLow;   // low 8 bit code
  buf[1] = byteHigh;  // high 8 bit number
  buf[2] = 0;         // AC Pan

  HidDev_Report( HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT,
                HID_CONSUMER_IN_RPT_LEN, buf );
}

/*********************************************************************
* @fn      hidKbdMouseRcvReport
*
* @brief   Process an incoming HID keyboard report.
*
* @param   len - Length of report.
* @param   pData - Report data.
*
* @return  status
*/
static uint8 hidKbdMouseRcvReport( uint8 len, uint8 *pData )
{
  
  // uartSendLikeBinary(pData);
  
  // LEDS locked keys
  if(len == HID_LED_OUT_RPT_LEN) {
        
    if ((pData[LED_CAPS_LOCK / 8] >> (LED_CAPS_LOCK % 8)) & 1) {
      HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
    } else {
      HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
    }
    
    if ((pData[LED_NUM_LOCK / 8] >> (LED_NUM_LOCK % 8)) & 1) {
        HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
    } else {
        HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF);
    }

    if ((pData[LED_SCROLL_LOCK / 8] >> (LED_SCROLL_LOCK % 8)) & 1) {
        HalLedSet(HAL_LED_3, HAL_LED_MODE_ON);
    } else {
        HalLedSet(HAL_LED_3, HAL_LED_MODE_OFF);
    }

    return SUCCESS;
  } else {
    return ATT_ERR_INVALID_VALUE_SIZE;
  }
}

/*********************************************************************
* @fn      hidKbdMouseRptCB
*
* @brief   HID Dev report callback.
*
* @param   id - HID report ID.
* @param   type - HID report type.
* @param   uuid - attribute uuid.
* @param   oper - operation:  read, write, etc.
* @param   len - Length of report.
* @param   pData - Report data.
*
* @return  GATT status code.
*/
static uint8 hidKbdMouseRptCB( uint8 id, uint8 type, uint16 uuid,
                              uint8 oper, uint8 *pLen, uint8 *pData )
{
  uint8 status = SUCCESS;

  // write
  if ( oper == HID_DEV_OPER_WRITE )
  {
    if ( uuid == REPORT_UUID )
    {
      // process write to LED output report; ignore others
      if ( type == HID_REPORT_TYPE_OUTPUT )
      {
        status = hidKbdMouseRcvReport( *pLen, pData );
      }
    }

    if ( status == SUCCESS )
    {
      status = HidKbM_SetParameter( id, type, uuid, *pLen, pData );
    }
  }
  // read
  else if ( oper == HID_DEV_OPER_READ )
  {
    status = HidKbM_GetParameter( id, type, uuid, pLen, pData );
  }

  return status;
}

/*********************************************************************
* @fn      hidKbdMouseEvtCB
*
* @brief   HID Dev event callback.
*
* @param   evt - event ID.
*
* @return  HID response code.
*/
static void hidKbdMouseEvtCB( uint8 evt )
{
  // process enter/exit suspend or enter/exit boot mode

  return;
}

/*
Setting up UART
- To use UART, HAL_UART=TRUE, preferrably POWER_SAVING is not enabled
- To use interrupts, HAL_UART_ISR = (1 or 2), HAL_UART_DMA=FALSE
- To use DMA, HAL_UART_ISR = 0, HAL_UART_DMA = (1 or 2)

+ HAL_UART_ISR = 1: Use USART 0
+ HAL_UART_ISR = 2: Use USART 1

Set PERCFG.UxCFG to choose alternative 1 or 2

For keyfob, USART 0 alt. 1 is being used:
- HAL_UART_ISR = 1
For HM-10, USART 1 alt. 2 is being used:
- HAL_UART_ISR = 2
For HM-11, RX on P0.2, TX on P0.3 => USART 0 alt. 1 is being used:
- HAL_UART_ISR = 1
*/

static void setupUART(void) {
  HalUARTInit();

  // configure UART
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = HAL_UART_BR_57600;
  uartConfig.flowControl          = HAL_UART_FLOW_OFF;
  uartConfig.intEnable            = TRUE;
  uartConfig.callBackFunc         = (halUARTCBack_t)uartCallback;

  //start UART
  //assumes no issues with starting UART
  #if (HAL_UART_ISR == 1)
    (void)HalUARTOpen(HAL_UART_PORT_0, &uartConfig);
  #else
    (void)HalUARTOpen(HAL_UART_PORT_1, &uartConfig);
  #endif
  
}

static void uartCallback(uint8 port, uint8 event) {

  switch(event) {
  case HAL_UART_RX_FULL:
  case HAL_UART_RX_ABOUT_FULL:
  case HAL_UART_RX_TIMEOUT:

      len = Hal_UART_RxBufLen(port);
      HalUARTRead(port, buffer, len);
            
      /*
        Receive Data, make buffer and process
      */

      for(i = 0; i < len; i++) 
      {
        if(buffer[i] != '\n') {
            rxBuffer[rxBufferIndex] = buffer[i];
            rxBufferIndex++;
        } else {
            rxBuffer[rxBufferIndex] = 0;
            processCommands();
            rxBufferIndex = 0;
            uartSend("1");
        }
      }
    break;
  }
}

//Buffer being processed stored in rxBuffer, does not include CRLF
/*
Command sets, chosen options need to be stored in non-volatile memory
- SN,<name>  + set device name
- S,R Reset the device
- S,ID print ID of module
- S,N print the current Bluetooth name
- S,D Set device to be discoverable
- S,DT  Disconnect device from host
- S,S Sleed mode disable
- S,A Sleed mode enable
- S,F Notificate key actions and others
- BL<porcent> Level of battery
- KP<keyCode> Press a key
- KR<keyCode> Release a key
- KPR<keyCode> Press ans release a key
- CP<lowByte><highByte> Press consumer key
- CR<lowByte><highByte> Release consumer key
- CPR<lowByte><highByte> Press and release consumer key
*/

static void processCommands() {
      
  // Keyboard commands
  if(rxBuffer[0] == 'K') {
      if (rxBuffer[1] == 'P' && rxBuffer[2] == 'R') {
         hidKbdMouseSendReport(0, rxBuffer[3]);
         hidKbdMouseSendReport(0, 0);
         if(notificateActionKeys) {
           uartSend("Key clicked\r\n");
         }
         
      } else if(rxBuffer[1] == 'P') {
        //Key pressed
        KBD_Report_AddKey(rxBuffer[2]);
        KBD_Report_Update();
        if(notificateActionKeys) {
          uartSend("key pressed\r\n");
        }
      } else if(rxBuffer[1] == 'R') {
        //Key released
        KBD_Report_RemoveKey(rxBuffer[2]);
        KBD_Report_Update();
        if(notificateActionKeys) {
          uartSend("key released\r\n");
        }
        
      }
    
  // Mouse commands
  } else if(rxBuffer[0] == 'M') {
    hidKbdMouseSendMouseReport(rxBuffer[1], rxBuffer[2], rxBuffer[3], rxBuffer[4]);
    if(notificateActionKeys) {
      uartSend("Mouse commands\r\n");
    }
        
  // Consumer keys
  } else if (rxBuffer[0] == 'C') {
    if (rxBuffer[1] == 'P' && rxBuffer[2] == 'R') {
         hidKbdMouseSendConsumerReport(rxBuffer[3], rxBuffer[4]);
         hidKbdMouseSendConsumerReport(0, 0);
         if(notificateActionKeys) {
           uartSend("Consumer key clicked\r\n");
         }
         
      } else if(rxBuffer[1] == 'P') {
        //Consumer key pressed
        hidKbdMouseSendConsumerReport(rxBuffer[2], rxBuffer[3]);
        if(notificateActionKeys) {
          uartSend("Consumer key pressed\r\n");
        }
      } else if(rxBuffer[1] == 'R') {
        //Consumer key released
        hidKbdMouseSendConsumerReport(0, 0);
        if(notificateActionKeys) {
          uartSend("Consumer key released\r\n");
        }
        
      }
    
  // Battery level
  } else if (rxBuffer[0] == 'B' && rxBuffer[1] == 'L') {
    batteryNotifyLevel(rxBuffer[2]);
    
  // Setting commands
  } else if(rxBuffer[0] == 'S') {
    if((rxBuffer[1] == 'N') && (rxBuffer[2] == ',')) {
      uint8 i;
      uint8 deviceNewName[20];
      uint8 deviceNewNameLength;
      uint8 deviceNewNameCRC;

      deviceNewNameLength = rxBufferIndex-3;
      if(deviceNewNameLength > 20) {
        uartSend("Name exceeds permitted length of 20 chars\r\n");
      } else {
        for(i = 3; i < rxBufferIndex; i++) {
          deviceNewName[i-3] = rxBuffer[i];
        }
        deviceNewName[deviceNewNameLength] = '\0';
        deviceNewNameCRC = getCRC(deviceNewName, deviceNewNameLength);

        osal_snv_write(SNV_ID_DEVICE_NAME, 20, deviceNewName);
        osal_snv_write(SNV_ID_DEVICE_NAME_LENGTH, 1, &deviceNewNameLength);
        osal_snv_write(SNV_ID_DEVICE_NAME_CRC, 1, &deviceNewNameCRC);
        uartSend("Name is being set, reset to set new name\r\n");
      }
    } else if((rxBuffer[1] == ',') && (rxBuffer[2] == 'R')) {
      //reset the device
      uartSend("reset\r\n");
      HAL_SYSTEM_RESET();
    } else if((rxBuffer[1] == ',') && (rxBuffer[2] == 'I') && (rxBuffer[3] == 'D')) {
      //print ID
      uartSend(VERSION_STRING);
    } else if((rxBuffer[1] == ',') && (rxBuffer[2] == 'N')) {
      //print Bluetooth name
      uartSend(device_name);
    } else if((rxBuffer[1] == ',') && (rxBuffer[2] == 'D')) {
      if(rxBuffer[3] == 'C') {
        //disconnect the device from host
        uartSend("Disconnecting from host...\r\n");
        GAPRole_TerminateConnection();
      } else {
        //set device to be discoverable
        uartSend("Set deveice to be discoverable\r\n");
      }
    } else if((rxBuffer[1] == ',') && (rxBuffer[2] == 'S')) {
      //enable sleep mode
      uartSend("SLEEP\r\n");
      sleepMode();
    } else if((rxBuffer[1] == ',') && (rxBuffer[2] == 'A')) {
      //disable sleep mode
      uartSend("ACTIVE\r\n");
      activeMode();
    } else if((rxBuffer[1] == ',') && (rxBuffer[2] == 'F')) {
      // Notificate key pressed and released
      if (notificateActionKeys) {
        notificateActionKeys = 0;
        uartSend("Key actions notification disabled\r\n");
      } else {
        notificateActionKeys = 1;
        uartSend("Key actions notification enabled\r\n");
      }
    }
  }
}

static void performPeriodicTask(void) {
  //send a blank keyboard report to keep connection connected
  hidKbdMouseSendReport(0,0);
  if(notificateActionKeys) {
    uartSend("Keep Alive\r\n");
  }
}

static void sleepMode(void) {
  //Configure RX pin as input with active high interrupt
  // - HM-10/HM-11: RX = P1.7
#if (HAL_UART_ISR == 1)
  P0SEL &= ~(1 << 2); //deselect as peripheral pin, just to be sure
  P0DIR &= ~(1 << 2); //input
  P0INP &= ~(1 << 2); //enable pull-up or pull-down
  P2INP &= ~(1 << 5); //select pull-up for all port 0 pins - active low
  PICTL |= (1 << 0);  //Falling edge on input gives interrupt (P0ICON)
  P0IFG &= ~(1 << 2); //Clear existing interrupt on P0.2
  P0IEN |= ~(1 << 2); //Enable interrupt on P0.2
#else
  //HM-10
  P1SEL &= ~(1 << 7); //deselect as peripheral pin, just to be sure
  P1DIR &= ~(1 << 7); //input
  P1INP &= ~(1 << 7); //enable pull-up or pull-down
  P2INP &= ~(1 << 6); //select pull-up for all port 1 pins - active low
  PICTL |= (1 << 2);  //Falling edge on input gives interrupt (P1ICONH)
  P0IFG &= ~(1 << 7); //Clear existing interrupt on P0.2
  P0IEN |= ~(1 << 7); //Enable interrupt on P0.2
#endif
  osal_pwrmgr_device( PWRMGR_BATTERY );
}

static void activeMode(void) {
  // Disable pin interrupts and re-enable UART
  // - HM-10: RX = P1.7
#if (HAL_UART_ISR == 1)
  P0SEL |= (1 << 2); //select as peripheral pin
  (void)HalUARTOpen(HAL_UART_PORT_0, &uartConfig);
#else
  //HM-10
  P1SEL |= (1 << 7); //select as peripheral pin
  (void)HalUARTOpen(HAL_UART_PORT_1, &uartConfig);
#endif
  osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
}
/*********************************************************************
ISR
*********************************************************************/
#pragma vector=P0INT_VECTOR
__interrupt void P0_ISR(void) {
  //Disable pin interrupts and re-enable UART

  HAL_ENTER_ISR();
  //clear interrupt
  P0IFG = 0;
  P0IF = 0;

  uint8 pV = P0_2;
  if(pV == 0) {
    activeMode();
  }
  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}

#pragma vector=P1INT_VECTOR
__interrupt void P1_ISR(void) {
  //HM-10 - Disable pin interrupts and re-enable UART

  HAL_ENTER_ISR();
  P1IFG = 0;
  P1IF = 0;

  uint8 pV = P1_7;
  if(pV) {
    //active low
    activeMode();
  }
  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}