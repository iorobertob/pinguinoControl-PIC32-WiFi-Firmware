
    /*-----------------------------------------------------
   MainDemo.c
     * Version: 2.0

    Author:  I/O Roberto Becerra--<http://iobridger.wordpress.com/>
    Date: Fr March 28 21:48:44 2014
    
    Description:
     
    Source file part of the communication protocol used to
    control a Pinguino Board with a PIC32MX440F256H embedded in it,  using a
    WiFi network, and a MOD WIFI module plugged to it. This MOD WIFI, as sold
    by Olimex, contains a WiFi chip by Microchip called MRF24WB0MA.
        By implementing the communication protocol in an Android Device, 
    communication via UDP is stablished, where the PIC communicates 
    its IP Address, to later receive UDP packets with control messages encoded 
    according to the protocol than can be found in the .xls file annexed in the 
    folder where this source code is located. 
        Thus, the Pinguino PIC32 board can be controlled in real time retrieving
    the states of Analog and Digital Pins, as well as setting them.
    This source file is optimized to be used with a Pinguino board,
    the PINGUINO PIC32 OTG. It nevertheless contains and implements
    definitions and functions that work with the PIC32MX440F256H.
    
        Modify file WF_Config.h to set the network SSID, Type and Security Mode, 
     among other parameters that affect how MOD WIFI connects to the local server. 
     
     *
     * ChangeLog:
     * 32/August/2013 - Added state change for RD0 together with RD4 since they are
     * Roberto          parallel in the PCB
     *                - Deleted random code rubbish in the Analog part of Pin Modes
     *                  and reordered the instructions because it was not reacting
     *                  until second analog input was activated
     *                - Got rid of delays in the CDC sending section.
     *                - Set the reports of states to be only once every 20 times
     *                  to avoid saturating the receiving system. 
     * 
     * 28/March/2014  - Modified code to simplify and clean it. Optimize it to be less
     * Roberto          application specific, with more reusable functions on a
     * v2.0             more abstract manner, such as UDP functions. 
     *                - Added functions to report/broadcast this chip's IP address
     *                  as well as generic bytes and byte arrays, over UDP ports. 
     *                - Added generic interpretations for functions that receive
     *                  bytes and byte arrays over UDP ports. 
     *                - Merged the previous version which used CDC communication 
     *                  and a PureData patch ruuning in a PC, with the new code 
     *                  containing WiFi ready functions. 
     *                - General restructuring of file to comply with the new 
     *                  purpose of wireless communication. 
     * 
     * 
     *    Code based on MainDemo.c provided by Microchip as part of its 
     *  Solutions Library. It is to work with Microchip's TCPIP Stack. 
     *  Some of the following information refers to this origin. 

    -----------------------------------------------------*/
/*********************************************************************
 *

 *
 *
 *
 *********************************************************************
 * FileName:        MainDemo.c
 * Dependencies:    TCPIP.h
 * Processor:       PIC32
 * Compiler:        Microchip C32 v1.11b or higher
 *                  Microchip C30 v3.24 or higher
 *                  Microchip C18 v3.36 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2010 Microchip Technology Inc.  All rights
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and
 * distribute:
 * (i)  the Software when embedded on a Microchip microcontroller or
 *      digital signal controller product ("Device") which is
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
 *      ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device
 *      used in conjunction with a Microchip ethernet controller for
 *      the sole purpose of interfacing with the ethernet controller.
 *
 * You should refer to the license agreement accompanying this
 * Software for additional information regarding your rights and
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 * File Description:
 * Change History:
 * Rev   Description
 * ----  -----------------------------------------
 * 1.0   Initial release
 * V5.36 ---- STACK_USE_MPFS support has been removed
 ********************************************************************/
/*
 * This macro uniquely defines this file as the main entry point.
 * There should only be one such definition in the entire project,
 * and this file must define the AppConfig variable as described below.
 */
#define THIS_IS_STACK_APPLICATION

#include <math.h>

// Include all headers for any enabled TCPIP Stack functions
#include "TCPIP Stack/TCPIP.h"

// Include functions specific to this stack application
#include "MainDemo.h"

// Used for Wi-Fi assertions
#define WF_MODULE_NUMBER   WF_MODULE_MAIN_DEMO

// Declare AppConfig structure and some other supporting stack variables
APP_CONFIG AppConfig;
static unsigned short wOriginalAppConfigChecksum;    // Checksum of the ROM defaults for AppConfig
BYTE AN0String[8];


// Private helper functions.
// These may or may not be present in all applications.

//Hardare functions
static void InitializePinguino(void);

//TCP IP Stack functions
static void InitAppConfig(void);
static void InitializeBoard(void);
static void ProcessIO(void);

//ModWifi related functions
void MODWifiSetThisIP(void);

//UDP related function
void UDPReceiveByte(WORD Port);
void UDPReceiveArray(WORD Port);
void UDPBroadcastIP(WORD Port);
void UDPBroadcastArray( BYTE * array2send, WORD Port);
void UDPBroadcastByte(BYTE byte, BYTE Port);

//Application related functions
void controlHandleArray(void);
void controlSetDigital(unsigned char data[]);
void controlSetAnalogue(unsigned char data[]);
void controlSetPin(unsigned char data[]);
void controlSendState(BYTE digital[], BYTE analog[]);


#if defined(WF_CS_TRIS)
    void WF_Connect(void);
    #if !defined(MRF24WG)
    extern BOOL gRFModuleVer1209orLater;
    #endif

    #if defined(DERIVE_KEY_FROM_PASSPHRASE_IN_HOST)
    tPassphraseReady g_WpsPassphrase;
    #endif    /* defined(DERIVE_KEY_FROM_PASSPHRASE_IN_HOST) */
#endif


#if defined(__C32__)
    void _general_exception_handler(unsigned cause, unsigned status)
    {
        Nop();
        Nop();
    }
#endif

#if defined(WF_CS_TRIS)
// Global variables
UINT8 ConnectionProfileID;
#endif

///////////////////////////////////////////////////////////////////////////////
////// MY VARIABLES AND SETTINGS . iobridger///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define PIC32_PINGUINO

//  WiFi protocol related definitions
#define SM_BROADCAST        2
#define SM_WAIT_FOR_DATA    3
#define SM_OPEN             1
#define BROADCAST           1
#define SINGLECAST          0
#define UDP_RECBYTE         0
#define UDP_RECARRAY        1
#define ANNOUNCE_UNO        1
#define ANNOUNCE_DOS        2
#define ANNOUNCE_TRES       3
#define ANNOUNCE_CUATRO     4
#define UDP_SEND_PORT       10101
#define UDP_RECEIVE_PORT    10111

//  Control related definitions
#define SET_DIGITAL_VALUE   0x90    //144
#define SET_ANALOG_VALUE    0xE0    //224
#define SET_PIN_MODE        0xF4    //244

// Application specific strings
static BYTE Pinguino[]      = "Pinguino";
static BYTE board[]         = "PIC32 OTG";
static BYTE argument1[]     = "IP A";

// Application variables
static unsigned char smByteState    = SM_OPEN;
static unsigned char smIPState      = SM_OPEN;
static unsigned char smArrayState   = SM_OPEN;
static unsigned char smStateR       = SM_OPEN;
static unsigned char UDPReceiveType = UDP_RECARRAY;
static unsigned char AnnounceIPWifi = 1;
static unsigned char counter        = 1;
static unsigned char Abyte0;
static unsigned char Abyte1;

// UDP related definitions for Datagram Sockets
static UDP_SOCKET  UDPIPSocket;
static UDP_SOCKET  UDPSendByteSocket;
static UDP_SOCKET  UDPSendArraySocket;
static UDP_SOCKET  UDPReceiveSocket;

// UDP and IP definitions
static BYTE UDP_Rec_Byte;
static BYTE UDP_Rec_Byte_Array[3]; //Change size of array by application
static BYTE UDPNumberBytesReceived;
static BOOL UDPBoolReceive;
static BYTE ThisIP[4];

// Control application arrays
static BYTE reportDg[22];
static BYTE reportAn[8];

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////.iobridger////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


/*
// Main application entry point.
*/
int main(void){

    static DWORD dwLastIP = 0;

    // Initialize application specific hardware
    InitializeBoard();

    // Initialize Pinguino specific hardware
    InitializePinguino();



    // Initialize stack-related hardware components that may be
    // required by the UART configuration routines
    TickInit();
    #if defined(STACK_USE_MPFS2)
    MPFSInit();
    #endif

    // Initialize Stack and application related NV variables into AppConfig.
    InitAppConfig();

    // Initialize core stack layers (MAC, ARP, TCP, UDP) and
    // application modules (HTTP, SNMP, etc.)
    StackInit();

    #if defined(WF_CS_TRIS)
    #if defined(DERIVE_KEY_FROM_PASSPHRASE_IN_HOST)
        g_WpsPassphrase.valid = FALSE;
    #endif    /* defined(DERIVE_KEY_FROM_PASSPHRASE_IN_HOST) */
    WF_Connect();
    #endif

    // Now that all items are initialized, begin the co-operative
    // multitasking loop.  This infinite loop will continuously
    // execute all stack-related tasks, as well as your own
    // application's functions.  Custom functions should be added
    // at the end of this loop.
    // Note that this is a "co-operative mult-tasking" mechanism
    // where every task performs its tasks (whether all in one shot
    // or part of it) and returns so that other tasks can do their
    // job.
    // If a task needs very long time to do its job, it must be broken
    // down into smaller pieces so that other tasks can have CPU time.
    while(1)
    {
        // This task performs normal stack task including checking
        // for incoming packet, type of packet and calling
        // appropriate stack entity to process it.
        StackTask();

        #if defined(WF_CS_TRIS)
        #if !defined(MRF24WG)
        if (gRFModuleVer1209orLater)
        #endif
            WiFiTask();
        #endif

        // This tasks invokes each of the core stack application tasks
        StackApplications();

        // Process application specific tasks here.
        // For this demo app, this will include the Generic TCP
        // client and servers, and the SNMP, Ping, and SNMP Trap
        // demos.  Following that, we will process any IO from
        // the inputs on the board itself.
        // Any custom modules or processing you need to do should
        // go here.
        #if defined(STACK_USE_ICMP_CLIENT)
        PingDemo();
        #endif

        if(AnnounceIPWifi == 1){
            UDPBroadcastIP(UDP_SEND_PORT);
        }

        switch (UDPReceiveType){
                case UDP_RECBYTE:
                    UDPReceiveByte(UDP_RECEIVE_PORT);
                    break;
                case UDP_RECARRAY:

                    UDPReceiveArray(UDP_RECEIVE_PORT);
                    controlHandleArray();
                    break;
        }

        ProcessIO();

        // If the local IP address has changed (ex: due to DHCP lease change)
        // write the new IP address to the LCD display, UART, and Announce
        // service
        if(dwLastIP != AppConfig.MyIPAddr.Val)
        {

            AnnounceIPWifi = 1;

            dwLastIP = AppConfig.MyIPAddr.Val;

            #if defined(STACK_USE_UART)
                putrsUART((ROM char*)"\r\nNew IP Address: ");
            #endif

            DisplayIPValue(AppConfig.MyIPAddr);

            #if defined(STACK_USE_UART)
                putrsUART((ROM char*)"\r\n");
            #endif


            #if defined(STACK_USE_ANNOUNCE)
                AnnounceIP();
            #endif


        }

    }
}

#if defined(WF_CS_TRIS)
/*****************************************************************************
 * FUNCTION: WF_Connect
 *
 * RETURNS:  None
 *
 * PARAMS:   None
 *
 *  NOTES:   Connects to an 802.11 network.  Customize this function as needed
 *           for your application.
 *****************************************************************************/
void WF_Connect(void){
    UINT8 channelList[] = MY_DEFAULT_CHANNEL_LIST;

    /* create a Connection Profile */
    WF_CPCreate(&ConnectionProfileID);

    WF_SetRegionalDomain(MY_DEFAULT_DOMAIN);

    WF_CPSetSsid(ConnectionProfileID,
                 AppConfig.MySSID,
                 AppConfig.SsidLength);

    WF_CPSetNetworkType(ConnectionProfileID, MY_DEFAULT_NETWORK_TYPE);

    WF_CASetScanType(MY_DEFAULT_SCAN_TYPE);


    WF_CASetChannelList(channelList, sizeof(channelList));

    // The Retry Count parameter tells the WiFi Connection manager how many attempts to make when trying
    // to connect to an existing network.  In the Infrastructure case, the default is to retry forever so that
    // if the AP is turned off or out of range, the radio will continue to attempt a connection until the
    // AP is eventually back on or in range.  In the Adhoc case, the default is to retry 3 times since the
    // purpose of attempting to establish a network in the Adhoc case is only to verify that one does not
    // initially exist.  If the retry count was set to WF_RETRY_FOREVER in the AdHoc mode, an AdHoc network
    // would never be established.
    WF_CASetListRetryCount(MY_DEFAULT_LIST_RETRY_COUNT);

    WF_CASetEventNotificationAction(MY_DEFAULT_EVENT_NOTIFICATION_LIST);

    WF_CASetBeaconTimeout(MY_DEFAULT_BEACON_TIMEOUT);

    #if !defined(MRF24WG)
        if (gRFModuleVer1209orLater)

    #endif


    #if defined(DERIVE_KEY_FROM_PASSPHRASE_IN_HOST)
        if (AppConfig.SecurityMode == WF_SECURITY_WPA_WITH_PASS_PHRASE
            || AppConfig.SecurityMode == WF_SECURITY_WPA2_WITH_PASS_PHRASE
            || AppConfig.SecurityMode == WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE) {
            WF_ConvPassphrase2Key(AppConfig.SecurityKeyLength, AppConfig.SecurityKey,
                AppConfig.SsidLength, AppConfig.MySSID);
            AppConfig.SecurityMode--;
            AppConfig.SecurityKeyLength = 32;
        }

    #endif    /* defined(DERIVE_KEY_FROM_PASSPHRASE_IN_HOST) */
		#if !defined(MRF24WG)
		Delay10us(10);  //give time to Roadrunner to clean message buffer, because Security message is a big data package
		#endif
    WF_CPSetSecurity(ConnectionProfileID,
                     AppConfig.SecurityMode,
                     AppConfig.WepKeyIndex,   /* only used if WEP enabled */
                     AppConfig.SecurityKey,
                     AppConfig.SecurityKeyLength);

    #if MY_DEFAULT_PS_POLL == WF_ENABLED
        WF_PsPollEnable(TRUE);
    #if !defined(MRF24WG)
        if (gRFModuleVer1209orLater)
            WFEnableDeferredPowerSave();
    #endif    /* !defined(MRF24WG) */
    #else     /* MY_DEFAULT_PS_POLL != WF_ENABLED */
        WF_PsPollDisable();
    #endif    /* MY_DEFAULT_PS_POLL == WF_ENABLED */


    #if defined(STACK_USE_UART)
        WF_OutputConnectionInfo(&AppConfig);
    #endif


    WF_CMConnect(ConnectionProfileID);
}
#endif /* WF_CS_TRIS */

// Writes an IP address to the LCD display and the UART as available
void DisplayIPValue(IP_ADDR IPVal){
// printf("%u.%u.%u.%u", IPVal.v[0], IPVal.v[1], IPVal.v[2], IPVal.v[3]);

    BYTE IPDigit[4];

    BYTE i;


    for(i = 0; i < sizeof(IP_ADDR); i++)
    {
        uitoa((WORD)IPVal.v[i], IPDigit);

        #if defined(STACK_USE_UART)
            putsUART((char *) IPDigit);
        #endif


        if(i == sizeof(IP_ADDR)-1)
                break;

        #if defined(STACK_USE_UART)
            while(BusyUART());
            WriteUART('.');
        #endif
    }

}

// Processes A/D data from the potentiometer
static void ProcessIO(void){
#if defined(__C30__) || defined(__C32__)
    // Convert potentiometer result into ASCII string
    uitoa((WORD)ADC1BUF0, AN0String);

#endif
}


/****************************************************************************
  Function:
    static void InitializeBoard(void)

  Description:
    This routine initializes the hardware.  It is a generic initialization
    routine for many of the Microchip development boards, using definitions
    in HardwareProfile.h to determine specific initialization.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
static void InitializeBoard(void){
    // LEDs
    LED0_TRIS = 0;
    LED1_TRIS = 0;
    LED2_TRIS = 0;
    LED3_TRIS = 0;
    LED4_TRIS = 0;
    LED5_TRIS = 0;
    LED6_TRIS = 0;
    LED7_TRIS = 0;
    LED_PUT(0x00);

#if defined(__18CXX)


#else    // 16-bit C30 and and 32-bit C32
    #if defined(__PIC32MX__)
    {
        // Enable multi-vectored interrupts
        INTEnableSystemMultiVectoredInt();

        // Enable optimal performance
        SYSTEMConfigPerformance(GetSystemClock());
        mOSCSetPBDIV(OSC_PB_DIV_1);                // Use 1:1 CPU Core:Peripheral clocks

        // Disable JTAG port so we get our I/O pins back, but first
        // wait 50ms so if you want to reprogram the part with
        // JTAG, you'll still have a tiny window before JTAG goes away.
        // The PIC32 Starter Kit debuggers use JTAG and therefore must not
        // disable JTAG.
        DelayMs(50);
        #if !defined(__MPLAB_DEBUGGER_PIC32MXSK) && !defined(__MPLAB_DEBUGGER_FS2)
            DDPCONbits.JTAGEN = 0;
        #endif
        LED_PUT(0x00);                // Turn the LEDs off

        CNPUESET = 0x00098000;        // Turn on weak pull ups on CN15, CN16, CN19 (RD5, RD7, RD13), which is connected to buttons on PIC32 Starter Kit boards
    }
    #endif

    #if defined(__dsPIC33F__) || defined(__PIC24H__)
        // Crank up the core frequency
        PLLFBD = 38;                // Multiply by 40 for 160MHz VCO output (8MHz XT oscillator)
        CLKDIV = 0x0000;            // FRC: divide by 2, PLLPOST: divide by 2, PLLPRE: divide by 2

        // Port I/O
        AD1PCFGHbits.PCFG23 = 1;    // Make RA7 (BUTTON1) a digital input
        AD1PCFGHbits.PCFG20 = 1;    // Make RA12 (INT1) a digital input for MRF24W PICtail Plus interrupt

        // ADC
        AD1CHS0 = 0;                // Input to AN0 (potentiometer)
        AD1PCFGLbits.PCFG5 = 0;     // Disable digital input on AN5 (potentiometer)
        AD1PCFGLbits.PCFG4 = 0;     // Disable digital input on AN4 (TC1047A temp sensor)


    #elif defined(__dsPIC33E__)||defined(__PIC24E__)

        // Crank up the core frequency
        PLLFBD = 38;                /* M  = 30   */
        CLKDIVbits.PLLPOST = 0;     /* N1 = 2    */
        CLKDIVbits.PLLPRE = 0;      /* N2 = 2    */
        OSCTUN = 0;

        /*    Initiate Clock Switch to Primary
         *    Oscillator with PLL (NOSC= 0x3)*/
        __builtin_write_OSCCONH(0x03);
        __builtin_write_OSCCONL(0x01);
        // Disable Watch Dog Timer
        RCONbits.SWDTEN = 0;
        while (OSCCONbits.COSC != 0x3);
        while (_LOCK == 0);            /* Wait for PLL lock at 60 MIPS */
        // Port I/O
        ANSELAbits.ANSA7 = 0 ;   //Make RA7 (BUTTON1) a digital input
        #if defined ENC100_INTERFACE_MODE > 0
            ANSELEbits.ANSE0 = 0;      // Make these PMP pins as digital output when the interface is parallel.
            ANSELEbits.ANSE1 = 0;
            ANSELEbits.ANSE2 = 0;
            ANSELEbits.ANSE3 = 0;
            ANSELEbits.ANSE4 = 0;
            ANSELEbits.ANSE5 = 0;
            ANSELEbits.ANSE6 = 0;
            ANSELEbits.ANSE7 = 0;
            ANSELBbits.ANSB10 = 0;
            ANSELBbits.ANSB11 = 0;
            ANSELBbits.ANSB12 = 0;
            ANSELBbits.ANSB13 = 0;
            ANSELBbits.ANSB15 = 0;
        #endif

        ANSELEbits.ANSE8= 0 ;    // Make RE8(INT1) a digital input for ZeroG ZG2100M PICtail

        AD1CHS0 = 0;             // Input to AN0 (potentiometer)
        ANSELBbits.ANSB0= 1;     // Input to AN0 (potentiometer)
        ANSELBbits.ANSB5= 1;     // Disable digital input on AN5 (potentiometer)
        ANSELBbits.ANSB4= 1;     // Disable digital input on AN4 (TC1047A temp sensor)

        ANSELDbits.ANSD7 =0;     //  Digital Pin Selection for S3(Pin 83) and S4(pin 84).
        ANSELDbits.ANSD6 =0;

        ANSELGbits.ANSG6 =0;     // Enable Digital input for RG6 (SCK2)
        ANSELGbits.ANSG7 =0;     // Enable Digital input for RG7 (SDI2)
        ANSELGbits.ANSG8 =0;     // Enable Digital input for RG8 (SDO2)
        ANSELGbits.ANSG9 =0;     // Enable Digital input for RG9 (CS)

        #if defined ENC100_INTERFACE_MODE == 0    // SPI Interface, UART can be used for debugging. Not allowed for other interfaces.
            RPOR9 = 0x0300;          //RP101= U2TX
            RPINR19 = 0X0064;        //RP100= U2RX
        #endif

        #if defined WF_CS_TRIS
            RPINR1bits.INT3R = 30;
            WF_CS_IO = 1;
            WF_CS_TRIS = 0;

        #endif

    #else    //defined(__PIC24F__) || defined(__PIC32MX__)
        #if defined(__PIC24F__)
            CLKDIVbits.RCDIV = 0;        // Set 1:1 8MHz FRC postscalar
        #endif

        // ADC
        #if defined(__PIC24FJ256DA210__) || defined(__PIC24FJ256GB210__)
            // Disable analog on all pins
            ANSA = 0x0000;
            ANSB = 0x0000;
            ANSC = 0x0000;
            ANSD = 0x0000;
            ANSE = 0x0000;
            ANSF = 0x0000;
            ANSG = 0x0000;
        #else
            AD1CHS = 0;                   // Input to AN0 (potentiometer)
            AD1PCFGbits.PCFG4 = 0;        // Disable digital input on AN4 (TC1047A temp sensor)
            #if defined(__32MX460F512L__) || defined(__32MX795F512L__)    // PIC32MX460F512L and PIC32MX795F512L PIMs has different pinout to accomodate USB module
                AD1PCFGbits.PCFG2 = 0;    // Disable digital input on AN2 (potentiometer)
            #else
                AD1PCFGbits.PCFG5 = 0;    // Disable digital input on AN5 (potentiometer)
            #endif
        #endif
    #endif

    // ADC
    AD1CON1 = 0x84E4;            // Turn on, auto sample start, auto-convert, 12 bit mode (on parts with a 12bit A/D)
    AD1CON2 = 0x0404;            // AVdd, AVss, int every 2 conversions, MUXA only, scan
    AD1CON3 = 0x1003;            // 16 Tad auto-sample, Tad = 3*Tcy
    #if defined(__32MX460F512L__) || defined(__32MX795F512L__)    // PIC32MX460F512L and PIC32MX795F512L PIMs has different pinout to accomodate USB module
        AD1CSSL = 1<<2;                // Scan pot
    #else
        AD1CSSL = 1<<5;                // Scan pot
    #endif

    // UART
    #if defined(STACK_USE_UART)

        #if defined(__PIC24E__) || defined(__dsPIC33E__)
            #if defined (ENC_CS_IO) || defined (WF_CS_IO)   // UART to be used in case of ENC28J60 or MRF24W
                __builtin_write_OSCCONL(OSCCON & 0xbf);
                RPOR9bits.RP101R = 3; //Map U2TX to RF5
                RPINR19bits.U2RXR = 0;
                RPINR19bits.U2RXR = 0x64; //Map U2RX to RF4
                __builtin_write_OSCCONL(OSCCON | 0x40);
            #endif
            #if(ENC100_INTERFACE_MODE == 0)                 // UART to be used only in case of SPI interface with ENC624Jxxx
                    __builtin_write_OSCCONL(OSCCON & 0xbf);
                RPOR9bits.RP101R = 3; //Map U2TX to RF5
                RPINR19bits.U2RXR = 0;
                RPINR19bits.U2RXR = 0x64; //Map U2RX to RF4
                __builtin_write_OSCCONL(OSCCON | 0x40);

            #endif
        #endif

        UARTTX_TRIS = 0;
        UARTRX_TRIS = 1;
        UMODE = 0x8000;            // Set UARTEN.  Note: this must be done before setting UTXEN

        #if defined(__C30__)
            USTA = 0x0400;        // UTXEN set
            #define CLOSEST_UBRG_VALUE ((GetPeripheralClock()+8ul*BAUD_RATE)/16/BAUD_RATE-1)
            #define BAUD_ACTUAL (GetPeripheralClock()/16/(CLOSEST_UBRG_VALUE+1))
        #else    //defined(__C32__)
            USTA = 0x00001400;        // RXEN set, TXEN set
            #define CLOSEST_UBRG_VALUE ((GetPeripheralClock()+8ul*BAUD_RATE)/16/BAUD_RATE-1)
            #define BAUD_ACTUAL (GetPeripheralClock()/16/(CLOSEST_UBRG_VALUE+1))
        #endif

        #define BAUD_ERROR ((BAUD_ACTUAL > BAUD_RATE) ? BAUD_ACTUAL-BAUD_RATE : BAUD_RATE-BAUD_ACTUAL)
        #define BAUD_ERROR_PRECENT    ((BAUD_ERROR*100+BAUD_RATE/2)/BAUD_RATE)
        #if (BAUD_ERROR_PRECENT > 3)
            #warning UART frequency error is worse than 3%
        #elif (BAUD_ERROR_PRECENT > 2)
            #warning UART frequency error is worse than 2%
        #endif

        UBRG = CLOSEST_UBRG_VALUE;
    #endif

#endif

// Deassert all chip select lines so there isn't any problem with
// initialization order.  Ex: When ENC28J60 is on SPI2 with Explorer 16,
// MAX3232 ROUT2 pin will drive RF12/U2CTS ENC28J60 CS line asserted,
// preventing proper 25LC256 EEPROM operation.

#if defined(WF_CS_TRIS)
    WF_CS_IO = 1;
    WF_CS_TRIS = 0;
#endif


}

/*********************************************************************
 * Function:        void InitAppConfig(void)
 *
 * PreCondition:    MPFSInit() is already called.
 *
 * Input:           None
 *
 * Output:          Write/Read non-volatile config variables.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
// MAC Address Serialization using a MPLAB PM3 Programmer and
// Serialized Quick Turn Programming (SQTP).
// The advantage of using SQTP for programming the MAC Address is it
// allows you to auto-increment the MAC address without recompiling
// the code for each unit.  To use SQTP, the MAC address must be fixed
// at a specific location in program memory.  Uncomment these two pragmas
// that locate the MAC address at 0x1FFF0.  Syntax below is for MPLAB C
// Compiler for PIC18 MCUs. Syntax will vary for other compilers.
//#pragma romdata MACROM=0x1FFF0
static ROM BYTE SerializedMACAddress[6] = {MY_DEFAULT_MAC_BYTE1, MY_DEFAULT_MAC_BYTE2, MY_DEFAULT_MAC_BYTE3, MY_DEFAULT_MAC_BYTE4, MY_DEFAULT_MAC_BYTE5, MY_DEFAULT_MAC_BYTE6};
//#pragma romdata
static void InitAppConfig(void){


    while(1)
    {
        // Start out zeroing all AppConfig bytes to ensure all fields are
        // deterministic for checksum generation
        memset((void*)&AppConfig, 0x00, sizeof(AppConfig));

        AppConfig.Flags.bIsDHCPEnabled = TRUE;
        AppConfig.Flags.bInConfigMode = TRUE;
        memcpypgm2ram((void*)&AppConfig.MyMACAddr, (ROM void*)SerializedMACAddress, sizeof(AppConfig.MyMACAddr));
//        {
//            _prog_addressT MACAddressAddress;
//            MACAddressAddress.next = 0x157F8;
//            _memcpy_p2d24((char*)&AppConfig.MyMACAddr, MACAddressAddress, sizeof(AppConfig.MyMACAddr));
//        }
        AppConfig.MyIPAddr.Val = MY_DEFAULT_IP_ADDR_BYTE1 | MY_DEFAULT_IP_ADDR_BYTE2<<8ul | MY_DEFAULT_IP_ADDR_BYTE3<<16ul | MY_DEFAULT_IP_ADDR_BYTE4<<24ul;
        AppConfig.DefaultIPAddr.Val = AppConfig.MyIPAddr.Val;
        AppConfig.MyMask.Val = MY_DEFAULT_MASK_BYTE1 | MY_DEFAULT_MASK_BYTE2<<8ul | MY_DEFAULT_MASK_BYTE3<<16ul | MY_DEFAULT_MASK_BYTE4<<24ul;
        AppConfig.DefaultMask.Val = AppConfig.MyMask.Val;
        AppConfig.MyGateway.Val = MY_DEFAULT_GATE_BYTE1 | MY_DEFAULT_GATE_BYTE2<<8ul | MY_DEFAULT_GATE_BYTE3<<16ul | MY_DEFAULT_GATE_BYTE4<<24ul;
        AppConfig.PrimaryDNSServer.Val = MY_DEFAULT_PRIMARY_DNS_BYTE1 | MY_DEFAULT_PRIMARY_DNS_BYTE2<<8ul  | MY_DEFAULT_PRIMARY_DNS_BYTE3<<16ul  | MY_DEFAULT_PRIMARY_DNS_BYTE4<<24ul;
        AppConfig.SecondaryDNSServer.Val = MY_DEFAULT_SECONDARY_DNS_BYTE1 | MY_DEFAULT_SECONDARY_DNS_BYTE2<<8ul  | MY_DEFAULT_SECONDARY_DNS_BYTE3<<16ul  | MY_DEFAULT_SECONDARY_DNS_BYTE4<<24ul;



        // Load the default NetBIOS Host Name
        memcpypgm2ram(AppConfig.NetBIOSName, (ROM void*)MY_DEFAULT_HOST_NAME, 16);
        FormatNetBIOSName(AppConfig.NetBIOSName);

        #if defined(WF_CS_TRIS)
            // Load the default SSID Name
            WF_ASSERT(sizeof(MY_DEFAULT_SSID_NAME)-1 <= sizeof(AppConfig.MySSID));
            memcpypgm2ram(AppConfig.MySSID, (ROM void*)MY_DEFAULT_SSID_NAME, sizeof(MY_DEFAULT_SSID_NAME));
            AppConfig.SsidLength = sizeof(MY_DEFAULT_SSID_NAME) - 1;

            AppConfig.SecurityMode = MY_DEFAULT_WIFI_SECURITY_MODE;

            #if (MY_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_OPEN)
                memset(AppConfig.SecurityKey, 0x00, sizeof(AppConfig.SecurityKey));
                AppConfig.SecurityKeyLength = 0;

            #elif MY_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WEP_40
                AppConfig.WepKeyIndex  = MY_DEFAULT_WEP_KEY_INDEX;
                memcpypgm2ram(AppConfig.SecurityKey, (ROM void*)MY_DEFAULT_WEP_KEYS_40, sizeof(MY_DEFAULT_WEP_KEYS_40) - 1);
                AppConfig.SecurityKeyLength = sizeof(MY_DEFAULT_WEP_KEYS_40) - 1;

            #elif MY_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WEP_104
                AppConfig.WepKeyIndex  = MY_DEFAULT_WEP_KEY_INDEX;
                memcpypgm2ram(AppConfig.SecurityKey, (ROM void*)MY_DEFAULT_WEP_KEYS_104, sizeof(MY_DEFAULT_WEP_KEYS_104) - 1);
                AppConfig.SecurityKeyLength = sizeof(MY_DEFAULT_WEP_KEYS_104) - 1;

            #elif (MY_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_WITH_KEY)       || \
                  (MY_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA2_WITH_KEY)      || \
                  (MY_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_AUTO_WITH_KEY)
                memcpypgm2ram(AppConfig.SecurityKey, (ROM void*)MY_DEFAULT_PSK, sizeof(MY_DEFAULT_PSK) - 1);
                AppConfig.SecurityKeyLength = sizeof(MY_DEFAULT_PSK) - 1;

            #elif (MY_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_WITH_PASS_PHRASE)     || \
                  (MY_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA2_WITH_PASS_PHRASE)    || \
                  (MY_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE)
                memcpypgm2ram(AppConfig.SecurityKey, (ROM void*)MY_DEFAULT_PSK_PHRASE, sizeof(MY_DEFAULT_PSK_PHRASE) - 1);
                AppConfig.SecurityKeyLength = sizeof(MY_DEFAULT_PSK_PHRASE) - 1;
            #elif (MY_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPS_PUSH_BUTTON)
                memset(AppConfig.SecurityKey, 0x00, sizeof(AppConfig.SecurityKey));
                AppConfig.SecurityKeyLength = 0;
            #elif (MY_DEFAULT_WIFI_SECURITY_MODE == WF_SECURITY_WPS_PIN)
                memcpypgm2ram(AppConfig.SecurityKey, (ROM void*)MY_DEFAULT_WPS_PIN, sizeof(MY_DEFAULT_WPS_PIN) - 1);
                AppConfig.SecurityKeyLength = sizeof(MY_DEFAULT_WPS_PIN) - 1;
            #else
                #error "No security defined"
            #endif /* MY_DEFAULT_WIFI_SECURITY_MODE */

        #endif

        // Compute the checksum of the AppConfig defaults as loaded from ROM
        wOriginalAppConfigChecksum = CalcIPChecksum((BYTE*)&AppConfig, sizeof(AppConfig));


        break;
    }
}

/********************************************************************
     * Function:        void UDPReportIP(WORD Port)
     * Input:           WORD Port - the port through which to broadcast IP address.
     * Output:          None
     * Overview:        This function opens an UDP Socket named UDPIPSocket
     *                  and uses it to broadcast the current IP address of this 
     *                  chip through port 'Port', it calls function
     *                  MODWifiSetThisIP()
     *                  to get such IP address into "ThisIP" array. It then 
     *                  broacastst the message /Pinguino/PIC32 OTG/IP A/address
     *                  where 'address' is the IP address. 
     * Note:            None
     *******************************************************************/
void UDPBroadcastIP(WORD Port){

switch(smIPState){

            case SM_OPEN:
                // Talk to a remote DHCP server.
                UDPIPSocket = UDPOpenEx(0, UDP_OPEN_SERVER,0,Port);

                if( UDPIPSocket == INVALID_UDP_SOCKET ){
                // Socket is not available
                // Return error.
                }
                else
                // Broadcast DHCP Broadcast message.
                smIPState = SM_BROADCAST;

            break;

            case SM_BROADCAST:
                //unsigned short int simon;

                if ( UDPIsPutReady(UDPIPSocket) ){
                // Socket is ready to transmit. Transmit the data...
                // Note that there is DHCPSocket parameter in UDPPut.
                // This UDPPut call will use active socket
                // as set by UDPIsPutReady() - that is DHCPSocket.
                    MODWifiSetThisIP();

                    UDPPut(0x2F);                           //  "/"
                    UDPPutArray(Pinguino, sizeof(Pinguino));// "Pinguino"
                    UDPPut(0x2F);                           //  "/"
                    UDPPutArray(board, sizeof(board));      // "PIC32 OTG"
                    UDPPut(0x2F);                           //  "/"
                    UDPPutArray(argument1, sizeof(argument1));// "IP A"
                    UDPPut(0x2F);                           //  "/"
                    UDPPutArray(ThisIP, sizeof(ThisIP));    //

                // Now transmit it.
                // In the server side 32 bits are received.
                    UDPFlush();

                }
            break;

    }

}

/********************************************************************
     * Function:        MODWifiSetThisIP(void)
     * Input:           None
     * Output:          None
     * Overview:        This function takes the bits of the current chip's
     *                  IP address from the structure AppConfig.MyIPAddr.bits
     *                  and then converts it into decimal notation to be
     *                  displayed and send with function
     *                  void UDPReportIP(void)
     * Note:            None
     *******************************************************************/
void MODWifiSetThisIP(void){

    ThisIP[0] = ((AppConfig.MyIPAddr.bits.b7) * 128 )+ ((AppConfig.MyIPAddr.bits.b6) * 64 )+
                ((AppConfig.MyIPAddr.bits.b5) * 32 )+ ((AppConfig.MyIPAddr.bits.b4) * 16) +
                ((AppConfig.MyIPAddr.bits.b3) * 8 )+ ((AppConfig.MyIPAddr.bits.b2) * 4 )+
                ((AppConfig.MyIPAddr.bits.b1) * 2 )+ ((AppConfig.MyIPAddr.bits.b0) * 1);

    ThisIP[1] = ((AppConfig.MyIPAddr.bits.b15) * 128 )+ ((AppConfig.MyIPAddr.bits.b14) * 64 )+
                ((AppConfig.MyIPAddr.bits.b13) * 32 )+ ((AppConfig.MyIPAddr.bits.b12) * 16) +
                ((AppConfig.MyIPAddr.bits.b11) * 8 )+ ((AppConfig.MyIPAddr.bits.b10) * 4 )+
                ((AppConfig.MyIPAddr.bits.b9) * 2 )+ ((AppConfig.MyIPAddr.bits.b8) * 1);

    ThisIP[2] = ((AppConfig.MyIPAddr.bits.b23) * 128 )+ ((AppConfig.MyIPAddr.bits.b22) * 64 )+
                ((AppConfig.MyIPAddr.bits.b21) * 32 )+ ((AppConfig.MyIPAddr.bits.b20) * 16) +
                ((AppConfig.MyIPAddr.bits.b19) * 8 )+ ((AppConfig.MyIPAddr.bits.b18) * 4 )+
                ((AppConfig.MyIPAddr.bits.b17) * 2 )+ ((AppConfig.MyIPAddr.bits.b16) * 1);

    ThisIP[3] = ((AppConfig.MyIPAddr.bits.b31) * 128 )+ ((AppConfig.MyIPAddr.bits.b30) * 64 )+
                ((AppConfig.MyIPAddr.bits.b29) * 32 )+ ((AppConfig.MyIPAddr.bits.b28) * 16) +
                ((AppConfig.MyIPAddr.bits.b27) * 8 )+ ((AppConfig.MyIPAddr.bits.b26) * 4 )+
                ((AppConfig.MyIPAddr.bits.b25) * 2 )+ ((AppConfig.MyIPAddr.bits.b24) * 1);

}

/********************************************************************
     * Function:        UDPReceiveByte(WORD Port)
     * Input:           The port number through which a byte is received.
     * Output:          None
     * Overview:        This function calls UDPGet to receive a byte from the Port
     *                  specified in the argument, it stores the byte in the
     *                  buffer "UDP_Rec_Byte".
     * Note:            None
     *******************************************************************/
void UDPReceiveByte(WORD Port){

    switch(smStateR){

           case SM_OPEN:
           // Talk to a remote DHCP server.
           UDPReceiveSocket = UDPOpenEx((DWORD)NULL, UDP_OPEN_SERVER,Port,Port);

           if ( UDPReceiveSocket == INVALID_UDP_SOCKET ){
                // Socket is not available
                // Return error.
           }
           else
                // Wait for response from DHCP server
                smStateR = SM_WAIT_FOR_DATA;
           break;

           case SM_WAIT_FOR_DATA:
           if ( UDPIsGetReady(UDPReceiveSocket) ){

               // Socket does contain some data. Fetch it all.
               // buffer is a pointer to BYTE.
               // Process it..
              UDPBoolReceive = UDPGet(&UDP_Rec_Byte);

              UDPDiscard();
          }
          break;

    }

}

/********************************************************************
     * Function:        UDPReceiveArray(WORD Port)
     * Input:           The port number through which the array is coming in.
     * Output:          None
     * Overview:        This function calls UDPGet to receive a byte array from
     *                  the Port specified in the argument, it stores the array
     *                  in the buffer "UDP_Rec_Byte_Array".
     *                  It stores the number of bytes in the received array in
     *                  UDPNumberBytesReceived.
     * Note:            None
     *******************************************************************/
void UDPReceiveArray(WORD Port){

    switch(smStateR){

           case SM_OPEN:
           // Talk to a remote DHCP server.
           UDPReceiveSocket = UDPOpenEx((DWORD)NULL, UDP_OPEN_SERVER,Port,Port);

           if ( UDPReceiveSocket == INVALID_UDP_SOCKET ){
                // Socket is not available
                // Return error.
           }
           else
                // Wait for response from DHCP server
                smStateR = SM_WAIT_FOR_DATA;
           break;

           case SM_WAIT_FOR_DATA:
           if ( UDPIsGetReady(UDPReceiveSocket) ){

               // Socket does contain some data. Fetch it all.
               // buffer is a pointer to BYTE.
               // Process it..
              UDPNumberBytesReceived = UDPGetArray((BYTE*)&UDP_Rec_Byte_Array, sizeof(UDP_Rec_Byte_Array));

              UDPDiscard();
          }
          break;

    }

}

/********************************************************************
     * Function:        controlHandleArray(void)
     * Input:           None
     * Output:          None
     * Overview:        This function evaluates the content of UDP_Rec_Byte_Array
     *                  and directs it to the corresponding function, depending on
     *                  the value in its first position. For more details on this
     *                  communication protocol refer to the .xls document included
 ``  *                  in the folder of this file and project.
     * Note:            None
     *******************************************************************/
void controlHandleArray(void){
    
    if ((UDP_Rec_Byte_Array[0] >= SET_DIGITAL_VALUE) && (UDP_Rec_Byte_Array[0] < 0xA0)){

                 //AnnounceIPWifi = 0;             // Stop broadcasting IP address.
                 controlSetDigital(UDP_Rec_Byte_Array);// Perform action given the coded message

              }

            else if ((UDP_Rec_Byte_Array[0] >= SET_ANALOG_VALUE) && (UDP_Rec_Byte_Array[0] < SET_PIN_MODE)){

                //AnnounceIPWifi = 0;             // Stop broadcasting IP address.
                controlSetAnalogue(UDP_Rec_Byte_Array);// Perform action given the coded message

            }

            else if ((UDP_Rec_Byte_Array[0] >= SET_PIN_MODE) && (UDP_Rec_Byte_Array[0] < 0xFF)){// SEE MIDI MESSAGE FORMAT

                //AnnounceIPWifi = 0;             // Stop broadcasting IP address.
                controlSetPin(UDP_Rec_Byte_Array);// Perform action given the coded message

            }

            counter++;

            // REPORT ONLY ONCE EVERY 20 TIMES, TO AVOID SATURATION OF RECEIVER.
            if(counter == 20){
                controlSendState(reportDg, reportAn);
                counter = 0;
            }
    
    
}

/********************************************************************
     * Function:        UDPBroadcasByte(BYTE byte, BYTE Port)
     * Input:           The byte to send and port through which to broadcast it.
     * Output:          None
     * Overview:        This function broadcasts a single byte as an UDP
     *                  package, using the port on the argument.
     * Note:            None
     *******************************************************************/
void UDPBroadcastByte(BYTE byte, BYTE Port){

switch(smByteState){

           case SM_OPEN:
                // Talk to a remote DHCP server.
                UDPSendByteSocket = UDPOpenEx(0, UDP_OPEN_SERVER,0,Port);

                if( UDPSendByteSocket == INVALID_UDP_SOCKET ){
                // Socket is not available
                // Return error.
                }

                else
                    // Broadcast DHCP Broadcast message.
                    smByteState = SM_BROADCAST;
            break;

            case SM_BROADCAST:

                if ( UDPIsPutReady(UDPSendByteSocket) ){
                // Socket is ready to transmit. Transmit the data...
                // Note that there is DHCPSocket parameter in UDPPut.
                // This UDPPut call will use active socket
                // as set by UDPIsPutReady() - that is DHCPSocket.

                UDPPut(byte);

                // Now transmit it.
                UDPFlush();

            }
            break;

    }

}

/********************************************************************
     * Function:        UDPBroadcasArray(BYTE byte, BYTE Port)
     * Input:           The byte array to send and port through which to send it.
     * Output:          None
     * Overview:        This function broadcasts a byte array as an UDP
     *                  package, using the port on the argument.
     * Note:            None
     *******************************************************************/
void UDPBroadcastArray(BYTE * array2send, WORD Port){

        switch(smArrayState){

            case SM_OPEN:
                // Talk to a remote DHCP server.
                UDPSendArraySocket = UDPOpenEx(0, UDP_OPEN_SERVER,0,Port);
           
                if( UDPSendArraySocket == INVALID_UDP_SOCKET ){
                // Socket is not available
                // Return error.
                }

                else
                    // Broadcast DHCP Broadcast message.
                    smArrayState = SM_BROADCAST;
            break;

            case SM_BROADCAST:
                
                if ( UDPIsPutReady(UDPSendArraySocket) ){
                // Socket is ready to transmit. Transmit the data...
                // Note that there is DHCPSocket parameter in UDPPut.
                // This UDPPut call will use active socket
                // as set by UDPIsPutReady() - that is DHCPSocket.

                UDPPutArray(array2send, sizeof(array2send));

                //...
                // Now transmit it.
                UDPFlush();

            }
            break;

    }

}

    /********************************************************************
     * Function:        void controlSetDigital(unsigned char data[])
     * Input:           unsigned char data[]
     * Output:          None
     * Overview:        This function takes an array of size 3, which contains
     *                  data encoded in a MIDI type of format, and uses it to
     *                  set digital values (1 or 0) to Pins on the PIC. In this
     *                  array the first byte, or data[0], has information about
     *                  the port to address. data[1] contains the pin to set,
     *                  and data[2] contains the actual value.
     *                  For the case of pins where Analog outpu is possible, the
     *                  function turns the Output Compare Module off.
     *                  Refer to PINGUINO_PD_INTERFACE.xls for details on the
     *                  messaging format.
     * Note:            None
     *******************************************************************/
    void controlSetDigital(unsigned char data[]){
        int port = data[0] - SET_DIGITAL_VALUE;
    #if defined(PIC32_PINGUINO) || defined(PIC32_PINGUINO_OTG)
        switch (port){

            case 0:// PORT B
                switch (data[2]){
                    case 0:// LOW
                        LATBCLR = pow(2,data[1]);
                    break;
                    case 1:// HIGH
                        LATBSET = pow(2,data[1]);
                    break;
                }
            break;

             case 1:// PORT C
                switch (data[2]){
                    case 0:// LOW
                        LATCCLR = pow(2,data[1]);
                    break;
                    case 1:// HIGH
                        LATCSET = pow(2,data[1]);
                    break;
                }
             break;

             case 2:// PORT D
                switch (data[2]){
                    case 0:// LOW
                        switch(data[1]){
                            case 2:
                                OC3CON = 0;//Output Compare Module off
                                break;
                            case 3:
                                OC4CON = 0;//Output Compare Module off
                                break;
                            case 4:
                                OC5CON = 0;//Output Compare Module off
                                OC1CON = 0;//Output Compare Module off
                                LATDCLR = pow(2,0);
                                break;
                            case 1:
                                OC2CON = 0;//Output Compare Module off
                                break;

                        }
                        LATDCLR = pow(2,data[1]);
                    break;
                    case 1:// HIGH
                        switch(data[1]){
                            case 2:
                                OC3CON = 0;//Output Compare Module off
                                break;
                            case 3:
                                OC4CON = 0;//Output Compare Module off
                                break;
                            case 4:
                                OC5CON = 0;//Output Compare Module off
                                OC1CON = 0;//Output Compare Module off
                                LATDSET = pow(2,0);
                                break;
                            case 1:
                                OC2CON = 0;//Output Compare Module off
                                break;

                        }
                        LATDSET = pow(2,data[1]);
                    break;
                }
             break;

             case 3:// PORT E
                switch (data[2]){
                    case 0:// LOW
                        LATECLR = pow(2, data[1]);
                    break;
                    case 1:// HIGH
                        LATESET = pow(2, data[1]);
                    break;
                }
             break;

             case 4:// PORT F
                switch (data[2]){
                    case 0:// LOW
                        LATFCLR = pow(2, data[1]);
                    break;
                    case 1:// HIGH
                        LATFSET = pow(2, data[1]);
                    break;
                }
             break;

             case 5:// PORT G
                switch (data[2]){

                    case 0:// LOW
                        LATGCLR = pow(2, data[1]);
                    break;
                    case 1:// HIGH
                        LATGSET = pow(2, data[1]);
                    break;
                }
                break;
        }
    #endif

    }


     /********************************************************************
     * Function:        void controlSetAnalogue(unsigned char data[])
     * Input:           unsigned char data[]
     * Output:          None
     * Overview:        This function takes an array of size 3, which contains
     *                  data encoded in a MIDI type of format, and uses it to
     *                  set Analog values (10 bits) to Pins on the PIC. In this
     *                  array the first byte, or data[0] gives information about
     *                  the port and pin to address. data[1] contains the Most
     *                  Significant byte (bits 9 and 8), while  data[2] contains
     *                  the Less Significant byte (bits 0 - 7).
     *                  Refer to PINGUINO_PD_INTERFACE.xls for details on the
     *                  messaging format.
     * Note:            None
     *******************************************************************/
    void controlSetAnalogue(unsigned char data[]){

        int channel = data[0] - SET_ANALOG_VALUE;
        UINT16  setpoint;

    #if defined(PIC32_PINGUINO) || defined(PIC32_PINGUINO_OTG)
        switch (channel){

            case 0:// PWM 0
                setpoint = (256*data[1])+data[2];
                OpenOC3( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
                OpenTimer2( T2_ON | T2_PS_1_1 | T2_SOURCE_INT, 0xFFFF);
                SetDCOC3PWM(setpoint);
                //analogwrite(D0, setpoint);// Legacy Pinguino Language Function

                break;

            case 1:// PWM 1
                setpoint = (256*data[1])+data[2];
                OpenOC4( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
                OpenTimer2( T2_ON | T2_PS_1_1 | T2_SOURCE_INT, 0xFFFF);
                SetDCOC4PWM(setpoint);
                //analogwrite(D1, setpoint);// Legacy Pinguino Language Function
                break;

            case 2:// PWM 2
                setpoint = (256*data[1])+data[2];
                OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
                OpenOC5( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
                OpenTimer2( T2_ON | T2_PS_1_1 | T2_SOURCE_INT, 0xFFFF);
                SetDCOC1PWM(setpoint);
                SetDCOC5PWM(setpoint);
                //analogwrite(D2, setpoint);// Legacy Pinguino Language Function
                break;

            case 3:// PWM 3
                setpoint = (256*data[1])+data[2];
                //OpenOC4( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
                //OpenTimer2( T2_ON | T2_PS_1_1 | T2_SOURCE_INT, 0xFFFF);
                //SetDCOC4PWM(setpoint);
                //analogwrite(LED2, setpoint);// Legacy Pinguino Language Function
                break;

        }
    #endif
    }


     /********************************************************************
     * Function:        void controlSetPin(unsigned char data[])
     * Input:           unsigned char data[]
     * Output:          None
     * Overview:        This function takes an array of size 3, which contains
     *                  data encoded in a MIDI type of format, and uses it to
     *                  set Pin Modes (Digital Input, Digital Output and
     *                  Analog Input) on the PIC.
     *                  In this array the first byte, or data[0] gives
     *                  information about the port to address. data[1]
     *                  contains the Pin number in such port, and data[2]
     *                  determins whether it is Digitl Input (1),
     *                  Digital Output(0) or Analog Input (2).
     *                  Refer to PINGUINO_PD_INTERFACE.xls for details on the
     *                  messaging format.
     * Note:            None
     *******************************************************************/
    void controlSetPin(unsigned char data[]){

        int port = data[0] - SET_PIN_MODE;

    #if defined(PIC32_PINGUINO) || defined(PIC32_PINGUINO_OTG)
        switch (port){

            case 0:// PORT B
                switch (data[2]){
                    case 0:// DIGITAL OUTPUT

                        AD1PCFGSET = pow(2, data[1]);   //SET THIS PIN TO DIGITAL
                        TRISBCLR = pow(2,data[1]);      //SET TO OUTPUT
                        if(AD1PCFG == 0x0000){
                            AD1CON1CLR = 0x8000;        //TURN ADC OFF
                        }
                    break;
                    case 1:// DIGITAL INPUT

                        AD1PCFGSET = pow(2, data[1]);   //SET THIS PIN TO DIGITAL
                        TRISBSET = pow(2, data[1]);     //SET TO INPUT
                        if(AD1PCFG == 0x0000){
                            AD1CON1CLR = 0x8000;        //TURN ADC OFF
                        }
                    break;
                    case 2:// ANALOG INPUT

                        TRISBCLR = pow(2, data[1]);     //SET TO OUTPUT
                        PORTBCLR = pow(2, data[1]);     //SET TO ZERO, PRECAUTION
                        TRISBSET = pow(2, data[1]);     //SET TO INPUT
                        AD1PCFGCLR = pow(2, data[1]);   //SET THIS PIN TO ANALOG


                        AD1CON1SET = 0x8000;            //ADC ON
                        AD1CON1SET = 0x0004;            //START SAMPLING.


                    break;
                }
             break;

             case 1:// PORT C
                switch (data[2]){
                    case 0:// DIGITAL OUTPUT
                        TRISCCLR = pow(2, data[1]);
                    break;
                    case 1:// DIGITAL INPUT
                        TRISCSET = pow(2, data[1]);
                    break;
                }
             break;

             case 2:// PORT D
                switch (data[2]){
                    case 0:// DIGITAL OUTPUT
                        switch(data[1]){
                            case 2:
                                OC3CON = 0;
                                break;
                            case 3:
                                OC4CON = 0;
                                break;
                            case 4:
                                OC5CON = 0;
                                OC1CON = 0;
                                TRISDCLR = pow(2, 0);
                                break;
                            case 1:
                                OC2CON = 0;
                                break;

                        }
                        TRISDCLR = pow(2, data[1]);
                    break;
                    case 1:// DIGITAL INPUT
                        switch(data[1]){
                            case 2:
                                OC3CON = 0;
                                break;
                            case 3:
                                OC4CON = 0;
                                break;
                            case 4:
                                OC5CON = 0;
                                OC1CON = 0;
                                TRISDSET = pow(2, 0);
                                break;
                            case 1:
                                OC2CON = 0;
                                break;

                        }
                        TRISDSET = pow(2, data[1]);
                    break;
                }
             break;

             case 3:// PORT E
                switch (data[2]){
                    case 0:// DIGITAL OUTPUT
                        TRISECLR = pow(2, data[1]);
                    break;
                    case 1:// DIGITAL INPUT
                        TRISESET = pow(2, data[1]);
                    break;
                }
             break;

             case 4:// PORT F
                switch (data[2]){
                    case 0:// DIGITAL OUTPUT
                        TRISFCLR = pow(2, data[1]);
                    break;
                    case 1:// DIGITAL INPUT
                        TRISFSET = pow(2, data[1]);
                    break;
                }
             break;

             case 5:// PORT G
                switch (data[2]){
                    case 0:// DIGITAL OUTPUT
                        TRISGCLR = pow(2, data[1]);
                    break;
                    case 1:// DIGITAL INPUT
                        TRISGSET = pow(2, data[1]);
                    break;
                }
             break;
        }
    #endif

    }


    /********************************************************************
     * Function:        void controlSendState(unsigned char Digital[],
     *                                          unsigned long int Analog[])
     * Input:           unsigned char Digital[], unsigned long int Analog[]
     * Output:          None
     * Overview:        This function fills the arrays given to it with the
     *                  current values of the Digital and Analog ports. For the
     *                  Digital, it reads the value stored in the PORTx register
     *                  whereas for the analog values, it reads through the ADC
     *                  Buffers, only giving back 8 values, since the Pinguino
     *                  here only has 8 Analog inputs. Then the function writes
     *                  this values through the CDC one by one, writing the
     *                  number 100 before the digital set of values, and a
     *                  200 before the Analog set, this is used in Pure Data to
     *                  demultiplex the information.
     * Note:            None
     *******************************************************************/
    void controlSendState(BYTE Digital[], BYTE Analog[]){

    #if defined(PIC32_PINGUINO) || defined(PIC32_PINGUINO_OTG)
        Digital[0] =  PORTDbits.RD2;
        Digital[1] =  PORTDbits.RD3;
        Digital[2] =  PORTDbits.RD4;
        Digital[3] =  PORTDbits.RD5;
        Digital[4] =  PORTDbits.RD6;    //THESE BITS CORRESPOND TO THOSE FOUND IN
        Digital[5] =  PORTDbits.RD7;    // CON 4 AND 5 IN PINGUINO PIC32 OTG
        Digital[6] =  PORTDbits.RD8;    // AND ARE EXCLUSIVELY DIGITAL
        Digital[7] =  PORTDbits.RD11;
        Digital[8] =  PORTBbits.RB13;
        Digital[9] =  PORTBbits.RB14;
        Digital[10] =  PORTGbits.RG9;
        Digital[11] =  PORTGbits.RG8;
        Digital[12] =  PORTGbits.RG7;
        Digital[13] =  PORTGbits.RG6;

        Digital[14] =  PORTBbits.RB1;
        Digital[15] =  PORTBbits.RB2;
        Digital[16] =  PORTBbits.RB3;    //THESE BITS CORRESPOND TO THOSE FOUND IN
        Digital[17] =  PORTBbits.RB4;    // CON 1 AND 2 IN PINGUINO PIC32 OTG
        Digital[18] =  PORTBbits.RB8;    // AND ARE ANALOG INPUTS AS WELL
        Digital[19] =  PORTBbits.RB9;
        Digital[20] =  PORTBbits.RB11;
        Digital[21] =  PORTBbits.RB10;


        Analog[0] = ADC1BUF0;
        Analog[1] = ADC1BUF1;
        Analog[2] = ADC1BUF2;
        Analog[3] = ADC1BUF3;           // ADC BUFFER
        Analog[4] = ADC1BUF4;
        Analog[5] = ADC1BUF5;
        Analog[6] = ADC1BUF6;
        Analog[7] = ADC1BUF7;

        int i = 0;
        UDPBroadcastByte(100, (BYTE)UDP_SEND_PORT);

        for(i = 0; i < 22; i++){        // REPORT DIGITAL PINS' VALUES
            //Delayms(5);
            UDPBroadcastByte(Digital[i], (BYTE)UDP_SEND_PORT);

        }

        i = 0;
        UDPBroadcastByte(200, (BYTE)UDP_SEND_PORT);

        for(i = 0; i < 8; i++){         // REPORT ANALOG INPUTS' VALUES

            if(Analog[i] >= 32){        // CONVERT TO TWO BYTES, EACH OF 5 BITS
               // Delayms(5);
                Abyte1 = (int)Analog[i] / 32;
                Abyte0 = Analog[i] - (32 * Abyte1);

                UDPBroadcastByte(Abyte1,(BYTE) UDP_SEND_PORT);
                //Delayms(5);
                UDPBroadcastByte(Abyte0,(BYTE) UDP_SEND_PORT);
                ;

            }
            else{
                //Delayms(5);
                UDPBroadcastByte(0,(BYTE) UDP_SEND_PORT);
                //Delayms(5);
                UDPBroadcastByte(Analog[i], (BYTE)UDP_SEND_PORT);

            }
        }

    #endif
    }


    /********************************************************************
     * Function:        InitializePinguino(void)
     * Input:           None
     * Output:          None
     * Overview:        This function initializes the states of the
     *                  pins used in the hardware for this PinguinControl
     *                  application. It sets all pins as outputs and low.
     * Note:            None
     *******************************************************************/
    void InitializePinguino(void){

        LATDCLR = 0x9FF;        // Clear bits on PortD corresponding to D0-D7 in
                                // Pinguino PIC32 OTG
        LATBCLR = 0x3000;       // Clear bits on PortB corresponging to D8-D9 in
                                // Pinguino PIC32 OTG
        LATGCLR = 0x200;        // Clear bit 10 on PortG, D10 in Pinguino PIC32 OTG
        LATBCLR = 0xC1E;        // Clear bits on PortA correspondint to A0-A3 and
                                // A6-A7 in Pinguino PIC32 OTG.
        
        TRISDCLR = 0x9FF;       // Make pins Output.
        TRISBCLR = 0x3000;
        TRISGCLR = 0x200;
        TRISBCLR = 0xC1E;


    }