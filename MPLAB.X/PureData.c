    /*-----------------------------------------------------
   PC_Control.c
     * Version: 1.3

    Author:  I/O Roberto Becerra--<http://iobridger.wordpress.com/>
    Date: Wed Jul 24 07:48:44 2013
    Description:
    Source file part of the communication protocol used to
    control a Pinguino Board from Pure Data.
     By implementing the communication function on a Pinguino code, the
    communication through the USB CDC is started, and if the Pure Data patch
    <io_cdc_pinguino_control_INTERFACE_2_0.pd> is used, with all its
    dependencies, the PIC32 OTG can be controlled in real time, retrieving
    the states of Analog and Digital Pins, as well as setting them.
     This source file is optimized to be used with a Pinguino board,
    the PINGUINO PIC32 OTG. It nevertheless contains and implements
    definitions and functions that work with the PIC32MX440F256H.
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

    -----------------------------------------------------*/
#include <pwm.c>
#include <const.h>
#include <math.h>
#include <__cdc.c>

#ifndef PUREDATA
    #define PUREDATA
    #define PIC32_PINGUINO
    #define SET_DIGITAL_VALUE   0x90
    #define SET_ANALOG_VALUE    0xE0
    #define SET_PIN_MODE        0xF4

    static int dataLength;
    static unsigned char dataBuf[3];
    static unsigned char Abyte0, Abyte1;
    static int counter = 0;

    /********************************************************************
     * Function:        void pureDataInit(void)*
     * Input:           None
     * Output:          None
     * Overview:        Initializes the TRISx and PORTx registers to work with
     *                  the specifics of the platform, in this case
     *                  Pinguino Pic32 OTG.
     *                  It also initializes all registers needed for the
     *                  ADC operation.
     * Note:            None
     *******************************************************************/
    void pureDataInit(){

    #if defined(PIC32_PINGUINO) || defined(PIC32_PINGUINO_OTG)
         TRISB = 0x0000;                  // ALL ANALOG PINS AS DIGITAL OUTPUTS
         PORTB = 0x0000;                  // ALL SET TO LOW.

         TRISD = 0x0011;                  /*SETTING TO 0 - OUTPUT- THE PINS IN
                                         * CON4 AND 5 IN PINGUINO PIC32 OTG,
                                         * EXCEPT D2, WHICH IS THE BUTTON
                                         * BUTTON AS INPUT */

         PORTD = 0x0000;                // INITIALIZE AS LOW VALUES.

         TRISG = 0x0000;          // INITIALIZE LED1 AS OUTPUT AND LOW

         PORTG = 0x0000;

         //ADC MODULE CONFIGURATION BITS:
         //************************************
         AD1PCFGSET = 0xFFFF;    //PINS IN PINGUINO PIC32 OTG ANALOG PORT
                                 //SET AS DIGITAL

         AD1CHS = 0;             //NEGATIVE INPUT SELECTOR FOR MUX A AND MUX B
                                 //SET TO VR-.


         AD1CON1 = 0x00E0;      /*FORMAT AS 16 BIT INTEGER
                                 * AUTO CONVERT
                                 * ON = 0 - ADC IS OFF INITIALY
                                 * AUTO SAMPLE START BIT = 0*/

         AD1CON2 = 0x41C;       /*ENABLE (CSNA) SCAN OF INPUTS,
                                 *SMPI = 0x7 - 8 SEQUENCES BEFORE INTERRUPT
                                 * BUFM = 0 - ONE 16 WORD BUFFER
                                 * ALTS = 0 - DO NOT ALTERNATE
                                 *REFERENCE VOLTAGES SET TO  AVDD AND AVSS*/

         AD1CON3 = 0x8F00;      /* ADC INTERNAL CLOCK
                                 * AUTO SAMPLE TIME BITS - 0b01111
                                 * CONVERSION CLOCK = 0  */

         AD1CSSLSET = 0b111100011110;//ANALOG INPUTS TO BE SCANNED
         //END OF ADC MODULE CONFIGURATION BITS**********************************************

    #endif
    }

    /********************************************************************
     * Function:        int pureDataCommunicate(void)*
     * Input:           None
     * Output:          None
     * Overview:        Reads the CDC port, and stores the read values on a
     *                  buffer (inputData)it then routes the input data to
     *                  functions that take care of specific actions.
     *                  Note that the size of the arrays passed to this function
     *                  correspond to the number of Pins available in the
     *                  specific Pinguino. It also prints a 50 through the CDC
     *                  when the button is pressed.
     * Return:          int dataLength - the length of the array received.
     * Note:            None
     *******************************************************************/
    int pureDataCommunicate(unsigned char inputData[], unsigned char reportDg[],
                                    unsigned long int reportAn[]){


        if (((PORTDbits.RD4 == 0)&&(TRISDbits.TRISD4))
                ||((PORTDbits.RD0 == 0)&&(TRISDbits.TRISD0))){ //IS BUTTON
            CDCwrite(50);                                      //PRESSED?
        }


        dataLength = CDCgets(inputData);


        if ((inputData[0] >= SET_DIGITAL_VALUE) && (inputData[0] < 0xA0)){

            pureDataSetDigital(inputData);

        }

        else if ((inputData[0] >= SET_ANALOG_VALUE) && (inputData[0] < SET_PIN_MODE)){

            pureDataSetAnalog(inputData);

        }

        else if ((inputData[0] >= SET_PIN_MODE) && (inputData[0] < 0xFF)){// SEE MIDI MESSAGE FORMAT


            pureDataSetPin(inputData);

        }

        counter++;

        // REPORT ONLY ONCE EVERY 20 TIMES, TO AVOID SATURATION OF RECEIVER.
        if(counter == 20){
        pureDataSendState(reportDg, reportAn);
        counter = 0;
        }

        return dataLength;

    }

    /********************************************************************
     * Function:        void pureDataSendState(unsigned char Digital[],
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
    void pureDataSendState(unsigned char Digital[], unsigned long int Analog[]){

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
        CDCwrite(100);

        for(i = 0; i < 22; i++){        // REPORT DIGITAL PINS' VALUES
            //Delayms(5);
            CDCwrite(Digital[i]);

        }

        i = 0;
        CDCwrite(200);

        for(i = 0; i < 8; i++){         // REPORT ANALOG INPUTS' VALUES

            if(Analog[i] >= 32){        // CONVERT TO TWO BYTES, EACH OF 5 BITS
               // Delayms(5);
                Abyte1 = (int)Analog[i] / 32;
                Abyte0 = Analog[i] - (32 * Abyte1);

                CDCwrite(Abyte1);
                //Delayms(5);
                CDCwrite(Abyte0);
                ;

            }
            else{
                //Delayms(5);
                CDCwrite(0);
                //Delayms(5);
                CDCwrite(Analog[i]);

            }
        }

    #endif
    }


    /********************************************************************
     * Function:        void pureDataSetDigital(unsigned char data[])
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
    void pureDataSetDigital(unsigned char data[]){
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
     * Function:        void pureDataSetAnalog(unsigned char data[])
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
    void pureDataSetAnalog(unsigned char data[]){

        int channel = data[0] - SET_ANALOG_VALUE;
        double setpoint;

    #if defined(PIC32_PINGUINO) || defined(PIC32_PINGUINO_OTG)
        switch (channel){

            case 0:// PWM 0
                setpoint = (256*data[1])+data[2];
                analogwrite(D0, setpoint);
                break;

            case 1:// PWM 1
                setpoint = (256*data[1])+data[2];
                analogwrite(D1, setpoint);
                break;

            case 2:// PWM 2
                setpoint = (256*data[1])+data[2];
                analogwrite(D2, setpoint);
                break;

            case 3:// PWM 3
                setpoint = (256*data[1])+data[2];
                analogwrite(LED2, setpoint);
                break;

        }
    #endif
    }


     /********************************************************************
     * Function:        void pureDataSetPin(unsigned char data[])
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
    void pureDataSetPin(unsigned char data[]){

        int port = data[0] - SET_PIN_MODE;

    #if defined(PIC32_PINGUINO) || defined(PIC32_PINGUINO_OTG)
        switch (port){

            case 0:// PORT B
                switch (data[2]){
                    case 0:// DIGITAL OUTPUT

                        AD1PCFGSET = pow(2, data[1]);   //SET THIS PIN TO DIGITAL
                        TRISBCLR = pow(2,data[1]);      //SET TO OUTPUT
                        if(AD1PCFG == 0x0000){
                            AD1CON1CLR = 0x8000;         //TURN ADC OFF
                        }
                    break;
                    case 1:// DIGITAL INPUT

                        AD1PCFGSET = pow(2, data[1]);   //SET THIS PIN TO DIGITAL
                        TRISBSET = pow(2, data[1]);     //SET TO INPUT
                        if(AD1PCFG == 0x0000){
                            AD1CON1CLR = 0x8000;         //TURN ADC OFF
                        }
                    break;
                    case 2:// ANALOG INPUT

                        TRISBCLR = pow(2, data[1]);      //SET TO OUTPUT
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




#endif

