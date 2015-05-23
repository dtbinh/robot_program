#include <math.h>
#include "configuration.h"

const char posArray1[8] = {0x11, 0x01, 0x03, 0x02, 0x06, 0x04, 0x14, 0x10};
const char posArray2[8] = {0x09, 0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08};
const char posArray3[8] = {0x90, 0x10, 0x30, 0x20, 0x60, 0x40, 0xC0, 0x80};

const char motor1Array[15] = {15, 135, 5, 67, 105, 130, 148, 161, 172, 180, 187, 192, 197, 201, 205};
//incremental  = [2 1 0 0 0 0 0 0 0 0 0 0 0 0 0];

const char motor2_1Array[15] = {0xa2, 0xd1, 0xe0, 0xe8, 0xed, 0xf0, 0xf2, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf8, 0xf9, 0xf9};
const char motor2_2Array[15] = {0x3f, 0x1f, 0xbf, 0x8f, 0x3f, 0x5f, 0x9a, 0x47, 0x94, 0x9f, 0x79, 0x2f, 0xc9, 0x4d, 0xbf};


//char  counter = 0;
int counter1 = 0;
int counter2 = 0;
int counter3 = 0;

char motor1 = 0;
char motor2 = 0;
char motor3 = 0;
char tmp1   = 0;
char tmp2   = 0;
char tmp3   = 0;

char motor1run = 0;
char motor2run = 0;
char motor3run = 0;

char incremental1 = 0;
char incremental3 = 0;
int direction1 = 0;
int direction2 = 0;
int direction3 = 0;
int loop1 = 0;
int loop3 = 0;


char counter_uart = 0;

char uart_data[15] = {0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D};


char init_received = 0;

char tmp_data=0;
char tmp_data2 = 0;

void interrupt global_interrupt(){          //single interrupt vector to handle all of ISR's

    GIE = 0 ;//Global interrupt disable in ISR
    CREN = 1; // seri port kapand?ysa acal?m

    if(RCIF){
        

        if(FERR == 0 && OERR == 0){
            tmp_data = RCREG;
            
            if(tmp_data == 0x55){
                counter_uart = 0;
                 GIE = 1;
                 return;
            }

                uart_data[counter_uart] = tmp_data;
                counter_uart += 1;
                if(counter_uart == 15){
                    counter_uart = 0;
                }
      
       
        
        }
        else{
             CREN = 0;
            tmp_data = RCREG;  // bos okuma yapal?m
             counter_uart = 0;
        }
        GIE = 1;
        return;

    }
   
    //Timer0 interrupt
    if(T0IF){
        T0IF = 0; //clear interrupt flag
 

        if(loop1 == 2){
            loop1 = 1;
            TMR0 = 0;
         }
        else if(loop1 == 1){
            loop1 = 0;
            TMR0 = 0;
        }
        else{
            TMR0 = motor1Array[motor1];
            loop1 = incremental1;
            counter1 += direction1;
           if(counter1 == 8)
            counter1 = 0;
          else if(counter1 < 0)
            counter1 = 8;
            
           PORTA = posArray1[counter1] & motor1run;
       
               
         }
        GIE = 1 ;//Global interrupt enable in ISR
  return;
      
    }

       //Timer1 interrupt
    if(T1IF){
        T1IF = 0; //clear interrupt flag
       
        counter2 += direction2;

        if(counter2 == 8)
            counter2 = 0;
        else if(counter2 < 0)
            counter2 = 8;

        TMR1H = motor2_1Array[motor2];
        TMR1L = motor2_2Array[motor2];
     
       
    }
 
PORTC = posArray2[counter2] | posArray3[counter3];


  GIE = 1 ;//Global interrupt enable in ISR
  return;
   
      



   
}

int main(void){
     //Internal RC osc operating at 8Mhz
      OSCCON  = 0x00;
      OSCCON |= 0b01110001;
      OSCTUNE = 0x00; //8Mhz calibrated frequency
    //Internal RC osc operating at 8MHz

      //Timer0 clock=Fosc/4, prescaler = 1/32;
      OPTION_REG &= 0b11010000;
      OPTION_REG |= 0b00000100;
      TMR0 = 0x00;
      T0IE = 1;
      //Timer0 clock=Fosc/4, prescaler = 1/256;

     
      //Timer1 clock=Fosc/4, prescaler = 1/1;
      TMR1GE = 0;
      T1CONbits.T1CKPS0 = 0;
      T1CONbits.T1CKPS1 = 0;
      T1OSCEN = 0;
      TMR1CS = 0;
      TMR1ON = 1;
      TMR1H = 0x00;
      TMR1H = 0x00;
      TMR1IE = 1;
      //Timer1 clock=Fosc/4, prescaler = 1/1;

      //UART 9600 8 bit asenkron konfig
      BRG16 = 0;
      BRGH = 0;
      SPBRGH = 0;
      SPBRG = 0x0C; //9600
      SYNC = 0;
      SPEN = 1;
      RCIE = 1;
      CREN = 1;
      //UART 9600 8 bit asenkron konfig
       
      
    
      //Timer2 clock=Fosc/4, prescaler = 1/16 , postscaler = 1/16;
     // T2CONbits.T2CKPS0= 1;
      //T2CONbits.T2CKPS1= 1;
      //T2CONbits.T2OUTPS0 = 1;
      //T2CONbits.T2OUTPS1 = 1;
      //T2CONbits.T2OUTPS2 = 1;
      //T2CONbits.T2OUTPS3 = 1;
      //TMR2ON = 1;
      //TMR2IE = 1;
    //Timer2 clock=Fosc/4, prescaler = 1/16 , postscaler = 1/16;


      //PORTA yI konfigure edelim
      TRISA = 0x00;
      TRISB = 0b00100000;
      TRISC = 0x00;
      ANSEL = 0x00;
      ANSELH = 0x00;
      //PORTA y? konfigure edelim

      PORTA = 0x00;
      PORTB = 0x00;
      PORTC = 0x00;
      PEIE = 1;
      GIE  = 1;

    while(1){


        tmp1 =  uart_data[14];

  if(tmp1 == 75)
      motor1run = 0x00;
  else
      motor1run = 0xFF;

        if(tmp1 >=60){
            tmp1 = tmp1 - 15;
            direction1 = -1;
        }
        else
            direction1 = 1;

      

        
        tmp1 = tmp1 - 45;
        if(tmp1 == 0)
            incremental1 = 2;
        else if(tmp1 == 1)
            incremental1 = 1;
        else
            incremental1 = 0;


        motor1 = tmp1;
       
       
           
      tmp2 =  uart_data[13];
      if(tmp2 >=60){
            tmp2 = tmp2 - 15;
            direction2 = -1;
        }
       else
         direction2 = 1;
       
        tmp2 = tmp2 - 45;
        motor2 =  tmp2;
      
        
    }
}







/*

void TIMER1_ISR(void){
  Change_to_New_Baud = true;         //Change to new baud in Sinus_Generator()

  //reset Timer1 registers
  TMR1H = 0x00;
  TMR1L = 0x00;

  PIR1 &= ~0x04; //Clear Timer1 interrupt flag

  Systick_Counter += 1;
  if(Systick_Counter > BATTERY_MEAS_EVERY_MILISECOND){
    Systick_Counter = 0;
    ADCON0 |= 0b00000010;         //If period overruns for battery reading start the ADC conversion
  }
}



*/


/*

#include "configuration.h"
#include "utility.h"
#include <math.h>
#include "ax25.h"
#include "audio_tone.h"
#include "isr.h"

//#define Debug_Modem_Packet                 //uncomment this line to debug the data encoded with AX25

#define I2C_ADDRESS                           0x60

extern bool PTT_OFF;                         //PTT_OFF flag, not to pull Ptt_Off function within an interrupt
extern bool MODEM_TRANSMITTING;              //flag to check whether modem_packet is fully transmitted

#ifdef Debug_Modem_Packet
extern uint8_t modem_packet[MODEM_MAX_PACKET];
uint8_t k = 0;
#endif


void interrupt global_interrupt(){          //single interrupt vector to handle all of ISR's

    INTCON &= ~0x80;                        //Global interrupt disable in ISR

    //ADC interrupt
    if(ADIF){
      ADC_ISR();
      return;
    }
    //ADC interrupt


    //Timer1 interrupt
    if(PIR1 & 0x04){
      TIMER1_ISR();
      return;
    }
    //Timer1 interrupt


    //Timer0 interrupt
    if(INTCON & 0x04){
      TIMER0_ISR();
      return;
    }
    //Timer0 interrupt


    //I2C interrupt
    if(SSP1IF){
      I2C_ISR();
      return;
    }
    //I2C interrupt

    INTCON |= 0x80;                       //Global interrupt enabled again
}

void System_Start(void){

    //Watchdog timer configuration for 128 seconds
      WDTPS4 = 1;
      WDTPS3 = 0;
      WDTPS2 = 0;
      WDTPS1 = 0;
      WDTPS0 = 1;
    //Watchdog timer configuration for 128 seconds

    //Internal RC osc with 4xPLL operating at 32MHz
      OSCCON  = 0x00;
      OSCCON |= 0b11110000;
      OSCTUNE = 0x00;
    //Internal RC osc with 4xPLL operating at 32MHz

    //Configurations for Timer0
      TMR0CS = 0;                 //Internal clock source (Fosc/4)
      PSA    = 1;                 //Do not use Prescaler
    //Configurations for Timer0

    //Configurations for Timer1
      TMR1ON = 1;                 //Timer1 always count
      TMR1GE = 0;

      TMR1CS1 = 0;                //Fosc/4
      TMR1CS0 = 0;

      T1CKPS1 = 1;                //1/8 prescaler
      T1CKPS0 = 1;

      CCP1M3 = 1;                 //Software interrupt on compare event
      CCP1M2 = 0;
      CCP1M1 = 1;
      CCP1M1 = 0;
    //Configurations for Timer1

    //Configurations for Dac0
      DACOE   = 1;
      DACPSS1 = 0;
      DACPSS0 = 0;
      DACNSS  = 0;
    //Configurations for Dac0


    //Configurations for Adc1
      ANSA1   = 1;              //RA1 analog input
      ADCON0 &= 0b10000011;
      ADCON0 |= 0b00000100;     //AN1 channel select
      ADNREF  = 0;              //Vref- = GND
      ADPREF1 = 0;
      ADPREF0 = 0;              //Vref+ = Vdd
      ADCS2   = 1;
      ADCS1   = 1;
      ADCS0   = 0;              //Fosc/64 for conversion clock
      ADFM    = 1;              //Output on right hand side
    //Configurations for Adc1


    //Reset Interrupt Flags
      TMR0IF = 0;
      TMR1IF = 0;
      CCP1IF = 0;
      ADIF   = 0;
    //Reset Interrupt Flags

    //Configurations for I2C
      SSPEN   = 1;               //Pin configurations for the I2C peripheral
      SSPM3   = 0;               //Slave operation with 7 bit adress, S/P interrupt disabled
      SSPM2   = 1;
      SSPM1   = 1;
      SSPM0   = 0;
      GCEN    = 0;               //General Call disabled
      ACKDT   = 0;               //ACK on every receive
      SEN     = 1;               //Clock stretch enabled
      SSP1ADD = I2C_ADDRESS;     //7 bit adress
      SSPMSK |= 0b11111110;      //Address check for all bits
      PCIE    = 0;               //P interrupt disable
      SCIE    = 0;               //S interrupt disable
      BOEN    = 0;               //SSP1BUF is updated ignoring SSPOV
      AHEN    = 0;               //Adress hold disable
      DHEN    = 0;               //Data hold disable
      SBCDE   = 0;               //Collision Detect interrupt disable
      SSP1IF  = 0;               //Clear interrupt flag
      SSP1IE  = 1;               //I2C interrupt enable
    //Configurations for I2C

    //Global Interrupt ve Peripheral Interrupt Enable
      INTCON |= 0xC0;
}


int main(void) {
    Gpio_Config();                   //Gpio configuration
    System_Start();
    while (!(OSCSTAT & (0x01))){}    //Wait for HFIOFS Osc. stable bit


    Timer1_Start();                  //Timer1 with 833 us period

    Dac0_Start_Hold();               //Start Dac output and make the output Vdd/2

    Adc1_Start();                    //Just configure Adc1 peripheral, conversion will be started within Timer1 ISR

    ADF7021_CHIP_POWER_DOWN;         //CE pin low , to reset the configuration on ADF7012
    Delay_ms(10);
    ADF7021_LOAD_REGISTER_DISABLE;   //LE pin high, to disable loading registers
    Delay_ms(10);
    ADF7021_CHIP_POWER_UP;           //CE pin high, to enable ADF7012
    Delay_ms(10);

    Delay_ms(200);


    s_address beacon_address[2] = {{"CUBEYY", 5},{"CUBEXX", 6}};  //Source and destination adresses with callsigns

    Ax25_Send_Header(beacon_address,2);                           //Header with 2 adresses
    Ax25_Send_String("HELLO");                                    //Send string
    Ax25_Send_Footer();                                           //Send Footer

    Modem_Setup();                                                //Set modem reset configurations to ADF7012
    ADF7012_SET_DATA_PIN;                                         //Data invert = ON , set data pin to stop transmit
    Delay_ms(100);
    Adf_Lock();                                                   //Try to achieve a good PLL lock, otherwise use default vco configuration
    Delay_ms(100);


     while(1){
          if(PTT_OFF){
		  Ptt_Off();                                      //Turn of PTT
		  PTT_OFF  = false;
	  }
          Delay_ms(2000);

	  Modem_Flush_Frame();                                   //Transmit modem_packet[]
          while(MODEM_TRANSMITTING);
	  Delay_ms(2000);



#ifdef Debug_Modem_Packet

          for (k=0; k< MODEM_MAX_PACKET; k++){
          Spi_Byte_Send(modem_packet[k]);
          }
          Delay_ms(3000);
#endif

          CLRWDT();                                            //Clear Watchdog timer
         }
    return (EXIT_SUCCESS);
}

// -----------------------------------------------------------------------


*/