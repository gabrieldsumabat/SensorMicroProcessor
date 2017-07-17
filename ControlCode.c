#include <hidef.h>       //These libraries contain multiple functions referenced in this code                  
#include "derivative.h"                    
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <mc9s12c32.h> /* d e r i v a t i v e inf o rma t i on */
#include "SCI.h"   //-Need to find this library

#pragma LINK_INFO DERIVATIVE "mc9s12c32"   

//Define given variables with set values, the LINE values are references to specific locations on the LCD.
#define DELAY 200                          
#define DELAY2 100                          
#define ONE_MILLISEC_FOR_TSCR2_1US   1000  
#define ONE_MILLISEC_FOR_TSCR2_333NS 3000  
#define PI 3.141592
#define LINE1 0x02 
#define LINE2 0x90
#define LINE3 0xA0
#define RDRF  0x20   // Receive Data Register Full Bit
#define TDRE  0x80   // Transmit Data Register Empty Bit

//Variables are used to match the link posted above in relation to a cartesian plane calculation using arctan.
int cnt = 0;
int xpul = 0, xrise=0, xfall=0, x_time=0, x_period=1;
int ypul = 0, yrise=0, yfall=0, y_time=0, y_period=1;
float ax, ay, ang;

//These functions were drawn from the 604 provided code snippets
void port_m_init (void);                   
void toggleLED(void);
void Timer_Init(void);
void Timer_mwait(unsigned short msec );
void spi_lcd_init(void);
void spi_write_lcd_command(char data);
void lcd_init(void);
void lcd_cursor_off(void);
void lcd_go_home(void);
void SPI_LCD_OutString(char *pt);
void init_out(void);
void ATD_Init(void);
void port_t_init(void);
void OutCRLF(void){
  SCI_OutChar(CR);
  SCI_OutChar(LF);
  PTT ^= 0x01;          // toggle bit 0
}

//Main Operation Loop
void main(void) {

//The 16 value comes from the number of bits included in each set
char axis[16];
char angle[16];
char string [20];
int counter=0;
char conv[20];
int overflow=0;
float data=0;
float mass=0;
char  *msg1 = "Gabriel Sumabat";
char  *msg2 = "Joseph Alacron";
char *test = "Created by       ";
short tcounter;
short matcounter;
unsigned short n;
//All functions are intialized along with interrupts being enabled. Utilizing interrupt functions are key in this lab.
PTT_PTT6=0;
PTT_PTT2=0;
PTT_PTT1=0;

EnableInterrupts;
Timer_Init();
ATD_Init();
port_m_init();
init_out();
spi_lcd_init();
port_t_init();
lcd_init();
lcd_go_home();

SPI_LCD_OutString("Press the Button");
for(;;){
      
      Timer_mwait(200);
         
      if ((PTT_PTT7==0) && (PTT_PTT6==0) && (PTT_PTT2==0) && (PTT_PTT1==0)){
         PTT_PTT6=1; 
         PTT_PTT7=1;
         
            lcd_go_home();
	          SPI_LCD_OutString(test);
	          spi_write_lcd_command(LINE2);
	          SPI_LCD_OutString(msg1);
	          spi_write_lcd_command(LINE3);
	          SPI_LCD_OutString(msg2);
	          Timer_mwait(1000);
	          ;
            
                
      } else if ((PTT_PTT7==0) && (PTT_PTT6==1) && (PTT_PTT2==0) && (PTT_PTT1==0)){
         PTT_PTT6=0;
         PTT_PTT2=1;
         PTT_PTT7=1;
        
                          Timer_mwait(500);
      
                          data=ATDDR0;
     
                          spi_write_lcd_command(0x01);
                          lcd_go_home();
                          SPI_LCD_OutString("Data:  ");
                          sprintf(conv, "%f", data);
                          SPI_LCD_OutString(conv);
      
                          mass = data*0.0035;
            
                          if (mass>130)      // Compensation for non-linearity
                          mass = mass-10;  //
                          else if (mass>110) //
                          mass = mass-6;  //
      
                          spi_write_lcd_command(0x90);
                          SPI_LCD_OutString("Mass:  ");
                          sprintf(conv, "%.1f", mass);
                          SPI_LCD_OutString(conv);
      
                     Timer_mwait(100);
                     tcounter++;
                     
      } else if ((PTT_PTT7==1) && (PTT_PTT6==0) && (PTT_PTT2==1) && (PTT_PTT1==0)){
         PTT_PTT6=0;
         PTT_PTT2=1;
         PTT_PTT7=1;
        
                          Timer_mwait(500);
      
                          data=ATDDR0;
     
                          spi_write_lcd_command(0x01);
                          lcd_go_home();
                          SPI_LCD_OutString("Data:  ");
                          sprintf(conv, "%f", data);
                          SPI_LCD_OutString(conv);
      
                          mass = data*0.0035;
            
                          if (mass>130)      // Compensation for non-linearity
                          mass = mass-10;  //
                          else if (mass>110) //
                          mass = mass-6;  //
      
                          spi_write_lcd_command(0x90);
                          SPI_LCD_OutString("Mass:  ");
                          sprintf(conv, "%.1f", mass);
                          SPI_LCD_OutString(conv);
      
                          Timer_mwait(100);
                          tcounter=0;
                     
                              	 
      } else if ((PTT_PTT7==0) && (PTT_PTT6==0) && (PTT_PTT2==1) && (PTT_PTT1==0)){
         PTT_PTT2=0;
         PTT_PTT1=1;
         PTT_PTT7=1;
                
                        counter++;
                        Timer_mwait(DELAY); // 0.5 secwait
                        
                        data=ATDDR0;
                        mass = data*0.0035;
            
                        if (mass>130)      // Compensation for non-linearity
                        mass = mass-10;  //
                        else if (mass>110) //
                        mass = mass-6;  //
                        
                        ax =  (((float)(x_time)-0.5*(x_period)))*8;  //Calcuates the acceleration of x using pulse width
                      	ay =  (((float)(y_time)-0.5*(y_period)))*8;  //Calculates the acceleration of y using the pulse width
            	          ang = 90-atan(ay/ax)*(180/PI);               //Using trig ratios, finds the angle using opp and adj through arctan
            	
                      	lcd_go_home();
	                      SPI_LCD_OutString("Safe Parameters ");
	                      
            	          if (ang <= 170 && ang >= 10) { spi_write_lcd_command(LINE2);
	                                                     SPI_LCD_OutString("Angle in SAFE");} 
	                      else {                         spi_write_lcd_command(LINE2);
	                                                     SPI_LCD_OutString("Angle in DANGER");}
	                      
	                      if (mass <= 360 && mass >= 0) { spi_write_lcd_command(LINE3);
	                                                     SPI_LCD_OutString("Mass in SAFE");} 
	                      else {                         spi_write_lcd_command(LINE3);
	                                                     SPI_LCD_OutString("Mass in DANGER");}
                        Timer_mwait(100);
                        tcounter++;
                       
       } else if ((PTT_PTT7==1) && (PTT_PTT6==0) && (PTT_PTT2==0) && (PTT_PTT1==1)){
         PTT_PTT2=0;
         PTT_PTT1=1;
         PTT_PTT7=1;
                
                        counter++;
                        Timer_mwait(DELAY); // 0.5 secwait
                        
                        data=ATDDR0;
                        mass = data*0.0035;
            
                        if (mass>130)      // Compensation for non-linearity
                        mass = mass-10;  //
                        else if (mass>110) //
                        mass = mass-6;  //
                        
                        ax =  (((float)(x_time)-0.5*(x_period)))*8;  //Calcuates the acceleration of x using pulse width
                      	ay =  (((float)(y_time)-0.5*(y_period)))*8;  //Calculates the acceleration of y using the pulse width
            	          ang = 90-atan(ay/ax)*(180/PI);               //Using trig ratios, finds the angle using opp and adj through arctan
            	
                      	lcd_go_home();
	                      SPI_LCD_OutString("Safe Parameters ");
	                      
            	          if (ang <= 170 && ang >= 10) { spi_write_lcd_command(LINE2);
	                                                     SPI_LCD_OutString("Angle in SAFE");} 
	                      else {                         spi_write_lcd_command(LINE2);
	                                                     SPI_LCD_OutString("Angle in DANGER");}
	                      
	                      if (mass <= 360 && mass >= 0) { spi_write_lcd_command(LINE3);
	                                                     SPI_LCD_OutString("Mass in SAFE");} 
	                      else {                         spi_write_lcd_command(LINE3);
	                                                     SPI_LCD_OutString("Mass in DANGER");}
                        Timer_mwait(100);
                        tcounter=0;
         
      } else if ((PTT_PTT7==0) && (PTT_PTT6==0) && (PTT_PTT2==0) && (PTT_PTT1==1)){
         PTT_PTT6=1;
         PTT_PTT1=0;
         PTT_PTT7=1;

          
            	spi_write_lcd_command(0x01);
            	ax =  (((float)(x_time)-0.5*(x_period)))*8;  //Calcuates the acceleration of x using pulse width
            	ay =  (((float)(y_time)-0.5*(y_period)))*8;  //Calculates the acceleration of y using the pulse width
            	ang = 90-atan(ay/ax)*(180/PI);               //Using trig ratios, finds the angle using opp and adj through arctan
            	
              lcd_go_home();
              SPI_LCD_OutString("Angle: ");
              sprintf(angle, "%.f", (float)ang);
              SPI_LCD_OutString(angle);
	  
              spi_write_lcd_command(LINE2);
           	  SPI_LCD_OutString("X-Axis:  ");
              sprintf(axis, "%d", x_time);
              SPI_LCD_OutString(axis);

              spi_write_lcd_command(LINE3);
	            SPI_LCD_OutString("Y-Axis:  ");
              sprintf(axis, "%d", y_time);
              SPI_LCD_OutString(axis);
 
               Timer_mwait(100);
               tcounter++;
               
      } else if ((PTT_PTT7==1) && (PTT_PTT6==1) && (PTT_PTT2==0) && (PTT_PTT1==0)){
         PTT_PTT6=1;
         PTT_PTT1=0;
         PTT_PTT7=1;

          
            	spi_write_lcd_command(0x01);
            	ax =  (((float)(x_time)-0.5*(x_period)))*8;  //Calcuates the acceleration of x using pulse width
            	ay =  (((float)(y_time)-0.5*(y_period)))*8;  //Calculates the acceleration of y using the pulse width
            	ang = 90-atan(ay/ax)*(180/PI);               //Using trig ratios, finds the angle using opp and adj through arctan
            	
              lcd_go_home();
              SPI_LCD_OutString("Angle: ");
              sprintf(angle, "%.f", (float)ang);
              SPI_LCD_OutString(angle);
	  
              spi_write_lcd_command(LINE2);
           	  SPI_LCD_OutString("X-Axis:  ");
              sprintf(axis, "%d", x_time);
              SPI_LCD_OutString(axis);

              spi_write_lcd_command(LINE3);
	            SPI_LCD_OutString("Y-Axis:  ");
              sprintf(axis, "%d", y_time);
              SPI_LCD_OutString(axis);
 
               
               Timer_mwait(100);
               tcounter=0;
                        
    } else if (tcounter==10) {   //Matlab Serial Communication
         
              lcd_go_home();
              SPI_LCD_OutString("Activate RUN ->");
              spi_write_lcd_command(LINE2);
           	  SPI_LCD_OutString("MATLAB COMM ACTIVE");
              spi_write_lcd_command(LINE3);
	            SPI_LCD_OutString("HOLD TO EXIT");
              Timer_mwait(100);
         
         DDRT |= 0x03; //PortT bits 1,0 are output
         SCI_Init (19200);
         SCI_OutString ("TechArts_S12C32");
         OutCRLF();
            //while (PTT_PTT7==1){
            while (matcounter<100) {
                PTT ^= 0x02;          // toggle bit 1
                SCI_OutString("InString: "); 
                SCI_InString(string,19);   
                SCI_OutString(" OutString="); SCI_OutString(string); OutCRLF();
     
                SCI_OutString("InUDec: ");  n=SCI_InUDec(); 
                SCI_OutString(" OutUDec="); SCI_OutUDec(n); OutCRLF();
      
                SCI_OutString("InUHex: ");  n=SCI_InUHex(); 
                SCI_OutString(" OutUHex="); SCI_OutUHex(n); OutCRLF();
                if (PTT_PTT7==0) {
                  matcounter++;
                }  else if (PTT_PTT7==1) {
                  matcounter=0;
                }
            }    
      }
  }
}

/* MATLAB CODE  -------------------------------------------------------

a = serial('COM3');
a.BaudRate = 19200;
fopen(a);

x = 1:60;
y = 1:60;
i = 1;
j = 1;
k = 1;


for i = 1:120
   data = fscanf(a, '%s')
   if(data(1) == 'T')
        if (length(data) == 3)
            x(j) = str2num(data(2))*10 + str2num(data(3));
        else
            x(j) = str2num(data(2));
        end
        j = j + 1;
   elseif (data(1) == 'D')
        if (length(data) == 3)
            y(k) = str2num(data(2))*10 + str2num(data(3));
        elseif (length(data) == 4)
            y(k) = str2num(data(4)) + str2num(data(2))*100 + str2num(data(3))*10;
        else
            y(k) = str2num(data(2));
        end
        k = k + 1;
   end

end

fclose(a)
plot(x, y)

*/


/* ---------------------------------------
* Input/Output commands. Taken from the given code snippets.
* ---------------------------------------- */

/*------------ATD_Init---------------*/

void ATD_Init(void) {
	
	
  ATDCTL2= 0x80;
  ATDCTL3= 0x08;
  ATDCTL4= 0x05;
  ATDCTL5= 0x20;

	
}
          //---------------------- LED PORT M INIT --------------------------

void port_t_init (void){
//Inputs
DDRT_DDRT7 = 0;
//Outputs
DDRT_DDRT6 = 1;
DDRT_DDRT2 = 1;
DDRT_DDRT1 = 1;
}

void port_m_init (void) { 
/* Port M is set up to flash the Esduino's built-in LED*/  
asm{ 
MOVB #32,DDRM 
CLR PTM 
} 
} 
          //---------------------- LED FLASH --------------------------


void toggleLED(void){
//toggle onboard LED 
  if (PTM_PTM5 == 1) 
    PTM_PTM5 = 0 ;
  else PTM_PTM5 = 1;
}

          //---------------------- A/D PIN INIT --------------------------

void init_out(void)
{

    DDRT = 0x00; // all ports input
    PTT = 0;     // input selected
    TIOS_IOS2 = 0;    //EN INPUT PT2
    TIOS_IOS1 = 0;    //EN INPUT PT1 
    TCTL4 = 0x3C;  // rising and falling edge IC2 and IC1
    TFLG1 = 0xFF;  // clear all flags    
    TIE = 0x06;    // arm channel 1,2
}


/* ---------------------------------------
* Timers
* ---------------------------------------- */

/*------------Timer_Init---------------*/

void Timer_Init(void) {
	
	
	TSCR1 = 0x80;	// Enable TCNT
	TSCR2 = 0x04;	// divide by TCNT pre scale for Esduino. TCNT at 0.333us
	PACTL = 0;	  // timer pre scale used for TCNT
	TIE   = 0x02;
	TIOS  = 0x00;
	TCTL4 = 0x3C;
	TFLG1 = 0xFF;

	
}


          //---------------------- Timer WAIT --------------------------

void Timer_mwait ( unsigned short msec ){  

         unsigned short startTime ;

          for(; msec>0; msec--){
              startTime = TCNT;

// while ((TCNT-startTime ) <= ONE MILLISEC FOR TSCR2 1US){}  
    while ((TCNT-startTime ) <= ONE_MILLISEC_FOR_TSCR2_333NS){}

}
} 

/* ---------------------------------------
* LCD
* ---------------------------------------- */

void spi_lcd_init(void) {
  
         // Configure Slave Select/Output Enable . Note that it is active low .
         DDRT_DDRT3=1; // Set PortT's PT3 as output.

         // Set PortM, MISO is input(0) and SCK as output(1)
         DDRM_DDRM2=0; // MISO (Master In Slave Out) is an input
         DDRM_DDRM5=1; // SCK (Cloc k) is an output

         // Set the SPI control registers
         SPICR1=0x50 ; // SPI Control Register1, bit7: Interrupt Enable, bit4: MSTR
         SPICR2=0x00 ; // SPI Control Register2, bit[7:0] - Disabled

         /* Set the SPI baud rate register
          * For a Baud rate of 375 kHz and an internal bus clock of 24MHz
          * we need to set the divisor to 64 (as per the equation on
          * MC9s12C-Family manual R.01.23, pg. 419.)
          */
         SPIBR=0x05 ; // divisor is 2^(0x05+1)=64.; BR=24000000/64.
}

          //---------------------- LCD COMMAND --------------------------

 
void spi_write_lcd_command(char dataout){
  
        int tmp ;
        
        /* 8-bit data length, 1 line, instruction table 1(see datasheet)*/
        asm{
          
              BCLR PTT,#$08
        }
        while ((SPISR&0x20)==0); /*wait until SPTEF flag set*/
        SPIDR = dataout; /*write data to device*/
        while ((SPISR&0x80)==0); /*wait until transmission complete*/
        tmp = SPIDR; /*Read data register to clear SPIF*/
        asm{
          
              BSET PTT,#$08
        }
}

          //---------------------- LCD DATA --------------------------

void spi_write_lcd_data(char dataout){
  
        int tmp;

        /* 8-bit data length, 1 line, instruction table1(see data sheet) */
        asm{
          
               BSET PTT,#$08
        }
        while ((SPISR&0x20)==0); /*wait until SPTEF flag set */
        SPIDR = dataout; /*write data to device*/
        while ((SPISR&0x80)==0); /*wait until transmission complete */
        tmp = SPIDR; /*Read data register to clear SPIF */
        asm{
               BCLR PTT,#$08
        }
}

          //---------------------- LCD INIT --------------------------
     
void lcd_init(void){
  
      /* Steps 1 - 6 need to be customized for each EA Dog display */
      /* Step 1 */
      Timer_mwait(50); /* need 40msec + after power-up */
      spi_write_lcd_command(0x39); //

      /* Step 2 */
      Timer_mwait(50); /* This can be 2msec or less */
      spi_write_lcd_command(0x1D); //

      /* Step 3 */
      Timer_mwait(50); /* This can be 2msec or less */
      spi_write_lcd_command(0x50); //

      /* Step 4 */
      Timer_mwait(50); /* This can be 2msec or less */
      spi_write_lcd_command(0x6C); //

      /* Step 5 */
      Timer_mwait(200); /* This must be 200msec or more */
      spi_write_lcd_command(0x7C); //

      /* Step 6 */
      Timer_mwait(50); /* This can be 2msec or less */
      spi_write_lcd_command(0x38); //

      /* Steps 7 - 9 are common between all EA DOG displays */

      /* Step 7 (Display On) */
      Timer_mwait(50); /* This can be 2msec or less */
      spi_write_lcd_command(0x0F);

      /* Step 8 (Clear Display) */
      Timer_mwait(50); /* This can be 2msec or less */
      spi_write_lcd_command(0x01);

      /* Step 9 (Entry Mode Set ) */
      Timer_mwait(200); /* This can be 2msec or less */
      spi_write_lcd_command(0x06);

      /* Steps 10+ are not required */
      Timer_mwait(2); // DEBUG
      spi_write_lcd_command(0x33); // DEBUG

}
  
          //---------------------- LCD CURSOR --------------------------
  
void lcd_cursor_off(void){
  
     Timer_mwait(50); /* This can be 2msec or less */
     spi_write_lcd_command(0x0C); /* command */
}

          //---------------------- LCD GO HOME --------------------------
   
void lcd_go_home(void){
  
    Timer_mwait(2); /* This can be 2msec or less */
    spi_write_lcd_command(0x02); /* command */
}

          //---------------------- LCD STRING OUT --------------------------

void SPI_LCD_OutString(char *pt){
  
        while (*pt){
          
                  spi_write_lcd_data(*pt);
                  pt++;
}
}


//These interrupts run in the background to allow consistent updating of the x and y positions using the pulses.
//The interrupts are edge based, Either a falling edge (high to low) or a rising edge (low to high). This requires a pull up or pull down resistor.

interrupt 9 void Channel1(void){  //PTT_TT1 
   if(ypul == 0){  //rising edge
      y_period = TC1 - yrise;   //Gets the period of y using the total time found minus the risen time.
      yrise = TC1;              //Sets the future rise time as the total elapsed time.
      //capture falling
      TCTL4_EDG1A = 1; //First edge is high       
      TCTL4_EDG1B = 0; //Second edge is low
  }else{ //falling edge
      yfall = TC1;     //Set fall time to total elapsed time
      //capture rising
      TCTL4_EDG1A = 0; //First edge is low       
      TCTL4_EDG1B = 1; //Second edge is high
      y_time = yfall-yrise;  //Calcualtes the difference between the two (50% is the baseline)
  }
  ypul = ~ypul;  //(not)ypul, inverts the current bit set. 
  TFLG1 = 0x02;  // clear channel 1 flag.
  
}

interrupt 10 void Channel2(void){  //PTT_TT2 

  if(xpul == 0){  //rising edge
      x_period = TC2 - xrise;   //Gets the period of x using the total time found minus the risen time.
      xrise = TC2;              //Sets the future rise time as the total elapsed time.
      //capture falling
      TCTL4_EDG2A = 1;         //First edge is high
      TCTL4_EDG2B = 0;         //Second edge is low
  }else{ //falling edge
      xfall = TC2;           //Set fall time to total elapsed time
      //capture rising
      TCTL4_EDG2A = 0;    //First edge is low      
      TCTL4_EDG2B = 1;    //Second edge is High  
      x_time = xfall-xrise; //Calcualtes the difference between the two (50% is the baseline)
  }
  xpul = ~xpul;           //(not)xpul, inverts the current bit set.
  TFLG1 = 0x04;           // clear channel 2 flag.
  
}
	