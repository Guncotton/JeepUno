#include <U8x8lib.h>
#include <Wire.h>
#include "thermistor.h"

U8X8_SSD1306_128X64_NONAME_HW_I2C OLED(U8X8_PIN_NONE);

//#############
//# Constants #
//#############

#define FAN_ON      185          // Fan turn on temperature.
#define FAN_DELTA   20           // Fan turn off delta T. 
#define NPULSES     16


#define FAN         12           // Pin assigned to Fan relay. 
#define TACH        11           // Pin assigned to Tachometer meter. 
#define TEST        7            // Pin assigned to testing.

// which analog pin to connect
#define COOLANT     A0
#define PS_OIL      A1
         
//######################
//# Thermistor Circuit #
//######################
#define THERMISTOR_BETA   4005        // Constant used in Beta parameter equation estimating R of NTC thermistor.
#define THERMISTOR_RO     2710        // Thermistor resistance @ 25C or 298.15K.
#define BIAS_RESISTOR     1000        // Value of the bias resistor in the thermistor circuit, in Ohms.

//#############
//# Variables #
//#############

volatile unsigned long LastTimerValue = 0;                       // Stores the last time we measured a pulse so we can calculate the period.
volatile unsigned long TimerPeriod = 0;                         // Stores the period between pulses in microseconds.
volatile unsigned long AvgTimerPeriod = 0;                       // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.
volatile unsigned long PeriodSum = 0;                                        // Stores the summation of all the periods to do the average.
volatile unsigned long Time;

//unsigned long RPM;                                              // Raw RPM without any processing.
volatile unsigned int PulseCount = 1;                                    // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.

volatile bool UpdateDisplay = false;                                     // Flag for a display update.
byte DataToDisplay = 0;                                         // Index value that selects what value to draw on display.    


//#############
//#   SETUP   #
//#############
void setup()  // Start of setup:
{    
  // Configure DIO & AIO
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, INPUT_PULLUP);       // RPM Input
  pinMode(TACH, OUTPUT);          // Tachometer meter. (PWM)
  pinMode(FAN, OUTPUT);           // Fan relay.
  pinMode(TEST, OUTPUT);          // Test pin for debugging program.
  
  analogReference(DEFAULT);       // 5v adc reference.

  // Configure OLED display.
  OLED.begin();
  OLED.setBusClock(400000);
  OLED.setFlipMode(1);
  OLED.setFont(u8x8_font_inb33_3x6_r);
  OLED.setCursor(8, 2);
  OLED.print(" F");

  //Serial.begin(250000);
  
  // Configure Timer2 for PWM, pin & duty cycle.
  
  TCCR2A = bit(COM2A1) | bit(WGM21) | bit(WGM20);
  TCCR2B = bit(CS22) | bit(CS21) | bit(CS20);                 // 256 prescaler
  TCNT2 = 0;
  TIMSK2 = 0;
  OCR2A = 29;                                      // Dutycycle for 0 reading.

  // Configure Timer1 for display update.

  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = bit(WGM12) | bit(CS12) | bit(CS10);   // turn on CTC mode, 1024 prescaler
  TCNT1  = 0;     // initialize counter value to 0
  
    // set compare match register for 0.5hz increments
  OCR1A = 31249;  // = (16*10^6) / (0.5*1024) - 1 (must be < 65536)
  
  TIMSK1 = bit(OCIE1A);    // enable timer compare interrupt

  //delay(1000);  //Since we don't have any readings stored we need a high enough value in micros() so divide doesn't return negative values.
  
  attachInterrupt(digitalPinToInterrupt(2), Pulse_Event, RISING);  // Enable interruption pin 2 when going from LOW to HIGH.

}  // End of setup.


/*##########################################
########        INTERRUPTS        ##########
##########################################*/

ISR(TIMER1_COMPA_vect)  // timer interrupt service routine
{
  UpdateDisplay = true;
} // End of ISR.


void Pulse_Event()  // The interrupt runs this to calculate the period between pulses:
{
  Time = micros();
  
  TimerPeriod = Time - LastTimerValue;

  LastTimerValue = Time;

  PeriodSum += TimerPeriod;  // Add the periods so later we can average.
    
  PulseCount++;  // Increase the counter for amount of readings by 1.
  
}  // End of Pulse_Event.


/*##########################################
##########      MAIN LOOP       ############
##########################################*/
void loop()  // Start of loop:
{
  byte Temperature;
  unsigned long frequency;
  
  while(1)
  {
    if(PulseCount >= NPULSES)  // If counter for amount of readings reach the set limit:
    { 
      frequency = 1E7 / (PeriodSum / PulseCount);                                
   
      OCR2A = byte(frequency / 58 + 29);
  
      //Serial.print(period);
      //Serial.println(frequency);
      
      PulseCount = 1;                                     // Reset the counter to start over.
      
      PeriodSum = TimerPeriod;                            // Reset PeriodSum to start a new averaging operation.
    }  

    Temperature = thermistor[analogRead(COOLANT)];
    
    if(Temperature >= FAN_ON)
    {
      digitalWrite(FAN, HIGH);
    }
    else if((Temperature + FAN_DELTA) < FAN_ON)
    {
      digitalWrite(FAN, LOW);
    }
    
    // Update display on Timer1 interrupt.
    if (UpdateDisplay)
    {
      Refresh_OLED(DataToDisplay);
      
      UpdateDisplay = false;
    }
  }
}  // End of loop.



/*##########################################
#############   FUNCTIONS   ################
##########################################*/
/*
// Returns temperature from thermistor in F.
float Thermistor_Temperature(int channel, byte sample_size)
{
  byte i;
  float average=0, result;

  // take N samples in a row, with a slight delay
  for (i=0; i < sample_size; i++) {
   average += analogRead(channel);
   delay(10);
  }
  
  average /= sample_size;

  if (average > 1020) return 0;
  
  // convert the value to resistance
  average = 1023 / average - 1;
  average = BIAS_RESISTOR / average;
  
  // 1/T = 1/To + 1/B * ln(R/Ro)
  
  result = average / THERMISTOR_RO;         // (R/Ro)
  result = log(result);                     // ln(R/Ro)
  result /= THERMISTOR_BETA;                // 1/B * ln(R/Ro)
  result += 1 / 298.15;                     // + (1/To)
  result = 1 / result;                      // Invert
  
  // convert to F
  result -= 273.15;                         
  result *= 1.8;
  result += 32;
  return result;
} // End of Thermistor_Temperature.
*/



// Draws temperature value on OLED display.
void Display_Temperature(const char *title, const char *value, bool invert)
{  
  OLED.setInverseFont(invert);
  OLED.setCursor(0,0);
  OLED.setFont(u8x8_font_px437wyse700a_2x2_r);
  OLED.drawString(0, 0, title);
  
  OLED.setFont(u8x8_font_inb33_3x6_r);
  OLED.drawString(0, 2, value);
} // End of Display_Temperature.



// Scans OLED display & activates fan relay if above threshold.
void Refresh_OLED(byte Item)
{
  byte result;
  char outStr[5];     // String for data to OLED.
  
  switch (Item)
    {
      case 0:   
        result = thermistor[analogRead(COOLANT)];
        
        dtostrf(result, 3, 0, outStr);
        
        if (digitalRead(FAN)) 
        {
          Display_Temperature("CLT Rtn",  outStr, true);
        }
        else
        {
          Display_Temperature("CLT Rtn", outStr, false);
        }

        DataToDisplay++;
        
        break;

      case 1:
          result = thermistor[analogRead(PS_OIL)];

          dtostrf(result, 3, 0, outStr);
          
          Display_Temperature("P/S Oil", outStr, false);
          
          DataToDisplay = 0;
          
          break;
    }
} // End of Refresh_OLED.
