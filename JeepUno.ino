#include <U8x8lib.h>
#include <Wire.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);

//###########
// Constants
//###########

#define FAN_ON      185          // Fan turn on temperature.
#define FAN_DELTA   20           // Fan turn off delta T. 

#define FAN         12           // Pin assigned to Fan relay. 
#define TACH        11           // Pin assigned to Tachometer meter. 
#define TEST        7            // Pin assigned to testing.

#define PulsesPerRevolution 6    // Alternators are commonly 6.
#define ZeroTimeout 100000       // Period of cutoff frequency. f = 1/(ZeroTimeout * 1e-6)

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

volatile unsigned long LastTime;                                // Stores the last time we measured a pulse so we can calculate the period.
volatile unsigned long PeriodBetweenPulses = ZeroTimeout+1000;  // Stores the period between pulses in microseconds.
volatile unsigned long PeriodAverage = ZeroTimeout+1000;        // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.

unsigned long RPM;                                              // Raw RPM without any processing.
unsigned int PulseCount = 16;                                   // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.

unsigned long PeriodSum;                                        // Stores the summation of all the periods to do the average.

unsigned long LastTimeCycleMeasure = LastTime;                  // Stores the last time we measure a pulse in that cycle.

unsigned long CurrentMicros = micros();  
unsigned int AmountOfReadings = 8;

float result=0;                                                 // Calculation result.
char payload[5];                                                // String for OLED.

bool UpdateDisplay = false;                                     // Flag for a display update.
byte DisplayItem = 0;                                           // Index value that selects what value to draw on display.    


void setup()  // Start of setup:
{   
  // Configure DIO & AIO
   
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, INPUT_PULLUP);       // RPM Input
  pinMode(TACH, OUTPUT);          // Tachometer meter. (PWM)
  pinMode(FAN, OUTPUT);           // Fan relay.
  pinMode(TEST, OUTPUT);          // Test pin for debugging program.
  
  analogReference(DEFAULT);       // 5v adc reference.
  
  // Configure Timer2 for PWM, pin & duty cycle.
  
  TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22) | _BV(CS21);                 // 256 prescaler
  OCR2A = 35;                                      // Dutycycle

  // Configure Timer1 for display update.

  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  TCNT1  = 0;     //initialize counter value to 0
  // set compare match register for 0.5hz increments
  OCR1A = 31311;  // = (16*10^6) / (0.5*1024) - 1 (must be < 65536)
  
  TCCR1B |= (1 << WGM12);   // turn on CTC mode
  
  TCCR1B |= (1 << CS12) | (1 << CS10);  // Set CS10 and CS12 bits for 1024 prescaler
  
  TIMSK1 |= (1 << OCIE1A);    // enable timer compare interrupt

  attachInterrupt(digitalPinToInterrupt(2), Pulse_Event, RISING);  // Enable interruption pin 2 when going from LOW to HIGH.
  
  Serial.begin(250000);

  // Configure OLED display.
  oled.begin();
  oled.setFont(u8x8_font_inb33_3x6_r);
  oled.setCursor(8, 2);
  
  delay(1000);  //Since we don't have any readings stored we need a high enough value in micros() so divide doesn't return negative values.
  
  oled.print(" F");
}  // End of setup.


/*##########################################
#############   INTERRUPTS   ###############
##########################################*/

ISR(TIMER1_COMPA_vect)  // timer interrupt service routine
{
  UpdateDisplay = true;
} // End of ISR.


void Pulse_Event()  // The interrupt runs this to calculate the period between pulses:
{
  PeriodBetweenPulses = micros() - LastTime;

  LastTime = micros();

  if(PulseCount >= AmountOfReadings)  // If counter for amount of readings reach the set limit:
  {
    PeriodAverage = PeriodSum / AmountOfReadings;  // Calculate the final period dividing the sum of all readings by the
                                                   // amount of readings to get the average.
    PulseCount = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
    PeriodSum = PeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.
  }
  else
  {
    PulseCount++;  // Increase the counter for amount of readings by 1.
    PeriodSum += PeriodBetweenPulses;  // Add the periods so later we can average.
  }
}  // End of Pulse_Event.


/*##########################################
#############   MAIN LOOP   ################
##########################################*/
void loop()  // Start of loop:
{
  RPM = Calc_RPM();

  //OCR2A = Calc_DutyCycle(RPM);    // Sets dutycycle of PWM.
  
  /*
  Serial.print("RPM: ");
  Serial.print(RPM);
*/
  // Update display on Timer1 interrupt.
  if (UpdateDisplay)
  {
    Refresh_OLED(DisplayItem);
    UpdateDisplay = false;
  }
}  // End of loop.


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


// Draws temperature value on OLED display.
void Display_Temperature(const char *title, const char *value, bool invert)
{  
  oled.setInverseFont(invert);
  oled.setCursor(0,0);
  oled.setFont(u8x8_font_px437wyse700a_2x2_r);
  oled.drawString(0, 0, title);
  
  oled.setFont(u8x8_font_inb33_3x6_r);
  oled.setCursor(0, 2);
  oled.print(value);
} // End of Display_Temperature.



// Scans OLED display.
void Refresh_OLED(byte Item)
{
  switch (Item)
    {
      case 0:   
        result = Thermistor_Temperature(COOLANT, 5);
         
        if (result >= FAN_ON)
        {
          digitalWrite(FAN, HIGH);
        }
        else if ((result + FAN_DELTA) < FAN_ON)
        {
          digitalWrite(FAN, LOW);  
        }
      
        dtostrf(result, 3, 0, payload);
         
        if (digitalRead(FAN)) 
        {
          Display_Temperature("CLT Rtn", payload, true);
        }
        else
        {
          Display_Temperature("CLT Rtn", payload, false);
        }

        DisplayItem++;
        
        break;

      case 1:
          result = Thermistor_Temperature(PS_OIL, 5);
          
          dtostrf(result, 3, 0, payload);
          
          Display_Temperature("P/S Oil", payload, false);
          
          DisplayItem = 0;
          
          break;
    }
} // End of Refresh_OLED.

// Calculates RPM from timer values.
unsigned long Calc_RPM()
{
  unsigned long Frequency, rpm;
  
  LastTimeCycleMeasure = LastTime;          // Store the LastTime in a variable.
  CurrentMicros = micros();                 // Store the micros() in a variable.

  // CurrentMicros should always be higher than LastTime, but in rare occasions that's not true.
  if(CurrentMicros < LastTimeCycleMeasure) LastTimeCycleMeasure = CurrentMicros;
  
  // Detect if pulses stopped or frequency is too low, so we can show 0 Frequency:
  if(PeriodBetweenPulses > ZeroTimeout || CurrentMicros - LastTimeCycleMeasure > ZeroTimeout)
  {
    Frequency = 0;  // Set frequency as 0.
  }

  //Serial.print("Period: ");
  //Serial.print(PeriodBetweenPulses);
  
  // Calculate the frequency. Decimal is shifted by 1 to the right to keep freq as integer.
  Frequency = 10E6 / PeriodAverage;
  
  // Calculate the RPM:
  rpm = Frequency / PulsesPerRevolution * 60;     // Frequency divided by amount of pulses per revolution multiply by
                                                  // 60 seconds to get minutes.
  rpm = rpm / 10;  // Remove the decimals.

  return rpm;
} // End of Calc_RPM.


byte Calc_DutyCycle(unsigned long rpm)
{
  byte result;

  // Coefficients m & b characterize the behavior of gauge.
  
  result = 1 * rpm + 10;
  return result;
} // End of Calc_DutyCycle.
