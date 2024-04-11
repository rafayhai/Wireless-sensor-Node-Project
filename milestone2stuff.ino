#include <avr/sleep.h>
#include <avr/power.h>

const int pwmPin = 9;
const int outputV = A0;
const int inputI = A5;
const int outputI = A4;
const int powerCheck = A2;
volatile unsigned long onTimeMicros = 25; // on-time period in microseconds
volatile unsigned long offTimeMicros = 25;// off-time period in microseconds
volatile float time = 0.0;
volatile bool pinState = LOW;
#define ULONG_MAX 4294967295UL
volatile int intervalCount = 0;
int intervalsFor30Sec =  30000;
static float dutyCycle = 0.50;
volatile bool promptOnce = false;
int n = 0;
double Vavg = 0.0;
double Vtot = 0.0;

volatile bool powerSaveOn = false;
volatile bool powerSaveOff = true;

unsigned long previousMicros = 0;
unsigned long currentMillis;

// Thermistor variables
int ThermistorPin = 1;
int Vo;
float R1 = 10000;
float logR2, R2, T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

float vin;

void setup() {
  pinMode(pwmPin, OUTPUT); // set pwmPin as an output
   Serial.end();
  Serial.begin(9600);
 
  noInterrupts();
    // Set up Timer 1 for Fast PWM mode
  TCCR1A = _BV(COM1A1) | _BV(WGM11); // Clear OC1A on compare match, set OC1A at BOTTOM, WGM11 = 1
  TCCR1B = _BV(WGM12) | _BV(WGM13) | _BV(CS10); // WGM12 = 1, WGM13 = 1 (Fast PWM mode, TOP = ICR1), CS10 = 1 (no prescaling)
  ICR1 = 399; // Set the TOP value for 50 kHz PWM frequency (20 us period) //this is the on time period
  OCR1A = 199; // Set the duty cycle (50%) 

  // Enable Timer 1 Output Compare A Match interrupt
  TIMSK1 |= _BV(OCIE1A);

  interrupts();
}

  ISR(TIMER2_COMPA_vect) {
    if (powerSaveOn) {
    intervalCount++;
    if (intervalCount >= intervalsFor30Sec) {
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  Tf = (Tc * 9.0)/ 5.0 + 32.0; 

  Serial.print("Temperature: ");
  Serial.print(Tf);
  Serial.print(" F; ");
  Serial.print(Tc);
  Serial.println(" C");
  intervalCount = 0; // Reset the counter if you want to start counting again
    }
  }
}

void loop() {
  currentMillis = millis();

  if (powerSaveOff) {
  //enableTimer1();
  sleep_disable();

  float Vout = analogRead(outputV);
  float voltage = Vout * (5.0 / 1024.0);
  voltage = voltage * 2.5;  // calculates actual output voltage (due to voltage divider)

  vin = analogRead(powerCheck);
  vin = vin * (5.0 / 1024.0);

  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  Tf = (Tc * 9.0)/ 5.0 + 32.0; 

  Serial.print("Temperature: ");
  Serial.print(Tf);
  Serial.print(" F; ");
  Serial.print(Tc);
  Serial.println(" C");

    n++;
  Vtot += voltage;
  if (n % 20 == 0) {
    Vavg = Vtot / 20;
    // Serial.print("Avg V: ");
    // Serial.println(Vavg);

    // stops duty cycle change once it reaches 20% or 70%
    if (Vavg < 9.95 && dutyCycle <= 0.7) {
      dutyCycle += 0.001;
    } else if (Vavg > 10.05 && dutyCycle >= 0.2) {
      dutyCycle -= 0.001;
    }
    Vtot = 0.0;
    setDutyCycle(dutyCycle);

  onTimeMicros = dutyCycle * 50;
  offTimeMicros = 50 - onTimeMicros;
}
  }
  else if (powerSaveOn) {
  // Sleep mode
  //sleep_cpu();
  Serial.print("Millis: ");
  Serial.println(currentMillis);

  // Read Vin and thermistor every 30s
  if (currentMillis - previousMicros >= 30000) {

  sleep_disable();

  vin = analogRead(powerCheck);
  vin = vin * (5.0 / 1024.0);

  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  Tf = (Tc * 9.0)/ 5.0 + 32.0; 

  Serial.print("Temperature: ");
  Serial.print(Tf);
  Serial.print(" F; ");
  Serial.print(Tc);
  Serial.println(" C");
  previousMicros = currentMillis;
  sleep_enable();
    }

  }

  if (vin <= 0.5) {
    powerSaveOn =true;
    powerSaveOff = false;
   set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  // Enable sleep
    sleep_enable();
    disableTimer1();
  }
  else {
    powerSaveOn =false;
    powerSaveOff = true;
    enableTimer1();
    sleep_disable(); 

  }
  
}

void disableTimer1() {
    TCCR1B &= ~(1 << CS10 | 1 << CS11 | 1 << CS12); // Stop Timer 1
}

void enableTimer1() {
    TCCR1B |= (1 << CS10); // Start Timer 1 with no prescaling, adjust as needed
}

// timer needed
 ISR(TIMER1_COMPA_vect) {
 }

//calculates a new OCR1A value based on new duty cycle percentage in the loop
void setDutyCycle(float dutyCyclePercent) {
  OCR1A = ICR1 * dutyCyclePercent;
}