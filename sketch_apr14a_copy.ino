// GROUP 5
// CPE 301.1001
// Semester Project
/* NOTES: 
DHT Library is: DHTlib by Rob Tillaart */


#include <dht.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <TimeLib.h>
#include <DS1307RTC.h>

/*---- Preprocessor Directives ----*/
// LCD
#define DHT11_PIN 10
#define DHT11_PWR 9
#define DHT11_GND 8

// MOTOR
#define ENABLE 44
#define DIRA 45
#define DIRB 46

//STEPPER
#define STEPS 100

// DS1307RTC
#define DS1307RTC_GND 18
#define DS1307RTC_PWR 19

/*---- Declarations ----*/
//Analog Stuff
volatile unsigned char* my_ADMUX = (unsigned char *)0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char *)0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char *)0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int *)0x78;

//STEPPER
Stepper stepper(STEPS, 54,55,56,57);
int previous = 5;

// LCD Constructor
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// DHT Declaration
dht DHT;

// WATER LEVEL SENSOR Values
int resval = 0;  // holds the value
int respin = A5; // sensor pin used


/*---- Functions ----*/
void setup(){
  // setup the ADC
  adc_init();  
  
  // LCD
  lcd.begin(16, 2);
  
  // Motor
  pinMode(ENABLE,OUTPUT);
  pinMode(DIRA,OUTPUT);
  pinMode(DIRB,OUTPUT);

  // DHT
  pinMode(DHT11_GND, OUTPUT);
  pinMode(DHT11_PWR, OUTPUT);
  digitalWrite(DHT11_GND, LOW);
  digitalWrite(DHT11_PWR, HIGH);

  // Stepper Setup
  stepper.setSpeed(30);

  // DS1307RTC 
  //set up pins to power the DS1307 module from the Arduino pins (20mA max)
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  digitalWrite(18, LOW);
  digitalWrite(19, HIGH);

  Serial.begin(9600);

  // DS1307RTC Output
  while (!Serial) ; // wait for serial
  delay(200);
  Serial.println("DS1307RTC Read Test");
  Serial.println("-------------------");
}

void loop(){
  // MOTOR
//  digitalWrite(DIRA,HIGH); //one way
//  digitalWrite(DIRB,LOW);
//  analogWrite(ENABLE,100); //enable on
//  //delay(200);
  motor();
  
  // LCD / TEMP / HUMIDITY Output
  temp();

  // WATER LEVEL SENSOR
  /* The resistance value is inversely proportional to the water level.
  As a result, the resistance value should be divided by 1000, since the max value is ~1024. */
  resval = analogRead(respin);
  Serial.print(resval);
  if (resval<=100){
    Serial.println("Water Level: High"); 
  } 
  else if (resval>100 && resval<=500){ 
    Serial.println("Water Level: Medium"); 
  } 
  else if (resval>500 && resval<=1000){ 
    Serial.println("Water Level: Low");
  } 
  else if (resval > 1000){ 
    Serial.println("Water Level: Empty"); 
  }

  // Stepper Motor
  // get the sensor value
  // int val = analogRead(0);
  int val = adc_read(0);

  // move a number of steps equal to the change in the
  // sensor reading
  stepper.step(val - previous);

  // remember the previous value of the sensor
  previous = val;


  // DS1307RTC
    tmElements_t tm;
//
//  if (RTC.read(tm)) {
//    Serial.print("Ok, Time = ");
//    print2digits(tm.Hour);
//    Serial.write(':');
//    print2digits(tm.Minute);
//    Serial.write(':');
//    print2digits(tm.Second);
//    Serial.print(", Date (D/M/Y) = ");
//    Serial.print(tm.Day);
//    Serial.write('/');
//    Serial.print(tm.Month);
//    Serial.write('/');
//    Serial.print(tmYearToCalendar(tm.Year));
//    Serial.println();
//  } else {
//    if (RTC.chipPresent()) {
//      Serial.println("The DS1307 is stopped.  Please run the SetTime");
//      Serial.println("example to initialize the time and begin running.");
//      Serial.println();
//    } else {
//      Serial.println("DS1307 read error!  Please check the circuitry.");
//      Serial.println();
//    }
//    delay(9000);
//  }

  delay(1000);

}


void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

// Device Functions
void motor(){
  digitalWrite(DIRA,HIGH); //one way
  digitalWrite(DIRB,LOW);
  analogWrite(ENABLE,100); //enable on 
}

void temp(){
  // LCD / TEMP / HUMIDITY Output
  int chk = DHT.read11(DHT11_PIN);
  lcd.setCursor(0,0); 
  lcd.print("Temp: ");
  lcd.print(DHT.temperature);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("Humidity: ");
  lcd.print(DHT.humidity);
  lcd.print("%");

//  Serial.print("Temp: ");
//  Serial.print(DHT.temperature);
}



// Analog Functions
void adc_init()
{
  // set up the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 5 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 3 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 2-0 to 0 to set prescaler selection to slow reading
 
  // set up the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
 
  // set up the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // reset the channel and gain bits
  *my_ADMUX  &= 0b11100000; //Resetting Admux 0-4
 
  // clear the channel selection bits
  *my_ADCSRB &= 0b11110111;
 
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
   
    // set MUX bit
    *my_ADCSRB |= 0b00001000;
  }
 
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
 
  // set bit ?? of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40; //Register bit 6 in ADCSRA is the bit that starts a conversion
 
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0); //ADIF is a register in ADCSRA that is basically an interrupt flag
 
  // return the result in the ADC data register
  return *my_ADC_DATA;
}
