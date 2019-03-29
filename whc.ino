//http://codius.ru/articles/Arduino_%D1%83%D1%81%D0%BA%D0%BE%D1%80%D1%8F%D0%B5%D0%BC_%D1%80%D0%B0%D0%B1%D0%BE%D1%82%D1%83_%D0%BF%D0%BB%D0%B0%D1%82%D1%8B_%D0%A7%D0%B0%D1%81%D1%82%D1%8C_2_%D0%90%D0%BD%D0%B0%D0%BB%D0%BE%D0%B3%D0%BE_%D1%86%D0%B8%D1%84%D1%80%D0%BE%D0%B2%D0%BE%D0%B9_%D0%BF%D1%80%D0%B5%D0%BE%D0%B1%D1%80%D0%B0%D0%B7%D0%BE%D0%B2%D0%B0%D1%82%D0%B5%D0%BB%D1%8C_%D0%90%D0%A6%D0%9F_%D0%B8_analogRead
//http://arduinonsk.ru/index.php/blog/87-preryvaniya-na-vsekh-pinakh-arduino
//https://www.teachmemicro.com/arduino-interrupt-tutorial/

#include <TM1637Display.h>
#include <math.h>
#include <EEPROM.h>
#include <MsTimer2.h>
#include "RTClib.h"

RTC_DS3231 rtc;
DateTime T;

#define SERIAL_R 11920 // сопротивление последовательного резистора

#define DOUBLE_TO_INT(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
// LED display connection pins (Digital Pins)
#define CLK 10
#define DIO 11

#define RELAY_PIN 7
#define RELAY_PIN_PD PD7

//CHANGE "PIND & (1 << PD5)" after modify button port!!!
//buttons
#define SBTN_PD PD4
#define UBTN_PD PD5
#define DBTN_PD PD6
#define SBTN 4
#define UBTN 5
#define DBTN 6
#define MOD_VAL_DELAY 180
//display blink count max
#define MC_MAX 4

//temp settings/limits
#define MAX_TEMP 75
#define MIN_TEMP 35
#define TEMP_PIN A0

TM1637Display display(CLK, DIO);
//current state of led display and controller logic
uint8_t volatile mode;
//settings display led blink counter
uint8_t volatile mc;
const float aref = 1.038;
//stainhart constants for my sensor
const double A = 0.0007130888742;//e-3
const double B = 0.0003107451187;//e-4
const double C = -0.0000002858893427;//e-7
const unsigned long debounceDelay = 80;
const uint8_t  tempBuffer = 4;
//prevent trigger during boot time
bool allowButtons = false;
//current temp
double temp;

bool volatile relayOn;
bool volatile lowBat = false;
bool volatile runOnce;
//curent settings value
uint8_t vi[] = {0, 0, 0};

unsigned long btnLast[] = {0, 0, 0};

//eeprom memory  of temp
uint8_t  ti;
//only in mode=0. 0 - temp, 1 - time
bool isTime;
#define BUFFER_SIZE 16 // For 12 Bit ADC data
volatile uint32_t result[BUFFER_SIZE];
volatile byte i = 0;
volatile uint32_t sum = 0;
volatile uint32_t tempRaw;
/*
  ADC and Timer1 setup function
  Argument 1: uint8_t channel must be between 0 and 7
  Argument 2: int frequency must be integer from 1 to 600
    WARNING! Any value above 600 is likely to result in a loss
    of data and could result in a reduced accuracy of ADC
    conversions
*/
void setupADC(uint8_t channel, int frequency)
{
  cli();
  ADMUX = (ADMUX & 0xe0) | (channel & 0x07);
  ADMUX |= (1 << REFS0) | (1 << REFS1);
  ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | (1 << ADATE) | _BV(ADIE);
  ADCSRB |= _BV(ADTS2) | _BV(ADTS0);  //Compare Match B on counter 1
  TCCR1A = _BV(COM1B1);
  TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);
  /*
    //improve temp sensor read, need additional hardware

    uint32_t clock = 250000;
    uint16_t counts = clock / (BUFFER_SIZE * frequency);
    OCR1A = counts;
  */
  TIMSK1 = _BV(OCIE1B);
  ADCSRA |= _BV(ADEN) | _BV(ADSC);

  sei();
}

ISR(ADC_vect) {
  result[i] = ADC;
  i = ++i & (BUFFER_SIZE - 1);
  for (int j = 0; j < BUFFER_SIZE; j++)
  {
    sum += result[j];
  }
  if (i == 0)
  {
    sum = sum >> 2;
    tempRaw = sum;
  }
  sum = 0;

}
ISR(TIMER1_COMPB_vect) {
}


ISR(PCINT2_vect) {
  if (allowButtons) {
    unsigned long btnTime = millis();
    if (PIND & (1 << SBTN_PD)  && debounceDelay < (btnTime - btnLast[0])) {
      btnLast[0] = btnTime;
      changeMode();

    } else if (isUpOn()  && debounceDelay < (btnTime - btnLast[1])) {
      btnLast[1] = btnTime;
      if (mode == 0) {
        mode = 1;
        vi[0] = ti;
      }
      upValue();

    } else if (PIND & (1 << DBTN_PD)  && debounceDelay < (btnTime - btnLast[2])) {
      btnLast[2] = btnTime;
      if (runOnce) {
        runOnce = false;
        //not possible to run when relay enabled, or already heated
      } else if (mode == 0 && hitLowTempLimit()) {
        runOnce = true;
      }

    }

  }

}
bool isUpOn() {
  return PIND & (1 << UBTN_PD);
}

void setup() {
  //Serial.begin(9600);

  setRelay();

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode( TEMP_PIN, INPUT );
  pinMode( SBTN, INPUT );
  pinMode( UBTN, INPUT );
  pinMode( DBTN, INPUT );
  //enable change interupt
  PCICR |= 0b00000100;
  PCMSK2 |= _BV(PCINT20) | _BV(PCINT21) | _BV(PCINT22);

  display.setBrightness(0x09);
  display.clear();

  setupADC(0, 16);
  MsTimer2::set (500, relay);

  initTime();
  MsTimer2::start ();

}
void setRelay() {
  //setup boiler relay temperature
  ti = EEPROM.read(0);
  //  Serial.print("ROM TEMP ");
  //  Serial.println(ti);
  if (ti <= 0 || ti > MAX_TEMP) {
    ti = MIN_TEMP;
    EEPROM.write(0, ti);
  }

  //0 freerun, 1 - temp settings, 2 - hour, 3 - minutes
  mode = 0;
  isTime = false;

  runOnce = false;
}
void initTime() {
  int count = 4;
  delay(2500);
  rtc.begin();
  while (count > 0) {
    T = rtc.now();
    checkBat();
    if (lowBat) {
      delay(500);
    } else {
      break;
    }
    count --;
  }
}
void checkBat() {
  lowBat = rtc.lostPower() || T.year() < 2019;
}

void upValue() {
  int index = mode - 1;
  vi[index]++;
  if (mode == 1 &&  vi[index] > MAX_TEMP) {
    vi[index] = MIN_TEMP;
  } else if ((mode == 2 &&  vi[index] > 23)) {
    vi[index] = 0;
  } else if (mode == 3 &&  vi[index] > 59) {
    vi[index] = 0;
  }
  updateDisplay();
  mc++;
  if (mc > MC_MAX) {
    mc = MC_MAX;
  }

}
//save current temp or update time
void processValue() {
  if (vi[0] > 0) {
    ti =  vi[0];
    EEPROM.write(0, ti);
  }
  bool both = vi[1] > 0 && vi[2] > 0;
  //if hour and minute
  if (lowBat && both) {

    lowBat = false;
    T = DateTime (2019, 3, 23, vi[1], vi[2], 1);
    rtc.adjust(T);
  } else if (!lowBat) {
    T = rtc.now();
    T = DateTime (T.year(), T.month(), T.day(), (both || vi[1] > 0 ? vi[1] : T.hour()), (both || vi[2] > 0 ? vi[2] : T.minute()), T.second());

    rtc.adjust(T);
  }
  vi[0] = 0;  vi[1] = 0; vi[2] = 0;  mode = 0;  mc = 0;
}
void changeMode() {
  mode++;
  if (mode > 3) {
    mode = 1;
  }
  if (mode == 1) {
    vi[0] = ti;
  } else if (mode == 2) {
    vi[1] = T.hour();
    if (lowBat) {
      vi[1] = 10;
    }
  } else if (mode == 3) {
    vi[2] = T.minute();
    if (lowBat) {
      vi[2] = 15;
    }
  }

  mc = MC_MAX;
  updateDisplay();
}

void updateDisplay() {
  int index = mode - 1;
  switch (mode) {
    case 1: //temp
      display.showNumberDecEx(vi[index], 0b00000000, false);
      break;
    case 2: //hour
      display.showNumberDecEx(vi[index] * 100, 0b01000000, true);
      break;
    case 3://minute
      display.showNumberDecEx(vi[index], 0b01000000, true);
      break;
    default:
      //blink display
      if (isTime) {
        display.showNumberDecEx(T.hour() * 100 +  T.minute(), 0b01000000, true);
      } else {
        display.showNumberDecEx(DOUBLE_TO_INT(temp) * 100 + ti, 0b00000000, false);

      }
      isTime = !isTime;
      break;
  }
}

void loop() {
  allowButtons = true;

  while (1) {
    T = rtc.now();
    checkBat();
    //digitalClockDisplay();
    if (mode == 0) {

      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      if (mode != 0)continue;
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      if (mode != 0)continue;

      if (lowBat) {
        display.clear();
        uint8_t data[] = { 0b1111100, 0b1110111, 0b0000111, 0b0110001 };
        display.setSegments( data);
        delay(1000);
      } else {
        updateDisplay();
      }

    } else {
      while (mc > 0) {
        bool on = isUpOn();
        while (on) {
          upValue();
          on = isUpOn();
          if (on) {
            delay(MOD_VAL_DELAY);
          }
        }
        display.clear();
        delay(500);
        updateDisplay();
        delay(500);

        mc--;
      }
      processValue();
    }
  }
}
/*
  void digitalClockDisplay() {
  DateTime dm = rtc.now();
  Serial.print(dm.hour());
  Serial.print(' ');
  Serial.print(dm.minute());
  Serial.print(' ');
  Serial.print(dm.second());
  Serial.print(' ');
  Serial.print(dm.day());
  Serial.print(' ');
  Serial.print(dm.month());
  Serial.print(' ');
  Serial.print(dm.year());

  Serial.println();
  Serial.print("LBat ");
  Serial.print(lowBat);
  Serial.print(" LPow ");
  Serial.print( rtc.lostPower());
  Serial.println();
  if ( rtc.lostPower()) {
    T = DateTime (2019, 3, 23, 1, 1, 1);
    rtc.adjust(T);
  }

        Serial.print("Temp ");
            Serial.print(temp);
            Serial.println();


  }
*/
bool hitLowTempLimit() {
  return temp <= ti - tempBuffer && !relayOn;
}
//controls relay
void relay() {
  //read temp
  temp = calcTemp();

  uint8_t h = T.hour();
  bool isTime = h > 22 || h >= 0 && h <= 7;
  bool isHealth = temp > 0 && temp < 80;
  bool needMod = false;
  relayOn = PIND & (1 << RELAY_PIN_PD) ;
  bool hitLow = hitLowTempLimit();
  /*
    Serial.println();
    Serial.println("isHealth : ");
    Serial.println(isHealth);
    Serial.println("isTime : ");
    Serial.println(isTime);
    Serial.println("hitLow  : ");
    Serial.println(hitLow);
    Serial.println("hitHigh : ");
    Serial.println(hitHigh);
    Serial.println();
  */
  //disable relay in setup mode
  if (mode != 0 || !isHealth || lowBat) {
    if ( relayOn ) {
      relayOn = false;
      needMod = true;
    }

  } else if (runOnce) {
    if (hitLow && !relayOn) {
      relayOn = true;
      needMod = true;

    } else if (temp >= ti && relayOn) {
      relayOn = false;
      needMod = true;

    } 
  } else {
    if (hitLow && isTime && !relayOn) {
      relayOn = true;
      needMod = true;
      runOnce = false;

    } else if ((!isTime || temp >= (isTime && ti > 50 ? 50 : ti)) && relayOn) {
      relayOn = false;
      needMod = true;
    }
  }

  if (temp > MAX_TEMP) {
    needMod = true;
    relayOn = false;
  }

  if (needMod) {
    //Serial.println("Relay change state : ");
    //Serial.println(relayOn);
    if (relayOn) {
      digitalWrite(RELAY_PIN, HIGH);
    } else {
      runOnce = false;
      digitalWrite(RELAY_PIN, LOW);
    }
  }
}
double calcTemp() {
  double Va = tempRaw * (aref / 4096.0) ;

  return toSteinhart( SERIAL_R * (1 / ((aref / Va) - 1)));
}

double toSteinhart(float R) {
  double Temp = log(R);
  Temp = 1 / (A + (B * Temp) + (C * Temp * Temp * Temp));
  Temp = Temp - 273.15;

  return Temp;
}
