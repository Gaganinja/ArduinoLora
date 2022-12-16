#define Program_Version "V1.0"
/************************ Include librairies *********************************/
#include <SPI.h>
#include "DHT.h"
#include <AirQuality.h>
#include <avr/wdt.h>                 //watchdog timer library, integral to Arduino IDE
#include <avr/sleep.h>
#include <SX128XLT.h>
#include "Settings.h"

/************************ Sleep Mode ******************************************/
#if defined ENABLE_SLEEP_MODE

volatile byte sleep_cycles_remaining;

ISR(WDT_vect)
{
    --sleep_cycles_remaining;
}

void sleepNode(unsigned int cycles)
{
    sleep_cycles_remaining = cycles;
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
    sleep_enable();

    WDTCSR |= _BV(WDIE);

    while (sleep_cycles_remaining) {
        sleep_mode(); // System sleeps here
    }                 // The WDT_vect interrupt wakes the MCU from here
    sleep_disable();  // System continues execution here when watchdog timed out

    WDTCSR &= ~_BV(WDIE);
}

void setup_watchdog(uint8_t prescalar)
{

    uint8_t wdtcsr = prescalar & 7;
    if (prescalar & 8)
        wdtcsr |= _BV(WDP3);
    MCUSR &= ~_BV(WDRF); // Clear the WD System Reset Flag
    WDTCSR = _BV(WDCE) | _BV(WDE);           // Write the WD Change enable bit to enable changing the prescaler and enable system reset
    WDTCSR = _BV(WDCE) | wdtcsr | _BV(WDIE); // Write the prescalar bits (how long to sleep, enable the interrupt to wake the MCU
}
#endif // Enable sleep mode

/************************ Define global vars & objs *********************************/
SX128XLT LT;  // Creating Lora Transmitter object
DHT dht(DHTPIN, DHTTYPE);
AirQuality aqs;

struct payload_t {
  float tmp;
  float hum; 
  int aq; 

  String toString(){
    return String(tmp)+String(';')+String(hum)+String(';')+ String(aq);
  }
};

bool SendOK;
int8_t TestPower;
uint8_t TXPacketL;
uint32_t qualityTime;
volatile bool wakeUpFlag = false;

/************************ Define LoRa communication functions *********************************/
void packet_is_OK()
{
  Serial.print(F(" "));
  Serial.print(TXPacketL);
  Serial.print(F(" Bytes SentOK"));
}

void packet_is_Error()
{
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                           //get the IRQ status
  Serial.print(F("SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
  LT.printIrqStatus();
  digitalWrite(LED1, LOW);                                  //this leaves the LED on slightly longer for a packet error
}

bool Send_Data_Packet(payload_t p)
{
  uint8_t bufffersize = sizeof(p)+4;

  String msg = p.toString();
  Serial.println(msg);
  char buff[bufffersize];
  msg.toCharArray(buff, bufffersize);
  Serial.println(buff);
  buff[bufffersize -1] = ' ';
  TXPacketL = sizeof(buff);
  
  if (sizeof(buff) > TXBUFFER_SIZE)              //check that defined buffer is not larger than TX_BUFFER
  {
    bufffersize = TXBUFFER_SIZE;
  }
  else
  {
    bufffersize = sizeof(buff);
  }

  TXPacketL = bufffersize;

  LT.printASCIIPacket( (uint8_t*) buff, bufffersize);
  digitalWrite(LED1, HIGH);

  if (LT.transmit( (uint8_t*) buff, TXPacketL, 10000, TXpower, WAIT_TX))
  {
    digitalWrite(LED1, LOW);
    return true;
  }
  else
  {
    return false;
  }
}

void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}

/************************ Define runtime Arduino functions *********************************/
void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(Program_Version));
  Serial.println();
  Serial.println(F("5_LoRa_TX_Sleep_Timed_Wakeup_Atmel Starting"));

  SPI.begin();
  dht.begin();
  aqs.init(QUALITY_SENSOR);       //Initialisation du capteur Qualité de l'air

 
  // initialisation première valeur du capteur qualité de l'air.
  aqs.last_vol = aqs.first_vol;
  aqs.first_vol = analogRead(QUALITY_SENSOR);
  aqs.counter = 0;
  aqs.timer_index = 1;

  pinMode(LED1, OUTPUT);                        //setup pin as output for indicator LED
  led_Flash(2, 125);                            //two quick LED flashes to indicate program start
  setup_watchdog(6);

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    led_Flash(2, 125);
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                            //long fast speed flash indicates device error
    }
  }

  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);

  Serial.print(F("Transmitter ready - TXBUFFER_SIZE "));
  Serial.println(TXBUFFER_SIZE);
  Serial.println();

}

void loop()
{
  payload_t payload;

  payload.tmp = dht.readTemperature(false ,true);
  payload.hum = dht.readHumidity();
  payload.aq = aqs.slope();
  qualityTime = millis();
  Serial.print(F("TEMP : "));
  Serial.print(payload.tmp);
  Serial.print(F("\t HUM : "));
  Serial.print(payload.hum);
  Serial.print(F("\t AirQ : "));
  Serial.println(payload.aq);
    
  digitalWrite(LED1, HIGH);
  Serial.print(TXpower);
  Serial.print(F("dBm "));
  Serial.print(F("TestPacket1> "));
  // Serial.flush();

  if (Send_Data_Packet(payload))
  {
    packet_is_OK();
  }
  else
  {
    packet_is_Error();
  }
  Serial.println();
  delay(packet_delay);

  LT.setSleep(CONFIGURATION_RETENTION);                        //preserve register settings in sleep.  
  Serial.println(F("Sleeping zzzzz...."));
  Serial.println();
  Serial.flush();
  digitalWrite(LED1, LOW);

  sleepNode(SECONDS_SLEEPING);                                            //goto sleep for 15 seconds
  
  wakeUpFlag = true;
  Serial.println(F("Awake !"));
  Serial.flush();
  digitalWrite(LED1, HIGH);
  LT.wake();

  Serial.print(TXpower);
  Serial.print(F("dBm "));
  Serial.print(F("TestPacket2> "));
  Serial.flush();
}

ISR(TIMER2_OVF_vect){
  {
    if(wakeUpFlag){ //set 2 seconds as a detected duty
      wakeUpFlag = false;
      aqs.last_vol=aqs.first_vol;
      aqs.first_vol=analogRead(QUALITY_SENSOR);
      aqs.counter=0;
      aqs.timer_index=1;
      PORTB=PORTB^0x20;
    }else{
      aqs.counter++; 
    }
  }
}