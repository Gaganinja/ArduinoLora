/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 06/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup. Some pins such as DIO2,
//DIO3, BUZZER may not be in used by this sketch so they do not need to be
//connected and should be set to -1.

// ********* SLEEP SETTINGS *************
#define ENABLE_SLEEP_MODE
#define SECONDS_SLEEPING 15

// ********* RADIO SETTINGS ***************
#define NSS 7
#define RFBUSY 3
#define NRESET A0
#define LED1 A4
#define DIO1 5
#define DIO2 -1                 //not used 
#define DIO3 -1                 //not used
#define RX_EN -1                //pin for RX enable, used on some SX1280 devices, set to -1 if not used
#define TX_EN -1                //pin for TX enable, used on some SX1280 devices, set to -1 if not used 
#define BUZZER -1

#define LORA_DEVICE DEVICE_SX1280                //we need to define the device we are using  

//LoRa Modem Parameters
const uint32_t Frequency = 2425000000;           //frequency of transmissions
const int32_t Offset = 0;                        //offset frequency for calibration purposes
const uint8_t Bandwidth = LORA_BW_0800;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;        //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate

const int8_t TXpower = 10;                       //Power for transmissions in dBm
const uint16_t packet_delay = 100;              //mS delay between packets
#define TXBUFFER_SIZE 32
// ********* END OF RADIO SETTINGS ************


// ********* DHT22 SETTINGS ****************
#define DHTPIN 8                      // Changer le pin sur lequel est branché le DHT
#define DHTTYPE DHT22                 // DHT 22  (AM2302)
// ********* END OF DHT22 SETTINGS ****************


// ********* AIR QUALITY SENSOR SETTINGS ****************
#define QUALITY_TIME 300000           //Temps entre deux affichages des valeurs sonores et qualité de l'air
#define QUALITY_SENSOR A3             //PIN Capteur qualité de l'air
// ********* AIR QUALITY SENSOR SETTINGS ****************