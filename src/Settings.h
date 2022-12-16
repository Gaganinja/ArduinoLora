// ********* SLEEP SETTINGS *************
#define ENABLE_SLEEP_MODE     // Let this in commentary to disable sleep mode
#define SECONDS_SLEEPING 15   // Time sleeping node for each cycle

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

// ********* DHT22 SETTINGS ****************
#define DHTPIN 8                      // Changer le pin sur lequel est branché le DHT
#define DHTTYPE DHT22                 // DHT 22  (AM2302)

// ********* AIR QUALITY SENSOR SETTINGS ****************
#define QUALITY_TIME 300000           //Temps entre deux affichages des valeurs sonores et qualité de l'air
#define QUALITY_SENSOR A3             //PIN Capteur qualité de l'air
