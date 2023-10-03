#include <Arduino.h>
#include <WString.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include <lmic.h>
#include <hal/hal.h>
#include <LoraMessage.h>

#ifndef CUSTOM_LORA_FQZ
#define CUSTOM_LORA_FQZ "EU 868 MHz"
#endif

// ----- Define Pins ----- //
#define I2C_SDA 8 // on teleAgriCulture Board V2.0 I2C_5V SDA is GPIO 15
#define I2C_SCL 9 // on teleAgriCulture Board V2.0 I2C_5V SCL is GPIO 16

#define SW_3V3 42 // board Power switch

#define LORA_SPI_HOST SPI2_HOST
#define LORA_SPI_DMA_CHAN SPI_DMA_DISABLED
#define LORA_CS 10
#define LORA_MOSI 11
#define LORA_SCLK 12
#define LORA_MISO 13
#define LORA_RST 17
#define LORA_DI0 18
#define LORA_DI1 14 // has to be briged on the old board
#define UNUSED_PIN 0xFF
#define LORA_PIN_RXTX UNUSED_PIN

#define uS_TO_S_FACTOR 1000000UL    /* Conversion factor for micro seconds to seconds */
#define uS_TO_MIN_FACTOR 60000000UL /* Conversion factor for micro seconds to minutes */
#define mS_TO_MIN_FACTOR 60000UL    /* Conversion factor for milli seconds to minutes */

// Pin mapping
static const int spiClk = 1000000; // 10 MHz
const lmic_pinmap lmic_pins = {
    .nss = LORA_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST,
    .dio = {LORA_DI0, LORA_DI1, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 14,
    .spi_freq = 1000000,
};

const unsigned long TIMEOUT = 2500;

// static osjob_t sendjob;
uint8_t dev_eui[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t app_eui[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t app_key[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

String lora_fqz = CUSTOM_LORA_FQZ;
String OTAA_APPEUI = "0000000000000000";                 // TTN --> msb first
String OTAA_DEVEUI = "70B3D57ED005D969";                 // TTN --> msb first
String OTAA_APPKEY = "6031C51A53AD955C32D7851BDADA997C"; // TTN --> msb first

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool loraJoined = false;

// ----- Deep Sleep LORA related -----//
RTC_DATA_ATTR u4_t RTC_LORAWAN_netid = 0;
RTC_DATA_ATTR devaddr_t RTC_LORAWAN_devaddr = 0;
RTC_DATA_ATTR u1_t RTC_LORAWAN_nwkKey[16];
RTC_DATA_ATTR u1_t RTC_LORAWAN_artKey[16];
RTC_DATA_ATTR u4_t RTC_LORAWAN_seqnoUp = 0;
RTC_DATA_ATTR u4_t RTC_LORAWAN_seqnoDn;
RTC_DATA_ATTR u1_t RTC_LORAWAN_dn2Dr;
RTC_DATA_ATTR u1_t RTC_LORAWAN_dnConf;
RTC_DATA_ATTR s1_t RTC_LORAWAN_adrTxPow;
RTC_DATA_ATTR u1_t RTC_LORAWAN_txChnl;
RTC_DATA_ATTR s1_t RTC_LORAWAN_datarate;
RTC_DATA_ATTR u2_t RTC_LORAWAN_channelMap;
RTC_DATA_ATTR s2_t RTC_LORAWAN_adrAckReq;
RTC_DATA_ATTR u1_t RTC_LORAWAN_rx1DrOffset;
RTC_DATA_ATTR u1_t RTC_LORAWAN_rxDelay;

#if (CFG_eu868)
RTC_DATA_ATTR u4_t RTC_LORAWAN_channelFreq[MAX_CHANNELS];
RTC_DATA_ATTR u2_t RTC_LORAWAN_channelDrMap[MAX_CHANNELS];
RTC_DATA_ATTR u4_t RTC_LORAWAN_channelDlFreq[MAX_CHANNELS];
RTC_DATA_ATTR band_t RTC_LORAWAN_bands[MAX_BANDS];
#endif

// https://www.loratools.nl/#/airtime
// Saves the LMIC structure during DeepSleep
RTC_DATA_ATTR lmic_t RTC_LMIC;
const unsigned TX_INTERVAL = 30U;

bool loraJoinFailed = false;
bool loraDataTransmitted = false;

bool freshBoot = true; // fresh start - not loading LMIC Config

String upload = "LORA";
int upload_interval = 2; // upload interval in minutes
bool lora_ADR = false;   // use ADR (for mobile nodes, needs more air time)
