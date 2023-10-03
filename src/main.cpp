/*\
 *
 * Copyright (c) 2023 artdanion
 * based also on https://jackgruber.github.io/2020-04-13-ESP32-DeepSleep-and-LoraWAN-OTAA-join/ but extended
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <Arduino.h>
#include <board_config.h>

#include <FS.h>
#include "SPIFFS.h"
#include <WString.h>

#include <ArduinoJson.h>

#include <SPI.h>
#include <driver/spi_master.h>

void GPIO_wake_up(void);

// ****************** Lora functions ******************* //

void os_getArtEui(u1_t *buf)
{
   std::copy(app_eui, app_eui + 8, buf);
}
void os_getDevEui(u1_t *buf)
{
   std::copy(dev_eui, dev_eui + 8, buf);
}
void os_getDevKey(u1_t *buf)
{
   std::copy(app_key, app_key + 16, buf);
}

void lora_sendData(void);
void do_send(LoraMessage &message);
void onEvent(ev_t ev);
void convertTo_LSB_EUI(String input, uint8_t *output);
void convertTo_MSB_APPKEY(String input, uint8_t *output);
void saveLORA_State(void);
void loadLORA_State(void);
void LoraWANPrintLMICOpmode(void);
void saveLMICToRTC(int deepsleep_sec);
void loadLMICFromRTC();

// ****************** Lora functions end ******************* //

int temperatur = 23;
bool sendDataLoRa = false;
bool gotoSleep = false;

void setup()
{
   // reset Pins after holding them during deep sleep
   gpio_reset_pin((gpio_num_t)SW_3V3);

   gpio_hold_dis((gpio_num_t)SW_3V3);
   gpio_deep_sleep_hold_dis();

   pinMode(LORA_CS, OUTPUT);
   pinMode(SW_3V3, OUTPUT);

   delay(1000); // for debugging in screen

   Serial.setTxTimeoutMs(5); // set USB CDC Time TX
   Serial.begin(115200);     // start Serial for debuging

   // Print the wakeup reason for ESP32
   GPIO_wake_up();
   esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

   delay(100);

   // Increment boot number and print it every reboot
   ++bootCount;
   Serial.println("\nBoot number: " + String(bootCount));
   Serial.printf("LoRa has joined: %s\n", loraJoined ? "true" : "false");

   if (bootCount == 1 || (bootCount % 720) == 0) // new join once every 24h
      freshBoot = true;
   if (bootCount > 60480) // bootCount resets every 84 days
      bootCount = 0;

   if (upload == "LORA")
   {
      digitalWrite(SW_3V3, HIGH);
      delay(500);

      sendDataLoRa = true;

      convertTo_LSB_EUI(OTAA_DEVEUI, dev_eui);
      convertTo_LSB_EUI(OTAA_APPEUI, app_eui);
      convertTo_MSB_APPKEY(OTAA_APPKEY, app_key);

      // LMIC init
      os_init();

      if (loraJoined && (!freshBoot)) // new join request every 24h because freshBoot gets reseted
      {
         loadLORA_State(); // load the LMIC settings
         delay(200);

         if ((RTC_LMIC.seqnoUp != 0) && CFG_LMIC_EU_like) // just for EU
         {
            Serial.println("\nload air time for channels");
            delay(200);
            loadLMICFromRTC();
            delay(200);
         }
      }
      else
      {
         // Reset the MAC state. Session and pending data transfers will be discarded.
         LMIC_reset();
         LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);
         freshBoot = false;
         loraJoined = false;
         // LMIC_startJoining();
      }
   }
}

void loop()
{
   if (upload == "LORA")
   {
      os_runloop_once(); // Run the LoRaWAN OS run loop

      if (sendDataLoRa) // If ít`s time to send lora data
      {
         temperatur = 22; // Read sensor data
         sendDataLoRa = false;
         gotoSleep = false;

         lora_sendData(); // Send data via LoRa
      }

      if (loraJoinFailed)
      {
         Serial.println("NO Gateway found, restarting...");
         Serial.println("restart ESP32");
         delay(1000);

         ESP.restart();
      }

      const bool timeCriticalJobs = os_queryTimeCriticalJobs(ms2osticksRound((TX_INTERVAL * 1000)));

      if (!timeCriticalJobs && !(LMIC.opmode & OP_TXRXPEND) && loraJoined) // if no transmission is active
      {
         if (gotoSleep)
         {
            Serial.print(F("Can go sleep... "));

            saveLORA_State();
            saveLMICToRTC(TX_INTERVAL);
            delay(200);

            int time_interval = upload_interval * uS_TO_MIN_FACTOR;
            esp_sleep_enable_timer_wakeup(time_interval);

            // Stop SPI
            spi_bus_free(SPI2_HOST);

            // Reset Pins
            gpio_reset_pin((gpio_num_t)SW_3V3);

            pinMode(SW_3V3, OUTPUT);
            digitalWrite(SW_3V3, LOW);

            gpio_hold_en((gpio_num_t)SW_3V3);

            gpio_deep_sleep_hold_en();

            Serial.print("Setup ESP32 to sleep for ");
            Serial.print(upload_interval);
            Serial.println(" minutes");
            Serial.print("wakeup in: ");
            Serial.print(time_interval);
            Serial.println(" MicroSeconds");

            Serial.end();
            Serial.flush();

            delay(100);

            esp_deep_sleep_start(); // Enter deep sleep mode
         }
      }
   }
}

void GPIO_wake_up()
{
   esp_sleep_wakeup_cause_t wakeup_reason;
   wakeup_reason = esp_sleep_get_wakeup_cause();

   uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();
   Serial.print("\nGPIO that triggered the wake up: GPIO ");
   Serial.println((log(GPIO_reason)) / log(2), 0);

   if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER)
   {
      freshBoot = false;
   }
}

void lora_sendData(void)
{
   Serial.print("\nDEVEUI[8]={ ");
   for (int i = 0; i < 8; i++)
   {
      Serial.print("0x");
      Serial.print(dev_eui[i], HEX);
      if (i < 7)
      {
         Serial.print(", ");
      }
   }
   Serial.println(" };");

   Serial.print("APPEUI[8]={ ");
   for (int i = 0; i < 8; i++)
   {
      Serial.print("0x");
      Serial.print(app_eui[i], HEX);
      if (i < 7)
      {
         Serial.print(", ");
      }
   }
   Serial.println(" };");

   Serial.print("APPKEY[16]={ ");
   for (int i = 0; i < 16; i++)
   {
      Serial.print("0x");
      Serial.print(app_key[i], HEX);
      if (i < 15)
      {
         Serial.print(", ");
      }
   }
   Serial.println(" };\n");

   // https://github.com/thesolarnomad/lora-serialization
   // JS decoder example online

   LoraMessage message;

   message.addTemperature(temperatur);

   Serial.println("\nSend Lora Data: ");
   do_send(message);
}

void convertTo_LSB_EUI(String input, uint8_t *output)
{
   // Check if input matches pattern
   if (input.length() != 16)
   {
      Serial.println("Input must be 16 characters long.");
      return;
   }
   for (int i = 0; i < 16; i++)
   {
      if (!isxdigit(input.charAt(i)))
      {
         Serial.println("Input must be in hexadecimal format.");
         return;
      }
   }

   // Convert string to byte array
   for (int i = 0; i < 8; i++)
   {
      String byteStr = input.substring(i * 2, i * 2 + 2);
      output[7 - i] = strtol(byteStr.c_str(), NULL, 16);
   }
}

void convertTo_MSB_APPKEY(String input, uint8_t *output)
{
   // Check if input matches pattern
   if (input.length() != 32)
   {
      Serial.println("Input must be 16 characters long.");
      return;
   }
   for (int i = 0; i < 32; i++)
   {
      if (!isxdigit(input.charAt(i)))
      {
         Serial.println("Input must be in hexadecimal format.");
         return;
      }
   }

   // Convert string to byte array
   for (int i = 0; i < 16; i++)
   {
      String byteStr = input.substring(i * 2, i * 2 + 2);
      output[i] = strtol(byteStr.c_str(), NULL, 16);
   }
}

void onEvent(ev_t ev)
{
   Serial.print(os_getTime());
   Serial.print(": ");
   switch (ev)
   {
   case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
   case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
   case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
   case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
   case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
   case EV_JOINED:
   {
      Serial.println(F("EV_JOINED"));

      u4_t netid = 0;
      devaddr_t devaddr = 0;
      u1_t nwkKey[16];
      u1_t artKey[16];
      LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
      Serial.print("netid: ");
      Serial.println(netid, DEC);
      Serial.print("devaddr: ");
      Serial.println(devaddr, HEX);
      Serial.print("artKey: ");
      for (size_t i = 0; i < sizeof(artKey); ++i)
      {
         Serial.print(artKey[i], HEX);
      }
      Serial.println("");
      Serial.print("nwkKey: ");
      for (size_t i = 0; i < sizeof(nwkKey); ++i)
      {
         Serial.print(nwkKey[i], HEX);
      }
      Serial.println("\n");

      loraJoined = true;
      loraJoinFailed = false;

      saveLORA_State();

      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX

      if (lora_ADR)
      {
         Serial.println("\nuse ADR for LORA (for mobile Nodes)");
         LMIC_setLinkCheckMode(0);
      }
      else
      {
         Serial.println("\nuse ADR for LORA (for mobile Nodes)");
      }
   }
   break;
   /*
       || This event is defined but not used in the code. No
       || point in wasting codespace on it.
       ||
       || case EV_RFU1:
       ||     Serial.println(F("EV_RFU1"));
       ||     break;
       */
   case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      loraJoinFailed = true;
      break;
   case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
   case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
      {
         Serial.println(F("Received ack"));
      }
      if (LMIC.dataLen)
      {
         Serial.print(F("Received "));
         Serial.print(LMIC.dataLen);
         Serial.println(F(" bytes of payload"));
      }
      gotoSleep = true;

      break;
   case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
   case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
   case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
   case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
   case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
   /*
       || This event is defined but not used in the code. No
       || point in wasting codespace on it.
       ||
       || case EV_SCAN_FOUND:
       ||    Serial.println(F("EV_SCAN_FOUND"));
       ||    break;
       */
   case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
   case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
   case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
   case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
   default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
   }
}

void do_send(LoraMessage &message)
{
   if (message.getLength() > 1)
   {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, message.getBytes(), message.getLength(), 0);
      // Serial.println(F("Packet queued\n"));
   }
}

void LoraWANPrintLMICOpmode(void)
{
   Serial.print(F("LMIC.opmode: "));
   if (LMIC.opmode & OP_NONE)
   {
      Serial.print(F("OP_NONE "));
   }
   if (LMIC.opmode & OP_SCAN)
   {
      Serial.print(F("OP_SCAN "));
   }
   if (LMIC.opmode & OP_TRACK)
   {
      Serial.print(F("OP_TRACK "));
   }
   if (LMIC.opmode & OP_JOINING)
   {
      Serial.print(F("OP_JOINING "));
   }
   if (LMIC.opmode & OP_TXDATA)
   {
      Serial.print(F("OP_TXDATA "));
   }
   if (LMIC.opmode & OP_POLL)
   {
      Serial.print(F("OP_POLL "));
   }
   if (LMIC.opmode & OP_REJOIN)
   {
      Serial.print(F("OP_REJOIN "));
   }
   if (LMIC.opmode & OP_SHUTDOWN)
   {
      Serial.print(F("OP_SHUTDOWN "));
   }
   if (LMIC.opmode & OP_TXRXPEND)
   {
      Serial.print(F("OP_TXRXPEND "));
   }
   if (LMIC.opmode & OP_RNDTX)
   {
      Serial.print(F("OP_RNDTX "));
   }
   if (LMIC.opmode & OP_PINGINI)
   {
      Serial.print(F("OP_PINGINI "));
   }
   if (LMIC.opmode & OP_PINGABLE)
   {
      Serial.print(F("OP_PINGABLE "));
   }
   if (LMIC.opmode & OP_NEXTCHNL)
   {
      Serial.print(F("OP_NEXTCHNL "));
   }
   if (LMIC.opmode & OP_LINKDEAD)
   {
      Serial.print(F("OP_LINKDEAD "));
   }
   if (LMIC.opmode & OP_LINKDEAD)
   {
      Serial.print(F("OP_LINKDEAD "));
   }
   if (LMIC.opmode & OP_TESTMODE)
   {
      Serial.print(F("OP_TESTMODE "));
   }
   if (LMIC.opmode & OP_UNJOIN)
   {
      Serial.print(F("OP_UNJOIN "));
   }
}

void saveLMICToRTC(int deepsleep_sec)
{
   Serial.println(F("Save LMIC to RTC"));
   RTC_LMIC = LMIC;

   // ESP32 can't track millis during DeepSleep and no option to advanced millis after DeepSleep.
   // Therefore reset DutyCyles

   unsigned long now = millis();

   // EU Like Bands
#if defined(CFG_eu868)
   Serial.println(F("Reset CFG_LMIC_EU_like band avail"));
   for (int i = 0; i < MAX_BANDS; i++)
   {
      ostime_t correctedAvail = RTC_LMIC.bands[i].avail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
      if (correctedAvail < 0)
      {
         correctedAvail = 0;
      }
      RTC_LMIC.bands[i].avail = correctedAvail;
   }

   RTC_LMIC.globalDutyAvail = RTC_LMIC.globalDutyAvail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
   if (RTC_LMIC.globalDutyAvail < 0)
   {
      RTC_LMIC.globalDutyAvail = 0;
   }
#else
   Serial.println(F("No DutyCycle recalculation function!"));
#endif
}

void loadLMICFromRTC()
{
   Serial.println(F("Load LMIC from RTC"));
   LMIC = RTC_LMIC;
}

// Function to store LMIC configuration to RTC Memory
void saveLORA_State(void)
{
   Serial.println(F("Save LMIC to RTC ..."));
   RTC_LORAWAN_netid = LMIC.netid;
   RTC_LORAWAN_devaddr = LMIC.devaddr;
   memcpy(RTC_LORAWAN_nwkKey, LMIC.nwkKey, 16);
   memcpy(RTC_LORAWAN_artKey, LMIC.artKey, 16);
   RTC_LORAWAN_dn2Dr = LMIC.dn2Dr;
   RTC_LORAWAN_dnConf = LMIC.dnConf;
   RTC_LORAWAN_seqnoDn = LMIC.seqnoDn;
   RTC_LORAWAN_seqnoUp = LMIC.seqnoUp;
   RTC_LORAWAN_adrTxPow = LMIC.adrTxPow;
   RTC_LORAWAN_txChnl = LMIC.txChnl;
   RTC_LORAWAN_datarate = LMIC.datarate;
   RTC_LORAWAN_adrAckReq = LMIC.adrAckReq;
   RTC_LORAWAN_rx1DrOffset = LMIC.rx1DrOffset;
   RTC_LORAWAN_rxDelay = LMIC.rxDelay;

#if (CFG_eu868)
   memcpy(RTC_LORAWAN_channelFreq, LMIC.channelFreq, MAX_CHANNELS * sizeof(u4_t));
   memcpy(RTC_LORAWAN_channelDrMap, LMIC.channelDrMap, MAX_CHANNELS * sizeof(u2_t));
   memcpy(RTC_LORAWAN_channelDlFreq, LMIC.channelDlFreq, MAX_CHANNELS * sizeof(u4_t));
   memcpy(RTC_LORAWAN_bands, LMIC.bands, MAX_BANDS * sizeof(band_t));
   RTC_LORAWAN_channelMap = LMIC.channelMap;
#endif

   Serial.println("LMIC configuration stored in RTC Memory.");
}

// Function to reload LMIC configuration from RTC Memory
void loadLORA_State()
{
   Serial.println(F("Load LMIC State from RTC ..."));

   LMIC_setSession(RTC_LORAWAN_netid, RTC_LORAWAN_devaddr, RTC_LORAWAN_nwkKey, RTC_LORAWAN_artKey);
   LMIC_setSeqnoUp(RTC_LORAWAN_seqnoUp);
   LMIC_setDrTxpow(RTC_LORAWAN_datarate, RTC_LORAWAN_adrTxPow);
   LMIC.seqnoDn = RTC_LORAWAN_seqnoDn;
   LMIC.dnConf = RTC_LORAWAN_dnConf;
   LMIC.adrAckReq = RTC_LORAWAN_adrAckReq;
   LMIC.dn2Dr = RTC_LORAWAN_dn2Dr;
   LMIC.rx1DrOffset = RTC_LORAWAN_rx1DrOffset;
   LMIC.rxDelay = RTC_LORAWAN_rxDelay;
   LMIC.txChnl = RTC_LORAWAN_txChnl;

#if (CFG_eu868)
   memcpy(LMIC.bands, RTC_LORAWAN_bands, MAX_BANDS * sizeof(band_t));
   memcpy(LMIC.channelFreq, RTC_LORAWAN_channelFreq, MAX_CHANNELS * sizeof(u4_t));
   memcpy(LMIC.channelDlFreq, RTC_LORAWAN_channelDlFreq, MAX_CHANNELS * sizeof(u4_t));
   memcpy(LMIC.channelDrMap, RTC_LORAWAN_channelDrMap, MAX_CHANNELS * sizeof(u2_t));
   LMIC.channelMap = RTC_LORAWAN_channelMap;
#endif
   delay(200);
   Serial.println("LMIC configuration reloaded from RTC Memory.");
}
