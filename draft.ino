#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include <lmic.h>
#include <hal/hal.h>
#include <U8x8lib.h>

#define LORA_SCK     5    
#define LORA_MISO    19   
#define LORA_MOSI    27 
#define LORA_SS      18  
#define LORA_RST     14   
#define LORA_DI0     26  
#define LORA_BAND    868E6

#include <MFRC522.h>

#define RFID_SDA 5 
#define RFID_SCK 18 
#define RFID_MOSI 23
#define RFID_MISO 19
#define RFID_RST 22

MFRC522 mfrc522(RFID_SDA, RFID_RST);  // Create MFRC522 instance
#define SS 18 // ESP32 GPIO18 (Pin18) – SX1276 NSS (Pin19) SPI Chip Select Input
#define MOSI 27 // ESP32 GPIO27 (Pin27) – SX1276 MOSI (Pin18) SPI Data Input
#define MISO 19 // ESP32 GPIO19 (Pin19) – SX1276 MISO (Pin17) SPI Data Output
#define SCK 5 // ESP32 GPIO5 (Pin5) – SX1276 SCK (Pin16) SPI Clock Input
#define RST 14 // ESP32 GPIO14 (Pin14) – SX1276 NRESET (Pin7) Reset Trigger Input
#define DIO0 26 // ESP32 GPIO26 (Pin15) – SX1276 DIO0 (Pin8) used by LMIC for IRQ RX_Done & TX_Done
#define DIO1 33 // ESP32 GPIO33 (Pin13) – SX1276 DIO1 (Pin9) used by LMIC for IRQ RX_Timeout
#define DIO2 32 // ESP32 GPIO32 (Pin12) – SX1276 DIO2 (Pin10) not used by LMIC
int current_spi = -1; // -1 - NOT STARTED   0 - RFID   1 - LORA
// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x5F, 0xEC, 0x90, 0x2D, 0x68, 0x3D, 0x5B, 0x28, 0x92, 0x87, 0xF5, 0x79, 0x31, 0xE4, 0xA5, 0x2F };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x43, 0xE1, 0x40, 0x57, 0x4A, 0x83, 0x4A, 0xAE, 0x96, 0xC6, 0x68, 0x82, 0x86, 0x9D, 0xBD, 0x3B };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011435 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "AIX!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20;

const lmic_pinmap lmic_pins = {
    .nss = SS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST,
    .dio = {DIO0, DIO0, DIO2},
};


void setup() {
  Serial.begin(9600);   // Initialize serial communications with the PC
 // while (!Serial);    // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
  Serial.println("Serial started");
  os_runloop();
  Serial.println("runloop finished");
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif
   // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868100000, DR_RANGE_MAP(DR_SF7, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868100000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 868100000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 868100000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 868100000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 868100000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 868100000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868100000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF7;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);
    

}

void loop() {
  
  bool card_present = RFID_check();
  if (card_present) do_send(&sendjob);;
  
}

void spi_select(int which) {
     if (which == current_spi) return;
     SPI.end();
     
     switch(which) {
        case 0:
          SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI);
          Serial.println("Switched to rfid");
          mfrc522.PCD_Init();   
        break;
        case 1:
          Serial.println("Switched to Lora");
          SPI.begin(LORA_SCK,LORA_MISO,LORA_MOSI,LORA_SS);
          LoRa.setPins(LORA_SS,LORA_RST,LORA_DI0);
        break;
     }

     current_spi = which;
}

int RFID_check() {
  spi_select(0);
  // Look for new cards
    Serial.println("looking for rfid tag");
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
    delay(100);
    return false;
  }

  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) {
    return false;
  }

  // Dump debug info about the card; PICC_HaltA() is automatically called
  mfrc522.PICC_DumpToSerial(&(mfrc522.uid));
  
  return true;
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
      spi_select(1);
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

  
