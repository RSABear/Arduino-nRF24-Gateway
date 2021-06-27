#include <SPI.h>
#include "RF24.h"            // https://github.com/nRF24/RF24/
#include <printf.h>          // look in Arduino/libraries/RF24 for the printf.h file
#include <TimeLib.h>         // https://github.com/PaulStoffregen/Time

// Node ID has nothing to do with the nRF24 Radio addressing, it is used in my protocol
#define Gateway_ID    00       // Node 0 = Gateway ID
#define Broadcast_ID  01       // Node 1 = Broadcast ID
//#define my_Node_ID   20        // Node ID 20 to 255 is for the clients
//#define my_Node_ID  28       // Test Node 2
//#define my_Node_ID  56       // Test Node 3      

// I used the following libraries and code to construct my programme
// https://sandervandevelde.wordpress.com/2016/03/06/using-nrf24l01-modules-between-arduinos/
// https://github.com/SensorsIot/NRF24L01-Bi-Directional-Communication/blob/master/NRF24L01-Bi-Directional-Communication.ino
// https://tmrh20.blogspot.com/
// https://www.instructables.com/Coding-Timers-and-Delays-in-Arduino/
// https://nrf24.github.io/RF24/index.html
// https://www.pjrc.com/teensy/td_libs_Time.html

// on Nano or Mini Pro
// |--------------------|
// |    UPSIDE      8 7 |
// |     PINS       6 5 |\
// |      ON        4 3 |\\
// |  OTHER SIDE    2 1 |\\
// |--------------------|\\
//                     \  \
// # DESC COLOR  Arduino port
// 1 GND  BLACK           GND
// 2 VCC  RED           3.3V!
// 3 CE   ORANGE           10
// 4 CSN  YELLOW            9
// 5 SCK  GREEN            13
// 6 MOSI BLUE             11
// 7 MISO VIOLET           12
// 8 LED  --|<|--[|||]-----Vss/GND  Optional
// 9 LED  --|<|--[|||]-----Vss/GND  Optional

// Select one of these, must match Node Settings
//#define DATARATE RF24_2MBPS
#define DATARATE RF24_1MBPS
//#define DATARATE RF24_250KBPS

// Select one of these
#define RF_POWER RF24_PA_MIN
//#define RF_POWER RF24_PA_LOW
//#define RF_POWER RF24_PA_MED
//#define RF_POWER RF24_PA_HIGH

// User Defines
#define SERIAL_DEBUG
//#define SERIAL_DEBUG_TIMES
//#define SERIAL_MESSAGE_STATS
//#define POLL_DEBUG
//#define TIME_DEBUG
//#define TELEMETRY_DEBUG
//#define PRINT_STATS
//#define PRINT_RX
//#define PRINT_TX

/* Hardware configuration: Set up nRF24L01 radio /*
/* on SPI bus plus pins 10 & 9 */
RF24 radio(10,9);

// These addresses work like a point to point connection  
uint8_t addresses[2][6] = {"10101","20202"};

#define  LED_1            8         // Green (pulse)
#define  LED_2            7         // Red   (Data RX/TX)

// Timer & LED's
#define ON  1
#define OFF 0

#define LED_ON  0              // Current sink, connect cathode to Vss
#define LED_OFF 1

#define  LED_INTERVAL_1   1000
uint32_t led_time_1     = 0;
bool     LED_1_State    = ON;

#define  LED_INTERVAL_2   250
uint32_t led_time_2     = 0;
bool     LED_2_State    = OFF;

uint32_t broadcast_interval  = 5000;
uint32_t time_4              = 0;

// Create two data buffers for radio rx and tx data
#define BUFFER_LEN   32              // 
uint8_t radioRXbuffer[BUFFER_LEN];   // RX Buffer
uint8_t radioTXbuffer[BUFFER_LEN];   // TX Buffer

// Protocol Header Data TX
struct protoCol_TX_HEADER{ 
  uint8_t   node_id     = Gateway_ID;
  uint8_t   id_to       = 0;
  uint8_t   type        = 0;
  uint16_t  message_id  = 0;  
};
protoCol_TX_HEADER messageHEADER_RX;

// Protocol Header Data RX
struct protoCol_RX_HEADER{ 
  uint8_t   node_id     = 0;
  uint8_t   id_to       = 0;
  uint8_t   type        = 0;
  uint16_t  message_id  = 0;  
};
protoCol_RX_HEADER messageHEADER_TX;


struct pollTXData{
  uint32_t  txData    = 0;
  uint32_t  rxData    = 0;
};
pollTXData pollTX;

struct pollRXData{
  uint32_t  txData    = 0;
  uint32_t  rxData    = 0;
};
pollRXData pollRX;

struct timeData{
  time_t  tx_time    = now();
};
timeData nt_Time;               // Time

struct message_Stats{ 
  uint16_t  tx_vol         = 0;
  uint16_t  rx_vol         = 0;
  uint16_t  tx_time        = 0;
  uint16_t  tx_time_end    = 0;
  uint16_t  tx_time_start  = 0;
  uint16_t  rx_time        = 0;
  uint16_t  rx_time_end    = 0;
  uint16_t  rx_time_start  = 0;
};
message_Stats nodeStats;

// Print Message Stats Timer
#define   INTERVAL_STATS     10000      // For testing, send data every x000ms
uint32_t  time_3           = 0;
bool      print_stats      = false;

// Define variables for messages
bool        msgComplete    = false;

// Define Protocol type requests 
#define POLL_REQUEST            10       // System testing, sends number and message id
#define POLL_RESPONSE           11       // System testing, receives the same number and message id
#define DATA_MESSAGE_SEND       12       
#define DATA_MESSAGE_DELIVERY   13       
#define TIME_REQUEST            14       // Time Request for nRF24 time sync message
#define TIME_RESPONSE           15       // Time sync message response
#define TX_BROADCAST            0xFF       // All node Broadcast Message

// Declare protocol variables
uint16_t  tx_poll_message_id  = 0;
uint16_t  tx_time_message_id  = 0;
uint16_t  tx_broadcast_message_id  = 0;

/* ************************************************************************************************************ */
void setup() {

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }  
  printf_begin();
  printf_P(PSTR("RF24 Gateway ID:%02u\n\r"),messageHEADER_TX.node_id);

  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  digitalWrite(LED_1, LED_OFF);
  digitalWrite(LED_2, LED_OFF);
 
  radio.begin();

  // Addressing
  radio.setAutoAck(false);
  radio.openReadingPipe(1,addresses[0]);

  // Radio Settings
  radio.setPALevel(RF_POWER);
  radio.setDataRate(DATARATE);
  radio.setChannel(0x7F);

  // Set the time
  setTime(8, 00, 00 ,01 ,01 ,1980);

  // Setup the sensors - Optional on Master
  
  radio.startListening();
    
}

/* ************************************************************************************************************ */
void loop() {

  // LED 1 Pulse
  if ((millis() - led_time_1) >= LED_INTERVAL_1) {
      led_time_1 = millis();
      if (LED_1_State){
        LED_1_State = OFF;
        digitalWrite(LED_1, LED_OFF);
      }
      else{
        LED_1_State = ON;
        digitalWrite(LED_1, LED_ON);
      }
    }

  // LED 2 RX/TX, switch the LED off after the on Interval
  if (LED_2_State && ((millis() - led_time_2) >= LED_INTERVAL_2)) {
      LED_2_State = OFF;
      digitalWrite(LED_2, LED_OFF);
    }

  #if defined (SERIAL_MESSAGE_STATS)
    // Display Message Stats
    if(millis() >= time_3 + INTERVAL_STATS){
      time_3 += INTERVAL_STATS;
      print_stats = true;
    }
  #endif

  // Send an random all Node Broadcast Message
  if(millis() >= time_4 + broadcast_interval){
    broadcast_interval = random(10000, 15000); 
    time_4 += broadcast_interval;
    tx_broadcast();
  }

  if( radio.available()){       // Check is we have received a message

    // Blink LED 2
    LED_2_State = ON;
    digitalWrite(LED_2, LED_ON);
    led_time_2 = millis();
    
    nodeStats.rx_time_start = micros();
    nodeStats.rx_vol++;                   // Update the Stats
    
    while (radio.available()) {
      radio.read(&radioRXbuffer, sizeof(radioRXbuffer) + 1);
    }

    // Copy the protocol structure from the RX buffer
    memcpy (&messageHEADER_RX, &radioRXbuffer, sizeof(messageHEADER_RX));
    
    messageHEADER_TX.id_to = messageHEADER_RX.node_id;               // Set the ID of the Node we are replying to
    messageHEADER_TX.message_id = messageHEADER_RX.message_id;       // Always reply with the message ID
    nodeStats.rx_time_end = micros();

    // Did we receive a broadcast message from a node?
    if (messageHEADER_RX.type == TX_BROADCAST){          // Did we receive a broadcast message?

      printf_P(PSTR("NODE BROADCAST RECEIVED ["));
      printf_P(PSTR("%04X]\n\r"),messageHEADER_RX.message_id);
      
    }
    
    if (Gateway_ID == messageHEADER_RX.id_to){        // If the message is for us, deal with it

      radio.stopListening();
      
      // Switch in the Request Type
      switch (messageHEADER_RX.type) {
          case POLL_REQUEST:
            poll_response();
            break;
          case DATA_MESSAGE_SEND:    
            break;
          case TIME_REQUEST:
            time_response();           
            break;
        }

      radio.startListening();


     } // New Message to deal with
     
  }

  if (msgComplete) {

    msgComplete = false;

    // This is a good place to print debug messages
    #if defined (SERIAL_DEBUG_TIMES)
      times_debug();
    #endif
 
    #if defined (PRINT_STATS)
      print_stats();
    #endif

    }
 
}

/* ************************************************************************************************************ */
void tx_broadcast()
{

    //printf_P(PSTR("tx_broadcast()\n\r"));
    clear_radio_buffers();
    
    nodeStats.tx_time_start = micros();
    radio.stopListening();

    // Set the Header information
    messageHEADER_TX.id_to = Gateway_ID;                           // Set our ID here as well
    messageHEADER_TX.type = TX_BROADCAST;                          // TX_BROADCAST
    messageHEADER_TX.message_id = random(0xFFFF);                  // Create a funky ID
    tx_broadcast_message_id = messageHEADER_TX.message_id;         // Save the message id


    // Copy the protocol structure to the TX buffer
    memcpy (&radioTXbuffer, &messageHEADER_TX, sizeof(messageHEADER_TX));

    radio.openWritingPipe(addresses[0]);
    radio.write(&radioTXbuffer, sizeof(radioTXbuffer) + 1);
    
    nodeStats.tx_vol++;                   // Update the Stats
    nodeStats.tx_time_end = micros();
    msgComplete = true;

    radio.startListening();

    // Show the outgoing message header
    print_tx_message_header();

}

/* ************************************************************************************************************ */
void time_response()
{

    //printf_P(PSTR("time_request()\n\r"));
  
    // Copy the protocol structure to the TX buffer
    messageHEADER_TX.type = TIME_RESPONSE;                          // TIME_REQUEST => TIME_RESPONSE
    
    memcpy (&radioTXbuffer, &messageHEADER_TX, sizeof(messageHEADER_TX));

    // Show the received header data
    print_rx_message_header();

    // Place the time into the variable
    nt_Time.tx_time = now();
    memcpy (&radioTXbuffer[sizeof(messageHEADER_TX)], &nt_Time.tx_time, sizeof(nt_Time.tx_time));

    nodeStats.tx_time_start = micros();
    radio.openWritingPipe(addresses[0]);

    radio.write(&radioTXbuffer, sizeof(radioTXbuffer) + 1);
    
    nodeStats.tx_vol++;                   // Update the Stats
    nodeStats.tx_time_end = micros();
    msgComplete = true;

    // Show the outgoing message header
    print_tx_message_header();

  }

/* ************************************************************************************************************ */
void poll_response()
{

    // printf_P(PSTR("poll_request()\n\r"));
    nodeStats.tx_time_start = micros();
    
    // Copy the protocol structure to the TX buffer
    messageHEADER_TX.type = POLL_RESPONSE;                           // POLL_REQUEST => POLL_RESPONSE

    // Copy the protocol structure to the TX buffer
    memcpy (&radioTXbuffer, &messageHEADER_TX, sizeof(messageHEADER_TX));

    // Copy the data out of the Buffer
    memcpy (&pollRX, &radioRXbuffer[sizeof(messageHEADER_RX)], sizeof(pollRX));

    // Now we can print the Header and the Data
    print_rx_message_header();
    
    // Swap the incomming data from tx to rx, just to do something :>)
    pollTX.rxData = pollRX.txData;
    pollTX.txData = 0;
    
    // Add the Data
    memcpy (&radioTXbuffer[sizeof(messageHEADER_TX)], &pollTX, sizeof(pollTX));
    radio.openWritingPipe(addresses[0]);
    
    radio.write(&radioTXbuffer, sizeof(radioTXbuffer) + 1);
    nodeStats.tx_vol++;                   // Update the Stats
    nodeStats.tx_time_end = micros();
    msgComplete = true;

    // Show the outgoing message header
    print_tx_message_header();

}

/* ************************************************************************************************************ */
void printTime(){
  
  // YYYY-MM-DD 
  printf_P(PSTR("%04d"),year());
  printf_P(PSTR("-%02d"),month());
  printf_P(PSTR("-%02d"),day());
  // HH:mm:ss 
  printf_P(PSTR(" %02d"),hour());
  printf_P(PSTR(":%02d"),minute());
  printf_P(PSTR(":%02d"),second());
   
}

/* ************************************************************************************************************ */
void clear_radio_buffers()
{
    for (uint8_t i = 0; i <= sizeof(radioTXbuffer); i++) {
      radioTXbuffer[i] = 0;
    }
    for (uint8_t i = 0; i <= sizeof(radioRXbuffer); i++) {
      radioRXbuffer[i] = 0;
    }
}

/* ************************************************************************************************************ */
void dump_radio_buffers()
{

  print_rx_buffer();
  print_tx_buffer();
  
}

/* ************************************************************************************************************ */
void print_rx_buffer()
{
    
    printf_P(PSTR("RX Buffer: "));
    for (uint8_t i = 0; i <= (sizeof(radioRXbuffer)-1); i++) {
      printf_P(PSTR("%02X "), radioRXbuffer[i]);
    }
    printf_P(PSTR("\n\r"));
  
}

/* ************************************************************************************************************ */
void print_tx_buffer()
{
    printf_P(PSTR("TX Buffer: "));
    for (uint8_t i = 0; i <= (sizeof(radioTXbuffer)-1); i++) {
      printf_P(PSTR("%02X "), radioTXbuffer[i]);
    }
    printf_P(PSTR("\n\r"));
}

/* ************************************************************************************************************ */
void print_rx_message_header()
{
  
    printf_P(PSTR("RX MESG_ID: %04X "),messageHEADER_RX.message_id);
    printf_P(PSTR("FROM: %02u "),messageHEADER_RX.node_id);
    printf_P(PSTR("TO: %02u "),messageHEADER_RX.id_to);
    printf_P(PSTR("TYPE: %02X "),messageHEADER_RX.type);
    switch (messageHEADER_RX.type) {
        case POLL_REQUEST:
          printf_P(PSTR("(POLL_REQUEST) DATA ["));
          printf_P(PSTR("txData: %04X "), pollRX.txData);
          printf_P(PSTR("rxData: %04X"), pollRX.rxData);
          break;
        case TIME_REQUEST:
          printf_P(PSTR("(TIME_REQUEST) ["));
          break;
        case TX_BROADCAST: 
          printf_P(PSTR("(TX_BROADCAST) ["));
          break;
      }
    printf_P(PSTR("]\n\r"));
    
}

/* ************************************************************************************************************ */
void print_tx_message_header()
{
    printf_P(PSTR("TX MESG_ID: %04X "),messageHEADER_TX.message_id);
    printf_P(PSTR("FROM: %02u "),messageHEADER_TX.node_id);
    printf_P(PSTR("TO: %02u "),messageHEADER_TX.id_to);
    printf_P(PSTR("TYPE: %02X "),messageHEADER_TX.type);
    switch (messageHEADER_TX.type) {
        case POLL_RESPONSE:
          printf_P(PSTR("(POLL_RESPONSE) DATA ["));
          printf_P(PSTR("txData: %04X "), pollTX.txData);
          printf_P(PSTR("rxData: %04X"), pollTX.rxData);
          break;
        case TIME_RESPONSE:
          printf_P(PSTR("(TIME_RESPONSE) ["));
          //printf_P(PSTR("Time: %lu "), nt_Time.tx_time );
          printf_P(PSTR("Time: "));
          printTime(nt_Time.tx_time);
          break;
        case TX_BROADCAST:
          printf_P(PSTR("(TX_BROADCAST) ["));
          printf_P(PSTR("%04X"),messageHEADER_TX.message_id);
          break;
      }    
    printf_P(PSTR("]\n\r"));
}

/* ************************************************************************************************************ */
void printTime(time_t t){
  
  // YYYY-MM-DD 
  printf_P(PSTR("%04d"),year(t));
  printf_P(PSTR("-%02d"),month(t));
  printf_P(PSTR("-%02d"),day(t));
  // HH:mm:ss 
  printf_P(PSTR(" %02d"),hour(t));
  printf_P(PSTR(":%02d"),minute(t));
  printf_P(PSTR(":%02d"),second(t));
   
}
/* ************************************************************************************************************ */
void time_calcs(){
  
  nodeStats.tx_time = nodeStats.tx_time_end - nodeStats.tx_time_start;
  nodeStats.rx_time = nodeStats.rx_time_end - nodeStats.rx_time_start;

  }

/* ************************************************************************************************************ */
void times_debug()
{

  time_calcs();

  printf_P(PSTR("  TX time: %u us\n\r"),nodeStats.tx_time);
  printf_P(PSTR("  RX time: %u us\n\r"),nodeStats.rx_time);

}

/* ************************************************************************************************************ */
