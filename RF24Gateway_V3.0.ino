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

// Select the Radio Frequency
#define radio_channel 0x7A
uint8_t hop_channel = 0x00;

// nRF24L01 buffer size
#define PAYLOAD_SIZE  32

// User Defines
#define SERIAL_DEBUG

/* Hardware configuration: Set up nRF24L01 radio /*
/* on SPI bus plus pins 10 & 9 */
RF24 radio(10,9);

// These addresses work like a point to point connection  
uint8_t addresses[2][6] = {"40401","70702"};

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

// Hop timeout
uint8_t   HOP_INTERVAL         = 500;      // 0.5 sec
uint32_t  hop_time             = 0;

// Create two data buffers for radio rx and tx data
#define BUFFER_LEN   32
uint8_t radioRXbuffer[BUFFER_LEN];   // RX Buffer
uint8_t radioTXbuffer[BUFFER_LEN];   // TX Buffer

#define DAT_LEN   128
#define FRAME_SIZE 32
uint8_t dataBuffer[DAT_LEN];           // Data Buffer Len
uint8_t hop_frame_counter = 0;

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

typedef struct Hop{ 
  uint8_t tx_addr[1][5] = {0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t rx_addr[1][5] = {0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t channel    = 0;
  uint8_t data_rate  = 0;
  uint8_t frames  = 0;
  uint16_t data_length  = 0;
};
Hop HOP_DATA;

// Define Protocol type requests 
#define POLL_REQUEST            10       // System testing, sends number and message id
#define POLL_RESPONSE           11       // System testing, receives the same number and message id
#define DATA_MESSAGE            12       
#define DATA_MESSAGE_DELIVERY   13       
#define TIME_REQUEST            14       // Time Request for nRF24 time sync message
#define TIME_RESPONSE           15       // Time sync message response
#define HOP_REQUEST             16       // Request to Hop Frequency and upload data
#define HOP_RESPONSE            17       // 
#define TX_BROADCAST            0xFF     // All node Broadcast Message

// Define the modes of operation
#define mode_broadcast   00
#define mode_hop         01

// Declare protocol variables
uint16_t  tx_poll_message_id  = 0;
uint8_t rx_mode = mode_broadcast;

/* ************************************************************************************************************ */
void radio_set_mode(uint8_t radio_config){

    switch (radio_config) {
      case 0x00:
          rx_mode = mode_broadcast;
          // Addressing
          radio.stopListening();
          radio.setAutoAck(false);
          radio.openReadingPipe(1,addresses[0]);
          // Radio Settings
          radio.setPALevel(RF_POWER);
          radio.setDataRate(DATARATE);
          radio.setChannel(radio_channel);
          radio.startListening();
        break;
      case 0x01:
          // Addressing
          radio.stopListening();
          radio.setChannel(hop_channel);
          radio.openReadingPipe(1,HOP_DATA.tx_addr[0][0]);
          radio.openWritingPipe(HOP_DATA.rx_addr[0][0]);
          radio.setAutoAck(true);
          radio.setPALevel(RF_POWER);
          radio.setDataRate(DATARATE);
          radio.startListening();
        break;
      default:
        break;
    }
   }
   
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
  radio_set_mode(0);

  // Setup the sensors
  randomSeed(Gateway_ID);

  // Set the time
  setTime(8, 00, 00 ,01 ,01 ,1980);

  // Setup the sensors - Optional on Master
  
    
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

  // Hop Time out, reset back to broadcast mode
  if (rx_mode && ((millis() - hop_time) >= HOP_INTERVAL)) {
    rx_mode = mode_broadcast;
    printf_P(PSTR("HOP TIMEOUT\n\r"));
    }


  // Need to fix this code up again
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

    // Bernard
    while (radio.available()) {
       radio.read(&radioRXbuffer, sizeof(radioRXbuffer) + 1);
      }

    // Most important Debug point
    //print_rx_buffer();

    switch (rx_mode){
      case mode_broadcast:
        rx_message_data();        // Process the message data
        break;
      case mode_hop:
        rx_hop_data();            // Deal with the data reaceived in the data buffer
        break;
        }
  }  
 
}
/* ************************************************************************************************************ */
void rx_message_data(){
  
  //printf_P(PSTR("rx_message_data()\n\r"));

  // Copy the protocol structure from the RX buffer
  memcpy (&messageHEADER_RX, &radioRXbuffer, sizeof(messageHEADER_RX));
  messageHEADER_TX.id_to = messageHEADER_RX.node_id;               // Set the ID of the Node we are replying to
  messageHEADER_TX.message_id = messageHEADER_RX.message_id;       // Always reply with the message ID

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
          //printf_P(PSTR("POLL_REQUEST\n\r"));
          poll_response();
          break;
        case DATA_MESSAGE:
          //printf_P(PSTR("DATA_MESSAGE\n\r"));
          break;
        case TIME_REQUEST:
          //printf_P(PSTR("TIME_REQUEST\n\r"));
          time_response();           
          break;
        case HOP_REQUEST:
          //printf_P(PSTR("HOP_REQUEST\n\r"));
          hop_response();
          break;
        case TX_BROADCAST:
          break;
        default:
         unknown_request();
      }
    radio.startListening();
    
   } // New Message to deal with
}
/* ************************************************************************************************************ */
void rx_hop_data()
{

   //printf_P(PSTR("rx_hop_data()\n\r"));
   
   uint8_t rx_index = (hop_frame_counter * BUFFER_LEN);
   hop_frame_counter++;

   //printf_P(PSTR("Frame %02X of %02X Index: %02X \n\r"),hop_frame_counter, HOP_DATA.frames, rx_index);
 
   memcpy (&dataBuffer[rx_index], &radioRXbuffer, BUFFER_LEN);    
  
   //printf_P(PSTR("Frame %d: "), hop_frame_counter);
   //for (uint8_t i_index = rx_index; i_index <= (rx_index + (BUFFER_LEN-1)); i_index++) {
   //    printf_P(PSTR("%02X "), dataBuffer[i_index]);
   // }
   //printf_P(PSTR("\n\r"));

     // 1 2 3 4 etc...
   if (hop_frame_counter >= HOP_DATA.frames){
      //print_data_buffer();                             // Print the data received during hop
      print_rx_hop();
      rx_mode = mode_broadcast;                          // When done switch back to broadcast mode
     }

   hop_time = millis();       // Reset the timeout time counter

}

/* ************************************************************************************************************ */
void hop_response()
{

    //printf_P(PSTR("hop_response()\n\r"));
    
    // Copy the protocol structure to the TX buffer
    messageHEADER_TX.type = HOP_RESPONSE;                           // HOP_REQUEST => HOP_RESPONSE

    // Copy the protocol structure to the TX buffer
    memcpy (&radioTXbuffer, &messageHEADER_TX, sizeof(messageHEADER_TX));

    // Now we can print the Header and the Data
    print_rx_message_header();

    // Copy the frame data out of the Buffer
    memcpy (&HOP_DATA, &radioRXbuffer[sizeof(messageHEADER_RX)], sizeof(HOP_DATA));

    radio.openWritingPipe(addresses[0]);
    radio.write(&radioTXbuffer, sizeof(radioTXbuffer) + 1);

    // Show the outgoing message header
    print_tx_message_header();

    // Switch into fequency hopped mode for receiving data
    rx_mode = mode_hop;
    hop_frame_counter = 0x00;
    hop_time = millis();       // Start the timeout time counter

}
/* ************************************************************************************************************ */
void tx_broadcast()
{

    //printf_P(PSTR("tx_broadcast()\n\r"));
    clear_radio_buffers();
    
    radio.stopListening();

    // Set the Header information
    messageHEADER_TX.id_to = Gateway_ID;                           // Set our ID here as well
    messageHEADER_TX.type = TX_BROADCAST;                          // TX_BROADCAST
    messageHEADER_TX.message_id = random(0xFFFF);                  // Create a funky ID

    // Copy the protocol structure to the TX buffer
    memcpy (&radioTXbuffer, &messageHEADER_TX, sizeof(messageHEADER_TX));

    radio.openWritingPipe(addresses[0]);
    radio.write(&radioTXbuffer, sizeof(radioTXbuffer) + 1);
    
    radio.startListening();

    // Show the outgoing message header
    print_tx_message_header();

}
/* ************************************************************************************************************ */
void unknown_request()
{

    printf_P(PSTR("unknown_request()\n\r"));
  
    // Show the received header data
    print_rx_message_header();

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

    radio.openWritingPipe(addresses[0]);

    radio.write(&radioTXbuffer, sizeof(radioTXbuffer) + 1);

    // Show the outgoing message header
    print_tx_message_header();

  }

/* ************************************************************************************************************ */
void poll_response()
{

    // printf_P(PSTR("poll_request()\n\r"));
    
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
    
    printf_P(PSTR("RX %02x: "), mode_broadcast);
    for (uint8_t i = 0; i <= (sizeof(radioRXbuffer)-1); i++) {
      printf_P(PSTR("%02X "), radioRXbuffer[i]);
    }
    printf_P(PSTR("\n\r"));
  
}

/* ************************************************************************************************************ */
void print_data_buffer()
{

  //printf_P(PSTR("print_data_buffer()\n\r"));
  
  uint8_t x = 0, y = 0, z = 16;
  uint8_t len = (sizeof(dataBuffer)-1);

  printf_P(PSTR("%04X: "), y);
      
  for (uint8_t i = 0; i <= len; i++) {
    printf_P(PSTR("%02X "), dataBuffer[i]);
    x++;
    if ((x >= z) && (i < len)){
      y++;
      printf_P(PSTR("\n\r%04X: "), (i+1));
      x = 0;
      }
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
        case HOP_REQUEST:
          printf_P(PSTR("(HOP_REQUEST) ["));
          printf_P(PSTR("%04X"),messageHEADER_RX.message_id);
          break;
        case TX_BROADCAST: 
          printf_P(PSTR("(TX_BROADCAST) ["));
          break;
        default:
          printf_P(PSTR("(UNKNOWN_REQUEST) ["));
          break;
      }
    printf_P(PSTR("]\n\r"));
    
}

/* ************************************************************************************************************ */
void print_rx_hop()
{

  printf_P(PSTR("RXDATA TX: "));
  for (uint8_t i = 0; i <= 4; i++) {
     printf_P(PSTR("%02X"),HOP_DATA.tx_addr[0][i]);
   }
  printf_P(PSTR(" RX: "));
  for (uint8_t i = 0; i <= 4; i++) {
     printf_P(PSTR("%02X"),HOP_DATA.rx_addr[0][i]);
   }
  printf_P(PSTR(" CH: %02X"),HOP_DATA.channel);
  printf_P(PSTR(" DR: %02X"),HOP_DATA.data_rate);
  printf_P(PSTR(" FRAMES: %02X"),HOP_DATA.frames);
  printf_P(PSTR(" LEN: %04X\n\r"),HOP_DATA.data_length);
  
}

/* ************************************************************************************************************ */
void print_hop_config()
{
  // Print Hop Information
  printf_P(PSTR(" TX Address: "));
  for (uint8_t i = 0; i <= 4; i++) {
     printf_P(PSTR("%02X "),HOP_DATA.tx_addr[0][i]);
   }
  printf_P(PSTR("\n\r"));
 
  printf_P(PSTR(" RX Address: "));
  for (uint8_t i = 0; i <= 4; i++) {
     printf_P(PSTR("%02X "),HOP_DATA.rx_addr[0][i]);
   }
  printf_P(PSTR("\n\r"));

  printf_P(PSTR("Hop Channel: %02X \n\r"),HOP_DATA.channel);
  printf_P(PSTR("  Data rate: %02X \n\r"),HOP_DATA.data_rate);
  printf_P(PSTR("     Frames: %02X \n\r"),HOP_DATA.frames);
  printf_P(PSTR("Data length: %02X \n\r"),HOP_DATA.data_length);
  
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
        case HOP_RESPONSE:
          printf_P(PSTR("(HOP_RESPONSE) ["));
          printf_P(PSTR("%04X"),messageHEADER_TX.message_id);
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
