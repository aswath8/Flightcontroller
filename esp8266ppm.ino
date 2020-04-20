//https://arduino.esp8266.com/stable/package_esp8266com_index.json

//https://www.arduino.cc/en/Tutorial.StringToIntExample
//https://www.arduino.cc/en/Tutorial/WiFiSendReceiveUDPString

/*
  WiFi UDP Send and Receive String

 This sketch wait an UDP packet on localPort using a WiFi shield.
 When a packet is received an Acknowledge packet is sent to the client on port remotePort

 Circuit:
 * WiFi shield attached

 created 30 December 2012
 by dlf (Metodo2 srl)

 */


#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

int status = WL_IDLE_STATUS;
const char* const ssid = "555"; // Your network SSID (name)
const char* const pass = "00011000"; // Your network password
const uint16_t localPort = 54321; // Local port to listen for UDP packets

char packetBuffer[4][4]; //buffer to hold incoming packet
//int yaw, thrust, roll, pitch;
int pwm[4];
char ReplyBuffer[] = "ack";       // a string to send back
int pin =2;
#define myPin 2
#define myPinBit (1<<2)
WiFiUDP Udp;

unsigned long mtime;
int packetSize;
int i=0;

// ===========================================================


// ===========================================================
//               delayTicks for ticks >= 17
//               by Alfons Mittelmeyer 2018
// -----------------------------------------------------------

void ICACHE_RAM_ATTR delayTicks(int32_t ticks) {

  uint32_t  expire_ticks = asm_ccount() + ticks -13;

  do {
    ticks = expire_ticks - asm_ccount();
  } while(ticks >= 6 );

  asm volatile (        // delay of remaining ticks by branches
    "blti %0, 1 , 0f;"
    "beqi %0, 1 , 0f;"
    "beqi %0, 2 , 0f;"
    "beqi %0, 3 , 0f;"
    "beqi %0, 4 , 0f;"
    "bgei %0, 5 , 0f;"
    "0:          ;" : : "r"(ticks)
   );
}
// ===========================================================


// ===========================================================
//           delay_us, which uses delayTicks
//           (use instead of delayMicroseconds)
//           by Alfons Mittelmeyer 2018
// -----------------------------------------------------------
void ICACHE_RAM_ATTR delay_us(int32_t us) {
  delayTicks(80 * us - 13); // -13 for function call of delayTicks and multiplication and subtraction
}
// ===========================================================



void setup() {
  //Serial.begin(250000);
  //Serial.println();
  //Serial.print(F("Connecting to "));
  //Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print('.');
  }
  //Serial.println();
  //printWifiStatus();

  //Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort); 
  pinMode(myPin,OUTPUT);
  WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 8, myPinBit );
  delay_us(3000);
}

void loop() {
  // if there's data available, read a packet
  packetSize = Udp.parsePacket();
  if (packetSize==16) {
    for(i=0;i<4;i++){
    // read the packet into packetBufffer
    Udp.read(packetBuffer[i], 4);
    packetBuffer[i][4] = 0;
    pwm[i]=atoi(packetBuffer[i]);
    }
    
   WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 4, myPinBit );
    delay_us(350);
    WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 8, myPinBit );
    delay_us(pwm[0]-350);
    
     WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 4, myPinBit );
    delay_us(350);
    WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 8, myPinBit );
    delay_us(pwm[1]-350);
    
     WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 4, myPinBit );
    delay_us(350);
    WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 8, myPinBit );
    delay_us(pwm[2]-350);
    
     WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 4, myPinBit );
    delay_us(350);
    WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 8, myPinBit );
    delay_us(pwm[3]-350);
     
   
   WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 4, myPinBit );
    delay_us(350);
    WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 8, myPinBit );
  }
}


// ===========================================================
//               Measuring time by CCOUNT register
// -----------------------------------------------------------
static inline volatile uint32_t asm_ccount(void) {
    uint32_t r;
    asm volatile ("rsr %0, ccount" : "=r"(r));
    return r;
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
