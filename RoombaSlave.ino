// RoombaSlave.ino
// Experimental sketch to control iRobot Roomba 880
// - runs on Arduino Pro Mini (5V) under the robot's handle
// - receives commands via attached XL4432-SMT transceiver
// - SCI communication according to iRobot Roomba 500 Open Interface Specification
//
// rev 2.0 - 2015.12.??
//    - uses RHReliableDatagram class for msg acknowledgement and retransmission
//
// rev 1.0 - 2015.07.18
//    - initial version
//    - uses RF22 driver by RadioHead http://www.airspayce.com/mikem/arduino/RadioHead/
//
// boredman@boredomprojects.net

#include <RHReliableDatagram.h>
#include <RH_RF22.h>
#include <SPI.h>

#define MY_ADDRESS 5
#define MASTER_ADDRESS 7

#define RF22_SDN_PIN 3
#define RMB_WKUP_PIN A3
#define RMB_VBAT_PIN A5

// instance of the radio driver
RH_RF22 rf22_driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram radio(rf22_driver, MY_ADDRESS);

uint8_t buf[RH_RF22_MAX_MESSAGE_LEN];

//const int irLedPin = 6;
//uint8_t irLedState;

void setup() 
{
  pinMode(RMB_WKUP_PIN, OUTPUT);
  digitalWrite(RMB_WKUP_PIN, HIGH);

  pinMode(RF22_SDN_PIN, OUTPUT);
  digitalWrite(RF22_SDN_PIN, HIGH);
  delay(500);
  digitalWrite(RF22_SDN_PIN, LOW);
  delay(500);

  Serial.begin(115200);
  Serial.setTimeout(2); // 2 ms is much longer than the duration of 1 byte (10/115200=86us)
                        // for some reason, 1 ms still caused splitting reads
 
  if (radio.init())
  {
  // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
    rf22_driver.setModemConfig(RH_RF22::GFSK_Rb9_6Fd45);
    rf22_driver.setTxPower(RH_RF22_TXPOW_14DBM);  // 1, 2, 5, *8*, 11, 14, 17, 20
  }

//pinMode(irLedPin, OUTPUT);
//analogWrite(irLedPin, 0);
//irLedState = 0;
//digitalWrite(irLedPin, irLedState);
}


void loop()
{
  static unsigned int velocity, radius;
  static unsigned int velocity_duration, radius_duration;
  static unsigned long drive_start_time;

  // Wait for a message addressed to us from the client
  if( radio.available() )
  {
    uint8_t len = sizeof(buf);
    uint8_t from;
    // receive msg and send acknowledge
    if( radio.recvfromAck(buf, &len, &from) && (from==MASTER_ADDRESS) )
    {
      // process command
      switch(buf[0])
      {
        // report last RSSI
        case 'r'  : buf[0] = rf22_driver.lastRssi();
                    radio.sendto(buf, 1, MASTER_ADDRESS);
                    break;
                    
        // measure battery voltage
        case 'v'  : MeasureBatteryVoltage((int*)buf);
                    radio.sendto(buf, 1, MASTER_ADDRESS);
                    break;
                    
        // wakeup roomba
        case 'W'  : digitalWrite(RMB_WKUP_PIN, LOW);
                    delay(600);
                    digitalWrite(RMB_WKUP_PIN, HIGH);
                    buf[0] = 'W';
                    radio.sendto(buf, 1, MASTER_ADDRESS);
                    break;
                    
        // pass through a general command to Roomba
        case '$'  : Serial.write(&buf[1],len-1);
                    break;
                    
        // execute a drive/turn with timing
        case '~'  : velocity = word( buf[1], buf[2] );
                    radius = word( buf[3], buf[4] );
                    velocity_duration = word( buf[5], buf[6] );
                    radius_duration = word( buf[7], buf[8] );
                    buf[0] = 137;         // DRIVE command
                    Serial.write(buf,5);  // buf[1..4] already contain velocity and radius
                    // record start time
                    drive_start_time = millis();
                    break;
                    
        // get timed drive/turn status
        case '#'  : buf[0] = highByte(velocity_duration);
                    buf[1] = lowByte(velocity_duration);
                    buf[2] = highByte(radius_duration);
                    buf[3] = lowByte(radius_duration);
                    radio.sendto(buf, 4, MASTER_ADDRESS);
                    break;
                    
/*      // toggle IR LED
        case 'l'  : if (irLedState == 0)
                    {
                      irLedState = 1;
                      strcpy((char*)buf,"LED ON");
                    }
                    else
                    {
                      irLedState = 0;
                      strcpy((char*)buf,"LED OFF");
                    }
                    digitalWrite(irLedPin, irLedState);
                    radio.sendto(buf, strlen((char*)buf), MASTER_ADDRESS);
                    break;
*/
        default   : break;
      }
    }
  }

  // radio out whatever Roomba is telling us
  if( Serial.available() )
  {
    byte len = Serial.readBytes(buf, RH_RF22_MAX_MESSAGE_LEN-10);
    // send to master without ack
    radio.sendto(buf, len, MASTER_ADDRESS);
  }

  // handle timed maneuvers here
  if( velocity_duration && ((millis()-drive_start_time) > velocity_duration) )
  {
    // full stop, including angle reset
    buf[0] = 137;
    buf[1] = 0; // velocity = 0
    buf[2] = 0;
    buf[3] = 0; // no turning
    buf[4] = 0;
    Serial.write(buf, 5);
    velocity_duration = 0;
  }

  if( radius_duration && ((millis()-drive_start_time) > radius_duration) )
  {
    // stop turning, preserving forward velocity
    buf[0] = 137;
    buf[1] = highByte(velocity);
    buf[2] = lowByte(velocity);
    buf[3] = 0; // no turning
    buf[4] = 0;
    Serial.write(buf, 5);
    radius_duration = 0;
  }

}


bool MeasureBatteryVoltage(int* vbat)
{
  int raw = analogRead(RMB_VBAT_PIN);
  *vbat = raw * 11 / 305;
  return true;
}


