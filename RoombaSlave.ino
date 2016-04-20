// RoombaSlave.ino
// Experimental sketch to control iRobot Roomba 880
// - runs on Arduino Pro Mini (5V) under the robot's handle
// - receives commands via attached XL4432-SMT transceiver
// - SCI communication according to iRobot Roomba 500 Open Interface Specification
//
// rev 3.0 - 2016.04.20
//    - enabled local driving control based on sensory inputs
//    
// rev 2.0 - 2015.12.20
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

byte buf[RH_RF22_MAX_MESSAGE_LEN];

int BatteryVoltage(void);
void Drive(unsigned int velocity, unsigned int radius);
void ProcessBump(byte packet);
void ProcessRadioMsg(byte *buf, byte len, byte from);

unsigned int velocity, radius;
unsigned int velocity_duration, radius_duration;
unsigned long drive_start_time;
unsigned long sensor_req_time;

bool local_control;
bool stop_local_control;

enum eState { 
  idle, 
  remote_cmd, 
  local_backoff, 
  local_turn 
} state;


/*******************************************************
 * 
 *******************************************************/
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

  local_control = false;
  stop_local_control = false;

  state = idle;
}


/*******************************************************
 * 
 *******************************************************/
void loop()
{
  // receive radio command from Master
  if( radio.available() )
  {
    byte len = sizeof(buf);
    byte from;
    // receive msg and send acknowledge
    if( radio.recvfromAck(buf, &len, &from) && (from==MASTER_ADDRESS) )
    {
      ProcessRadioMsg(buf, len, from);
    }
  }

  // receive serial data from Roomba
  if( Serial.available() )
  {
    byte len = Serial.readBytes(buf, RH_RF22_MAX_MESSAGE_LEN-10);
    if( local_control )
    {
      ProcessBump(buf[0]);
      if( stop_local_control )
      {
        local_control = false;
        stop_local_control = false;
      }
    }
    else
    { // radio out whatever Roomba is telling us
      radio.sendtoWait(buf, len, MASTER_ADDRESS);
    }
  }

  // request sensor status
  if( local_control )
  { // time to send request again?
    if( (millis()-sensor_req_time) > 15 ) //[ms]
    {
      buf[0] = 142; // single read sensors
      buf[1] = 7;   // bumper status
      Serial.write(buf, 2);
      sensor_req_time = millis();
    }
  }


  // handle timed maneuvers here
  if( state == remote_cmd )
  {
    if( velocity_duration && ((millis()-drive_start_time) > velocity_duration) )
    {
      // full stop, including angle reset
      Drive(0,0);
      velocity_duration = 0;
      state = idle;
    }
  
    if( radius_duration && ((millis()-drive_start_time) > radius_duration) )
    {
      // stop turning, preserving forward velocity
      Drive(velocity,0);
      radius_duration = 0;
      state = idle;
    }
  }

}


/*******************************************************
 * 
 *******************************************************/
void ProcessRadioMsg(byte *buf, byte len, byte from)
{
  if( len == 0 )
    return;

  switch(buf[0])
  {
    // report last RSSI
    case 'r'  : buf[0] = rf22_driver.lastRssi();
                radio.sendtoWait(buf, 1, MASTER_ADDRESS);
                break;
                
    // measure battery voltage
    case 'v'  : buf[0] = lowByte( BatteryVoltage() );
                radio.sendtoWait(buf, 1, MASTER_ADDRESS);
                break;
                
    // wakeup roomba
    case 'W'  : digitalWrite(RMB_WKUP_PIN, LOW);
                delay(600);
                digitalWrite(RMB_WKUP_PIN, HIGH);
                break;
                
    // pass through a general command to Roomba
    case '$'  : Serial.write(&buf[1],len-1);
                break;
                
    // execute a drive/turn with timing
    case '~'  : velocity = word(buf[1], buf[2]);
                radius = word(buf[3], buf[4]);
                velocity_duration = word(buf[5], buf[6]);
                radius_duration = word(buf[7], buf[8]);
                if( state == idle )
                {
                  Drive(velocity, radius);
                  // record start time
                  drive_start_time = millis();
                  state = remote_cmd;
                }
                break;
                
    // get timed drive/turn/bump status
    case '#'  : buf[0] = state;
                //buf[0] = bool(velocity_duration);
                //buf[1] = bool(radius_duration);
                radio.sendtoWait(buf, 1, MASTER_ADDRESS);
                break;

    // enable local control of driving based on sensors
    case 'L'  : if( (bool)buf[1] )
                { 
                  if( local_control )
                    break;
                  local_control = true;
                  // send Sensors command
                  buf[0] = 142; // single read sensors
                  buf[1] = 7;   // bumper status
                  Serial.write(buf, 2);
                  // record start time
                  sensor_req_time = millis();
                }
                else
                {
                  if( local_control )
                    stop_local_control = true;
                }
                break;

    default   : break;
  }
}

char hex2nib(byte h)
{
  if( h <= 9 )
    return h + '0';
  else
    return h + 'A'; 
}

/*******************************************************
 * 
 *******************************************************/
void ProcessBump(byte packet)
{
  static byte bumped_side;
  static unsigned long turn_start_time;

  switch( state )
  {
    case idle         : 
    case remote_cmd   : if( (packet & 0x03) != 0 )
                        { // slowly back away
                          Drive(-50,0);
                          // remember the bump direction
                          bumped_side = packet & 0x03;
                          state = local_backoff;
                          velocity_duration = radius_duration = 0;
                        }
                        break;

    case local_backoff: // make sure we are away from wall
                        if( (packet & 0x03) == 0 )
                        {
                          if( bumped_side & 0x01 ) // right side?
                            Drive(100,1);   // turn in place counter-clockwise
                          else
                            Drive(100,-1);  // turn in place clockwise
                          // remember turn start time
                          turn_start_time = millis();
                          state = local_turn;
                        }
                        break;

    case local_turn   : if( (millis()-turn_start_time) > 1000 )  //[ms]
                        { //end turn
                          Drive(0,0);
                          state = idle;
                        }
                        break;

    default           : break;
  }

}


/*******************************************************
 * 
 *******************************************************/
void Drive(unsigned int velocity, unsigned int radius)
{
  byte cmd[5];
  cmd[0] = 137; // drive command
  cmd[1] = highByte(velocity);
  cmd[2] = lowByte(velocity);
  cmd[3] = highByte(radius);
  cmd[4] = lowByte(radius);
  Serial.write(cmd, 5);
}


/*******************************************************
 * 
 *******************************************************/
int BatteryVoltage()
{
  int raw = analogRead(RMB_VBAT_PIN);
  return (raw * 11 / 305);
}


