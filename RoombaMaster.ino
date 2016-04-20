// RoombaMaster.ino
// Experimental sketch to control iRobot Roomba 880
// - runs on Arduino Pro Mini (5V) connected to RaspberryPi's serial port
// - sends commands to Roomba via XL4432-SMT transceiver
// - uses iRobot Roomba 500 Open Interface Specification
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

#define MY_ADDRESS 7
#define SLAVE_ADDRESS 5

#define RF22_SDN_PIN 3

// instance of the radio driver
RH_RF22 rf22_driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram radio(rf22_driver, MY_ADDRESS);

// Dont put this on the stack:
uint8_t buf[RH_RF22_MAX_MESSAGE_LEN];

const int irLedPin = 6;

void setup() 
{
  pinMode(RF22_SDN_PIN, OUTPUT);
  digitalWrite(RF22_SDN_PIN, HIGH);
  delay(500);
  digitalWrite(RF22_SDN_PIN, LOW);
  delay(500);

  Serial.begin(115200);
  Serial.setTimeout(10);
  
  if (!radio.init())
    Serial.println("init failed");
  else
  {
  //Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
    rf22_driver.setModemConfig(RH_RF22::GFSK_Rb9_6Fd45);
    rf22_driver.setTxPower(RH_RF22_TXPOW_14DBM);  // 1, 2, 5, *8*, 11, 14, 17, 20
  }

  pinMode(irLedPin, OUTPUT);
  analogWrite(irLedPin, 0);
}


void loop()
{
  static int velocity = 0;
  static int angle = 0;
  byte cmd, str[2];
  static bool human_mode;

  if( Serial.available() )
  {
    cmd = Serial.read();
    if( cmd == 27 ) //direction keys
    {
      if( (cmd=Serial.readBytes(str,2)) != 2 )
        cmd = -1;
      else if( str[0] == 91 )
        cmd = str[1] - 53;
      else
        cmd = -1;
    }

    human_mode = false;
    
    switch(cmd)
    {
      // general Roomba command (machine mode)
      case '$'  : // get payload data
                  int n;
                  if( (n = Serial.read()) == -1 )
                  {
                    Serial.write('?');  // parameter error
                    break;
                  }
                  if( n > (RH_RF22_MAX_MESSAGE_LEN-1) )
                  {
                    Serial.write('?');  // parameter error
                    break;
                  }
                  buf[0] = '$';
                  if( Serial.readBytes(&buf[1], n) != n )
                  {
                    Serial.write('?');  // parameter error
                    break;
                  }
                  // send it out
                  if( radio.sendtoWait(buf, n+1, SLAVE_ADDRESS) )
                    Serial.write('!');  // ok
                  else
                    Serial.write('x');  // comm error
                  break;

      // execute a drive/turn with timing (machine mode)
      case '~'  : /*
                  buf[0] = '~';
                  buf[1] = highByte(velocity);
                  buf[2] = lowByte(velocity);
                  buf[3] = highByte(radius);
                  buf[4] = lowByte(radius);
                  buf[5] = highByte(velocity_duration);   // 0 = forever
                  buf[6] = lowByte(velocity_duration);
                  buf[7] = highByte(radius_duration);      // 0 = forever
                  buf[8] = lowByte(radius_duration);
                  */
                  buf[0] = '~';
                  if( Serial.readBytes(&buf[1], 8) != 8 )
                  {
                    Serial.write('?');  // parameter error
                    break;
                  }
                  if( radio.sendtoWait(buf, 9, SLAVE_ADDRESS) )
                    Serial.write('!');  // ok
                  else
                    Serial.write('x');  // comm error
                  break;

      // get timed drive/turn status
      case '#'  : buf[0] = '#';  
                  if( radio.sendtoWait(buf, 1, SLAVE_ADDRESS) )
                    Serial.write('!');  // ok
                  else
                    Serial.write('x');  // comm error
                  break;

      // enable local control of driving based on sensors
      case 'L'  : buf[0] = 'L';
                  // convert parameter into number
                  int i;
                  i = 300;  // 3 sec timeout
                  while( ! Serial.readBytes(&buf[1],1) )
                  {
                    if( --i == 0 )
                    {
                      Serial.write('?');  // parameter error
                      break;
                    }
                  }
                  buf[1] -= '0';
                  if( buf[1] < 0 || buf[1] > 1 )
                  {
                    Serial.write('?');  // parameter error
                    break;
                  }
/*                if( Serial.readBytes(&buf[1], 1) != 1 )
                  {
                    Serial.write('?');  // parameter error
                    break;
                  }
*/                if( radio.sendtoWait(buf, 2, SLAVE_ADDRESS) )
                    Serial.write('!');  // ok
                  else
                    Serial.write('x');  // comm error
                  break;

// *** human mode commands ***

      // set IR lamp brightness
      case 'l'  : // convert parameter into number
//                int i;
                  i = 300;  // 3 sec timeout
                  while( ! Serial.readBytes(str,1) )
                  {
                    if( --i == 0 )
                    {
                      Serial.println('?');
                      return;
                    }
                  }
                  str[0] -= '0';
                  if( str[0] < 0 || str[0] > 5 )
                    break;
                  // set LED PWM (0..5 ==> 0..255)
                  analogWrite(irLedPin, str[0] * 51);
                  Serial.print("LED=");
                  Serial.println(str[0]*51);
                  human_mode = true;
                  break;
                  
      // read temperature
      case 't'  : Serial.println( rf22_driver.temperatureRead(RH_RF22_TSRANGE_0_128C, 0) / 2 );
                  human_mode = true;
                  break;
                  
      // get last RSSI
      case 'r'  : buf[0] = 'r';
                  if( ! radio.sendtoWait(buf, 1, SLAVE_ADDRESS) )
                    Serial.print("sendto failed");
                  else
                  {
                    int8_t t = rf22_driver.lastRssi();
                    Serial.print("my last rssi = ");
                    Serial.print((uint8_t)t);
                    Serial.print('(');
                    Serial.print(t);
                    Serial.print("); remote rssi =");
                  }
                  human_mode = true;
                  break;
                  
      // measure battery voltage
      case 'v'  : buf[0] = 'v';
                  if(!radio.sendtoWait(buf, 1, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  else
                    Serial.print("measured voltage [V]:");
                  human_mode = true;
                  break;
                  
      // Wakeup Roomba
      case 'W'  : buf[0] = 'W';
                  if(!radio.sendtoWait(buf, 1, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  human_mode = true;
                  break;
                  
      // START command -> PASSIVE mode
      case 'S'  : buf[0] = '$';
                  buf[1] = 128;
                  if(!radio.sendtoWait(buf, 2, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  human_mode = true;
                  break;

      // SAFE mode
      case 'A'  : velocity = 0;
                  angle = 0;
                  buf[0] = '$';
                  buf[1] = 131;
                  if(!radio.sendtoWait(buf, 2, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  human_mode = true;
                  break;

      // FULL mode
      case 'F'  : velocity = 0;
                  angle = 0;
                  buf[0] = '$';
                  buf[1] = 132;
                  if(!radio.sendtoWait(buf, 2, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  human_mode = true;
                  break;

      // CLEAN command
      case 'C'  : buf[0] = '$';
                  buf[1] = 135;
                  if(!radio.sendtoWait(buf, 2, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  human_mode = true;
                  break;

      // SPOT command
      case 'O'  : buf[0] = '$';
                  buf[1] = 134;
                  if(!radio.sendtoWait(buf, 2, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  human_mode = true;
                  break;

      // DOCK command
      case 'D'  : buf[0] = '$';
                  buf[1] = 143;
                  if(!radio.sendtoWait(buf, 2, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  human_mode = true;
                  break;

      // POWER DOWN command
      case 'P'  : buf[0] = '$';
                  buf[1] = 133;
                  if(!radio.sendtoWait(buf, 2, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  human_mode = true;
                  break;

      // Sensor - Battery voltage
      case 'b'  : buf[0] = '$';
                  buf[1] = 142;
                  buf[2] = 22;
                  if(!radio.sendtoWait(buf, 3, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  else
                    Serial.print("sensor battery voltage [mV]:");
                  human_mode = true;
                  break;
                  
      // Sensor - Chargind state
      case 'x'  : buf[0] = '$';
                  buf[1] = 142;
                  buf[2] = 21;
                  if(!radio.sendtoWait(buf, 3, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  else
                    Serial.print("sensor charging state:");
                  human_mode = true;
                  break;
                  
      // sensor battery charge
      case 'c'  : buf[0] = '$';
                  buf[1] = 142;
                  buf[2] = 25;
                  if(!radio.sendtoWait(buf, 3, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  else
                    Serial.print("sensor battery charge [mAh]:");
                  human_mode = true;
                  break;

      // Drive command -> increase forward speed  (Arrow up)
      case 12   : velocity += 20;
                  buf[0] = '$';
                  buf[1] = 137;
                  buf[2] = highByte(velocity);
                  buf[3] = lowByte(velocity);
                  buf[4] = highByte(angle);
                  buf[5] = lowByte(angle);
                  if(!radio.sendtoWait(buf, 6, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  human_mode = true;
                  break;
      
      // Drive command -> decrease forward speed  (Arrow down)
      case 13   : velocity -= 20;
                  buf[0] = '$';
                  buf[1] = 137;
                  buf[2] = highByte(velocity);
                  buf[3] = lowByte(velocity);
                  buf[4] = highByte(angle);
                  buf[5] = lowByte(angle);
                  if(!radio.sendtoWait(buf, 6, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  human_mode = true;
                  break;
      
      // Drive command -> Turn Left   (Arrow left)
      case 14  : if (angle > 0)
                    angle = 0;
                  else if (angle == 0)
                    angle = -2000;
                  else
                    angle /= 2;
                  buf[0] = '$';
                  buf[1] = 137;
                  buf[2] = highByte(velocity);
                  buf[3] = lowByte(velocity);
                  buf[4] = highByte(angle);
                  buf[5] = lowByte(angle);
                  if(!radio.sendtoWait(buf, 6, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  human_mode = true;
                  break;
      
      // Drive command -> Turn Right  (Arrow right)
      case 15   : if (angle < 0)
                    angle = 0;
                  else if (angle == 0)
                    angle = 2000;
                  else
                    angle /= 2;
                  buf[0] = '$';
                  buf[1] = 137;
                  buf[2] = highByte(velocity);
                  buf[3] = lowByte(velocity);
                  buf[4] = highByte(angle);
                  buf[5] = lowByte(angle);
                  if(!radio.sendtoWait(buf, 6, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  human_mode = true;
                  break;

      // Drive command -> ???   (Page Up)
      case 0    :
                  human_mode = true;
                  break;
      
      // Drive command -> Stop   (Page Dn)
      case 1    : velocity = 0;
                  angle = 0;
                  buf[0] = '$';
                  buf[1] = 137;
                  buf[2] = highByte(velocity);
                  buf[3] = lowByte(velocity);
                  buf[4] = highByte(angle);
                  buf[5] = lowByte(angle);
                  if(!radio.sendtoWait(buf, 6, SLAVE_ADDRESS))
                    Serial.print("sendto failed");
                  human_mode = true;
                  break;
      
      default   : Serial.print('?');
                  break;
    }
  }

  if( radio.available() )
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    // receive from Roomba with acknowledgement
    if( radio.recvfromAck(buf, &len, &from) && (from==SLAVE_ADDRESS) )
    {
      if( human_mode )
      {
        for(int i=0; i<len; i++)
        {
          Serial.print(' ');
          Serial.print(buf[i]);
        }
        Serial.println();
      }
      else
        Serial.write(buf, len);
    }
  }

}

  
