#include <OrangutanLEDs.h>
#include <OrangutanAnalog.h>
#include <OrangutanMotors.h>
#include <OrangutanLCD.h>

#include <avr/pgmspace.h>

OrangutanLCD lcd;
OrangutanMotors motors;
OrangutanAnalog analog;
OrangutanLEDs leds;

byte payload[3];
int index;

const char levels[] PROGMEM = {0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111};
 
// This function loads custom characters into the LCD.  Up to 8
// characters can be loaded; we use them for 6 levels of a bar graph
// plus a back arrow and a musical note character.
void load_custom_characters()
{
    lcd_load_custom_character(levels+0,0); // no offset, e.g. one bar
    lcd_load_custom_character(levels+1,1); // two bars
    lcd_load_custom_character(levels+2,2); // etc...
    lcd_load_custom_character(levels+4,3); // skip level 3
    lcd_load_custom_character(levels+5,4);
    lcd_load_custom_character(levels+6,5);
    clear(); // the LCD must be cleared for the characters to take effect
}

// 10 levels of bar graph characters
const byte bar_graph_characters[10] = {' ',0,0,1,2,3,3,4,5,0xff};

void setup() {
    load_custom_characters();
    Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
    payload[0] = 0;
    payload[1] = 0;
    payload[2] = 0;
    delay(1000);
    index = 0;
}

void set_motor_speeds(int leftMotor, int rightMotor)
{
    motors.setSpeeds(leftMotor, rightMotor);
}

void show_speed_lcd(int leftSpeed, int rightSpeed)
{
    lcd.gotoXY(0, 0);
    lcd.print("<- ");
    lcd.print(leftSpeed);        // print the resulting motor speed (-255 - 255)
    lcd.print("   ");

    lcd.gotoXY(0, 1);
    lcd.print("-> ");
    lcd.print(rightSpeed);
    lcd.print("   ");
}

void loop() {
    lcd.gotoXY(7, 0);
    char c = read_battery_millivolts_3pi() / 600;
    print_character(c);

    while (Serial.available() != 0) {
      char c = Serial.read();

      if (c == 0x13) {
        process_payload();
      }

      payload[0] = payload[1];
      payload[1] = payload[2];
      payload[2] = c;
    }
}
// 7e 00 09 81 cc cc 30 00 00 00 13 a3
/* Receive message
 *  Delimiter (0x7E)
 *  Length_16 (between length - checksum fields)
 *  Frame type (0x81)
 *  16bit source addr  
 *  RSSI_8 (signal strength)
 *  Options_8
 *  Rf data
 *  Checksum
 */

void process_payload()
{
  unsigned char command = payload[0] & 0x3F; // Command is 6 least significant bits.
  unsigned char motor1IsReversed = (payload[0] & 0x80) >> 7; // Motor 1 dir is most significant bit.
  unsigned char motor2IsReversed = (payload[0] & 0x40) >> 6; // Motor 2 dir is 2nd most significant bit.

  int motor1Speed = payload[1] * (motor1IsReversed == 1 ? -1 : 1);
  int motor2Speed = payload[2] * (motor2IsReversed == 1 ? -1 : 1);

  show_speed_lcd(motor1Speed, motor2Speed);
  set_motor_speeds(motor1Speed, motor2Speed);
}
