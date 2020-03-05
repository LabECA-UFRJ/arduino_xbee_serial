#include <OrangutanLEDs.h>
#include <OrangutanAnalog.h>
#include <OrangutanMotors.h>
#include <OrangutanLCD.h>

#include <XBee.h>

#include <avr/pgmspace.h>

const int c_BaudRate = 9600;

OrangutanLCD lcd;
OrangutanMotors motors;
OrangutanAnalog analog;
OrangutanLEDs leds;

XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();

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

void setup() 
{
    load_custom_characters();
    Serial.begin(c_BaudRate);
    xbee.setSerial(Serial);
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

void show_battery_lcd()
{
    lcd.gotoXY(7, 0);
    char c = read_battery_millivolts_3pi() / 600;
    print_character(c);
}

void loop() 
{
    show_battery_lcd();

    bool dataAvailable = receive_xbee_data();
    if (dataAvailable)
        process_payload(rx16.getData(), rx16.getDataLength());
}

bool receive_xbee_data()
{
    xbee.readPacket();

    if (xbee.getResponse().isAvailable() == false)
        return false;

    if (xbee.getResponse().getApiId() != RX_16_RESPONSE)
        return false;

    xbee.getResponse().getRx16Response(rx16);
    return true;
}

// TODO: Check for other payload sizes.
// e.g. command 0x01 will be a send me your battery level, so it only has 1 byte payload.
void process_payload(uint8_t* data, uint8_t size)
{
    unsigned char command = data[0] & 0x3F;                    // Command is 6 least significant bits.
    unsigned char motor1IsReversed = (data[0] & 0x80) >> 7;    // Motor 1 dir is most significant bit.
    unsigned char motor2IsReversed = (data[0] & 0x40) >> 6;    // Motor 2 dir is 2nd most significant bit.

    int motor1Speed = data[1] * (motor1IsReversed == 1 ? -1 : 1);
    int motor2Speed = data[2] * (motor2IsReversed == 1 ? -1 : 1);

    show_speed_lcd(motor1Speed, motor2Speed);
    set_motor_speeds(motor1Speed, motor2Speed);
}
