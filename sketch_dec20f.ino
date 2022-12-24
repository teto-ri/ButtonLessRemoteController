#include <Arduino.h>
#include <EEPROM.h>

#if RAMEND <= 0x4FF || (defined(RAMSIZE) && RAMSIZE < 0x4FF)
#define RAW_BUFFER_LENGTH 120
#elif RAMEND <= 0xAFF || (defined(RAMSIZE) && RAMSIZE < 0xAFF) // 0xAFF for LEONARDO
#define RAW_BUFFER_LENGTH 500                                  // 600 is too much here, because we have additional uint8_t rawCode[RAW_BUFFER_LENGTH];
#else
#define RAW_BUFFER_LENGTH 750
#endif

#define NO_LED_FEEDBACK_CODE        // saves 92 bytes program memory
#define EXCLUDE_UNIVERSAL_PROTOCOLS // Saves up to 1000 bytes program memory.
#define EXCLUDE_EXOTIC_PROTOCOLS    // saves around 650 bytes program memory if all other protocols are active

#define RECORD_GAP_MICROS 12000    // Activate it for some LG air conditioner protocols
#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.
#include <IRremote.hpp>

struct storedIRDataStruct
{
  IRData receivedIRData[26];
  // extensions for sendRaw
} sStoredIRData;

bool IRDataExist[26];

/* Define the digital pins used for the clock and data */
#define SCL_PIN 8
#define SDO_PIN 9

enum RGB_Color
{
  RED,
  GREEN,
  BLUE,
  CYAN,
  MAGENTA,
  YELLOW,
  WHITE
};

int red_light_pin = A0;
int green_light_pin = A1;
int blue_light_pin = A2;

/* Used to store the key state */
byte Key;
byte pre = 0;
void setup()
{
  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);
  rgb_LED(WHITE); // setup start

  /* Initialise the serial interface */
  Serial.begin(9600);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/ || defined(USBCON) /*STM32_stm32*/ || defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
  delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif

  IrReceiver.begin(2, ENABLE_LED_FEEDBACK); // 리모컨 수신부 시작
  IrSender.begin(3, ENABLE_LED_FEEDBACK);   // Specify send pin and enable feedback LED at default feedback LED pin

  /* Configure the clock and data pins */
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDO_PIN, INPUT);

  for (int i = 0; i < 26; i++)
  {
    IRDataExist[i] = false;
  }

  IRDataRetrieveFromEEPROM(); // retrieve IRData from EEPROM

  rgb_LED(GREEN); // setup complete
}

/* Main program */
void loop()
{
  /* Read the current state of the keypad */
  Key = Read_Keypad();
  keypad_loop(Key);
  
  /* Wait a little before reading again
     so not to flood the serial port*/

  delay(10);
}

struct EEPROM_IRData // total 16bytes
{
  uint8_t exist;           // if data exists: 42
  decode_type_t protocol;  // UNKNOWN, NEC, SONY, RC5, PULSE_DISTANCE, ...
  uint16_t address;        // Decoded address
  uint16_t command;        // Decoded command
  uint16_t extra;          // Used for Kaseikyo unknown vendor ID. Ticks used for decoding Distance protocol.
  uint16_t numberOfBits;   // Number of bits received for data (address + command + parity) - to determine protocol length if different length are possible.
  uint8_t flags;           // See IRDATA_FLAGS_* definitions
  uint32_t decodedRawData; // Up to 32 bit decoded raw data, used for sendRaw functions.
};

void IRDataRetrieveFromEEPROM()
{
  for (int i = 0; i < 26; i++)
  {
    EEPROM_read_IRData(i);
  }
}

void IRDataClearFromEEPROM() {
  for (int i = 0; i < 26; i++)
  {
    EEPROM_delete_IRData(i);
  }
}

void IRDataClearFromSRAM() {
  for (int i = 0; i < 26; i++)
  {
    IRDataExist[i] = false;
  }
}

void EEPROM_write_IRData(IRData *aIRReceivedData, int idx)
{ // SRAM -> EEPROM
  EEPROM_IRData buffer;
  buffer.exist = 42;
  buffer.protocol = aIRReceivedData->protocol;
  buffer.address = aIRReceivedData->address;
  buffer.command = aIRReceivedData->command;
  buffer.extra = aIRReceivedData->extra;
  buffer.numberOfBits = aIRReceivedData->numberOfBits;
  buffer.flags = aIRReceivedData->flags;
  buffer.decodedRawData = aIRReceivedData->decodedRawData;
  EEPROM.put(idx * 16, buffer);
}

void EEPROM_read_IRData(int idx)
{ // EEPROM -> SRAM
  if (EEPROM.read(idx * 16) != 42)
    return; // data not exist
  EEPROM_IRData buffer;
  EEPROM.get(idx * 16, buffer);
  IRDataExist[idx] = true;
  sStoredIRData.receivedIRData[idx].protocol = buffer.protocol;
  sStoredIRData.receivedIRData[idx].address = buffer.address;
  sStoredIRData.receivedIRData[idx].command = buffer.command;
  sStoredIRData.receivedIRData[idx].extra = buffer.extra;
  sStoredIRData.receivedIRData[idx].numberOfBits = buffer.numberOfBits;
  sStoredIRData.receivedIRData[idx].flags = buffer.flags;
  sStoredIRData.receivedIRData[idx].decodedRawData = buffer.decodedRawData;
}

void EEPROM_delete_IRData(int idx)
{
  EEPROM.update(idx * 16, 255);
}

bool storeCode(IRData *aIRReceivedData, int idx)
{
  if (aIRReceivedData->flags & IRDATA_FLAGS_IS_REPEAT)
  {
    return false;
  }
  if (aIRReceivedData->flags & IRDATA_FLAGS_IS_AUTO_REPEAT)
  {
    return false;
  }
  if (aIRReceivedData->flags & IRDATA_FLAGS_PARITY_FAILED)
  {
    return false;
  }
  if (aIRReceivedData->protocol == UNKNOWN)
  {
    Serial.println(F("UNKNOWN protocol, skipped."));
    return false;
  }

  sStoredIRData.receivedIRData[idx] = *aIRReceivedData; // SRAM store

  IrReceiver.printIRResultShort(&Serial);
  sStoredIRData.receivedIRData[idx].flags = 0; // clear flags -esp. repeat- for later sending

  EEPROM_write_IRData(aIRReceivedData, idx); // EEPROM store

  IRDataExist[idx] = true;
  Serial.println();
  return true;
}

void sendCode(storedIRDataStruct *aIRDataToSend, int idx)
{
  if (!IRDataExist[idx])
  {
    Serial.println(F("Not Sent: Not found"));
    return;
  }

  auto tProtocol = aIRDataToSend->receivedIRData[idx].protocol;
  auto tAddress = aIRDataToSend->receivedIRData[idx].address;
  auto tCommand = aIRDataToSend->receivedIRData[idx].command;

  if (tProtocol == LG2)
  {
    IrSender.sendLG2(tAddress, tCommand, 3);
  }
  else
  {
    IrSender.write(&aIRDataToSend->receivedIRData[idx], 3);
  }
  Serial.print(F("Sent: "));
  printIRResultShort(&Serial, &aIRDataToSend->receivedIRData[idx], false);
}

/* Read the state of the keypad */
int Read_Keypad(void)
{
  int Count;
  int Key_State = 0;

  /* Pulse the clock pin 16 times (one for each key of the keypad)
     and read the state of the data pin on each pulse */
  for (Count = 1; Count <= 16; Count++)
  {
    digitalWrite(SCL_PIN, LOW);

    /* If the data pin is low (active low mode) then store the
       current key number */
    if (!digitalRead(SDO_PIN))
      Key_State = Count;

    digitalWrite(SCL_PIN, HIGH);
  }

  return Key_State;
}

int x_predef_arr[] = {
    -3,
    -1,
    1,
    3,
    -3,
    -1,
    1,
    3,
    -3,
    -1,
    1,
    3,
    -3,
    -1,
    1,
    3,
};

int y_predef_arr[] = {
    3,
    3,
    3,
    3,
    1,
    1,
    1,
    1,
    -1,
    -1,
    -1,
    -1,
    -3,
    -3,
    -3,
    -3,
};

int get_x(int num)
{
  // return (num - 1) % 4;
  return x_predef_arr[num - 1];
}

int get_y(int num)
{
  // return (num - 1) / 4;
  return y_predef_arr[num - 1];
}

float get_distance(float x_1, float x_2, float y_1, float y_2)
{
  return sqrt(pow(x_2 - x_1, 2) + pow(y_2 - y_1, 2));
}

float get_slope(int start_num, int end_num)
{
  return get_raw_slope(get_x(start_num), get_y(start_num), get_x(end_num), get_y(end_num));
}

float get_raw_slope(int x_1, int y_1, int x_2, int y_2)
{
  int delta_x = x_2 - x_1;
  int delta_y = y_2 - y_1;
  float slope = atan2(delta_y, delta_x) * 180 / PI;
  return normalize_slope(slope);
}

float normalize_slope(float slope)
{
  while (true)
  {
    if (slope < 0)
      slope += 360;
    if (slope >= 360)
      slope -= 360;
    if (slope >= 0 && slope < 360)
      return slope;
  }
}

int closest_slope(int num)
{ // get closest slope(deg)
  bool init = false;
  int best;
  int best_delta;
  for (int i = 0; i < 8; i++)
  {
    int delta = abs(num - i * 45);
    if (!init || delta < best_delta)
    {
      init = true;
      best = i * 45;
      best_delta = delta;
    }
  }
  if (best_delta > abs(num - 360))
  {
    return 0;
  }
  return best;
}

float average(int arr[], int length)
{
  int value = 0;
  for (int i = 0; i < length; i++)
  {
    value += arr[i];
  }
  return value * 1.0 / length;
}

int get_circular_direction(int arr[], int length)
{ // clockwise: 1, counter-clockwise: 2
  int arr_x[length];
  int arr_y[length];
  int arr_slope[length];
  int direction_norm = 0;

  for (int i = 0; i < length; i++)
  { // split x, y to array
    arr_x[i] = get_x(arr[i]);
    arr_y[i] = get_y(arr[i]);
  }

  int avg_x = average(arr_x, length - 1);
  int avg_y = average(arr_y, length - 1);

  for (int i = 0; i < length; i++)
  { // get slope array
    arr_slope[i] = get_raw_slope(avg_x, avg_y, arr_x[i], arr_y[i]);
  }

  for (int i = 0; i < length; i++)
  { // calculate direction
    int j;
    if (i == 0)
    {
      j = length;
    }
    else
    {
      j = i - 1;
    }

    if (arr_slope[i] > arr_slope[j])
    { // if slope increases
      direction_norm++;
    }

    if (arr_slope[i] < arr_slope[j])
    { // if slope decreases
      direction_norm--;
    }
  }

  if (direction_norm < 0)
  { // clockwise
    return 1;
  }

  if (direction_norm > 0)
  { // counter-clockwise
    return 2;
  }
}

void arr_zerofill(int arr[], int length)
{
  for (int i = 0; i < length; i++)
  {
    arr[i] = 0;
  }
}

// operation mode; normal: 0, gesture_setup: 1;

int operation_mode;

void IR_send(int idx)
{
  sendCode(&sStoredIRData, idx);
}

void IR_save(int idx)
{
  bool success = false;
  while (true)
  {
    if (IrReceiver.decode())
    {
      success = storeCode(IrReceiver.read(), idx);
      IrReceiver.resume();
      if (success)
        break;
    }
  }
}

// keypad

const int keypad_length = 16;
int keypad_data[keypad_length];
int keypad_data_idx = 0;
unsigned long keypad_last_button_millis = millis();

unsigned long keypad_long_press_start_millis = millis();
bool keypad_long_press_mode = false;
int keypad_long_press_delay = 1500;
int keypad_long_long_press_delay = 3000;
bool keypad_gesture_setup_waiting = false;

bool keypad_reset_queue = false;
int keypad_reset_delay = 250;

void keypad_loop(int keypad_num)
{
  bool process_keypad = false;
  if (keypad_num == 0)
  { // if keypad released
    process_keypad = keypad_check_reset();
  }
  else
  { // if keypad pressed
    process_keypad = keypad_store_data(keypad_num);
  }

  if (process_keypad)
  {
    keypad_process();
  }

  keypad_execute_reset(); // if reset queued, execute reset
}

bool keypad_store_data(int keypad_num)
{
  if (keypad_gesture_setup_waiting)
    return false; // waiting for release gesture_setup command press

  keypad_last_button_millis = millis();

  if (operation_mode == 0)
  {
    rgb_LED(YELLOW);
  }
  else if (operation_mode == 1)
  {
    rgb_LED(BLUE);
  }

  if (keypad_data_idx != 0)
  { // if not first press
    if (keypad_data[keypad_data_idx - 1] == keypad_num)
    { // if same keypad number

      if (!keypad_long_press_mode)
      {
        keypad_long_press_mode = true;
        keypad_long_press_start_millis = millis();
        return false; // wait for long press
      }

      unsigned long millis_delta = millis() - keypad_long_press_start_millis;

      if (millis_delta > keypad_long_long_press_delay && operation_mode == 0 && keypad_data_idx == 1)
      { // special gesture
        if (keypad_special_preset_setup())
          return false;
      }

      if (millis_delta > keypad_long_press_delay && operation_mode == 0)
      {              // long press mode
        return true; // process same data
      }

      return false;
    }
    else
    {
      keypad_long_press_mode = false;
    }
  }
  keypad_data[keypad_data_idx] = keypad_num;
  keypad_data_idx++;
  return false; // wait for further input
}
/*
const int gesture_preset_length_1 = 3;
const int gesture_preset_length_2 = 16;
int gesture_preset[gesture_preset_length_1][gesture_preset_length_2] = {
    {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}, // tap
    {0, 45, 90, 135, 180, 225, 270, 315},                    // linear(deg)
    {1, 2}                                                   // circular(clockwise, counter-clockwise)
}
*/
/*
  gesture_preset_idx;
  tap: 0-15;
  linear(deg): 16-23;
  circular(clockwise, counter-clockwise): 24-25;
*/

int gesture_tap_idx(int keypad_gesture_value)
{
  return keypad_gesture_value - 1;
}

int gesture_linear_idx(int keypad_gesture_value)
{
  return 16 + keypad_gesture_value / 45;
}

int gesture_circular_idx(int keypad_gesture_value)
{
  return 23 + keypad_gesture_value;
}

bool keypad_special_preset_setup()
{
  int keypad_special_num = keypad_data[0];
  if (keypad_data[0] == 1)
  {
    operation_mode = 1;
    keypad_reset_queue = true;
    keypad_gesture_setup_waiting = true;
  }
  if (keypad_data[0] == 4)
  {
    rgb_LED(WHITE);
    IRDataClearFromEEPROM();
    IRDataClearFromSRAM();
    Serial.println(F("Data reset complete"));
    delay(100);
    keypad_reset_queue = true;
  }
  return false;
}

int keypad_gesture_pattern()
{
  if (keypad_data_idx == 1)
    return 0; // tap

  int keypad_start_num = keypad_data[0];
  int keypad_end_num = keypad_data[keypad_data_idx - 1];

  int arr_x[keypad_data_idx];
  int arr_y[keypad_data_idx];

  Serial.print(F("Key pressed: "));
  for (int i = 0; i < keypad_data_idx; i++)
  { // split x, y to array
    arr_x[i] = get_x(keypad_data[i]);
    arr_y[i] = get_y(keypad_data[i]);
    Serial.print(keypad_data[i]);
    Serial.print(F(" "));
  }
  Serial.println();

  float avg_x = average(arr_x, keypad_data_idx);
  float avg_y = average(arr_y, keypad_data_idx);

  float keypad_start_to_end = get_distance(get_x(keypad_start_num), get_x(keypad_end_num), get_y(keypad_start_num), get_y(keypad_end_num));
  float keypad_start_to_avg = get_distance(get_x(keypad_start_num), avg_x, get_y(keypad_start_num), avg_y);
  float keypad_avg_to_end = get_distance(avg_x, get_x(keypad_end_num), avg_y, get_y(keypad_end_num));

  if (keypad_start_to_end * 2 > (keypad_start_to_avg + keypad_avg_to_end) * 1.9)
    return 1;
  return 2;
}

void keypad_process()
{
  int keypad_gesture_type = keypad_gesture_pattern();
  int keypad_gesture_value;
  int keypad_gesture_idx;
  int keypad_start_num = keypad_data[0];
  int keypad_end_num = keypad_data[keypad_data_idx - 1];

  bool keypad_process_success = false;

  if (keypad_gesture_type == 0)
  { // tap
    keypad_gesture_value = keypad_start_num;
    keypad_gesture_idx = gesture_tap_idx(keypad_gesture_value);
  }

  else if (keypad_gesture_type == 1)
  { // linear
    keypad_gesture_value = closest_slope(get_slope(keypad_start_num, keypad_end_num));
    keypad_gesture_idx = gesture_linear_idx(keypad_gesture_value);
  }

  else if (keypad_gesture_type == 2)
  { // circular
    keypad_gesture_value = get_circular_direction(keypad_data, keypad_data_idx);
    keypad_gesture_idx = gesture_circular_idx(keypad_gesture_value);
  }

  if (operation_mode == 0)
  { // send IR_data correspond to gesture; normal mode
    rgb_LED(RED);
    IR_send(keypad_gesture_idx);
  }
  if (operation_mode == 1)
  { // save IR_data correspond to gesture; gesture_setup mode
    rgb_LED(MAGENTA);
    IR_save(keypad_gesture_idx);
    operation_mode = 0;
  }

  Serial.print(F("Gesture type: "));
  Serial.print(keypad_gesture_type);
  Serial.print(F(" "));
  Serial.print(F("Gesture idx: "));
  Serial.print(keypad_gesture_idx);
  Serial.print(F(" "));
  Serial.print(F("Gesture value: "));
  Serial.println(keypad_gesture_value);

  keypad_process_success = true;

  if (!keypad_process_success)
  {
    rgb_LED(WHITE); // no gesture found
  }
}

bool keypad_check_reset()
{
  if (operation_mode == 1 && keypad_gesture_setup_waiting)
    keypad_gesture_setup_waiting = false;
  if (millis() - keypad_last_button_millis > keypad_reset_delay && keypad_data_idx != 0)
  {
    keypad_reset_queue = true; // reset keypad data array, after process keypad data
    return true;               // process data -> reset(init) data
  }
  return false;
}

void keypad_execute_reset()
{
  if (keypad_reset_queue)
  {
    keypad_reset_queue = false;
    arr_zerofill(keypad_data, keypad_length);
    keypad_data_idx = 0;
    keypad_long_press_mode = false;
    keypad_long_press_start_millis = 0;
    if (operation_mode == 0)
    {
      rgb_LED(GREEN);
    }
    if (operation_mode == 1)
    {
      rgb_LED(CYAN);
    }
    return;
  }
}

void rgb_LED(RGB_Color color)
{
  switch (color)
  {
  case RED:
    RGB_color(255, 0, 0);
    break; // keypad_process; operation_mode: 0; keypad released when normal mode
  case GREEN:
    RGB_color(0, 255, 0);
    break; // after reset; operation_mode: 0; normal idle state
  case BLUE:
    RGB_color(0, 0, 255);
    break; // keypad_store_data; operation_mode: 1; keypad pressed when gesture_setup mode
  case CYAN:
    RGB_color(0, 255, 255);
    break; // after reset; operation_mode: 1; gesture_setup idle state
  case MAGENTA:
    RGB_color(255, 0, 255);
    break; // keypad_process; operation_mode: 1; keypad released when gesture_setup mode
  case YELLOW:
    RGB_color(255, 255, 0);
    break; // keypad_store_data; operation_mode: 0; keypad pressed when normal mode
  case WHITE:
    RGB_color(255, 255, 255);
    break; // general error; no gesture found
  default:
    break;
  }
}

void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
{
  analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
}
