#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Bounce2.h>
#define ENC_DECODER             ENC_NORMAL
#include <ClickEncoder.h>
#include <TimerOne.h>


#define LED_BLUE                7

#define BUTTON_OK               8
#define BUTTON_ADJ              9

#define CIN_SENSE               A1
#define COUT_SENSE              A0
#define VIN_SENSE               A6
#define VOUT_SENSE              A2

#define TFT_CS                  10  // Chip select line for TFT display
#define TFT_DC                  4   // Data/command line for TFT
#define TFT_RST                 3   // Reset line for TFT (or connect to +5V)

#define ENC_A                   5
#define ENC_B                   6

#define POWER_ENABLE            A3

#define DIGITAL_POT_ADDR        0x28
#define MAX_DIGITAL_POT_VALUE   0x0100
#define MIN_DIGITAL_POT_VALUE   0x0000

#define DIGITAL_POT_VREG_INC    0x04
#define DIGITAL_POT_VREG_DEC    0x08
#define DIGITAL_POT_VREG_READ   0x0C

#define DIGITAL_POT_CIN_INC     0x14
#define DIGITAL_POT_CIN_DEC     0x18
#define DIGITAL_POT_CIN_WRITE   0x10
#define DIGITAL_POT_CIN_READ    0x1C


#define MAX_VOLTAGE_READ        4.965
#define MIN_VOLTAGE_READ        0.457


#define VIN_IN_MAX              1023.0
#define VIN_IN_MIN              0.0

#define VIN_OUT_MAX             5.0
#define VIN_OUT_MIN             0.0

#define VOUT_IN_MAX             1023.0
#define VOUT_IN_MIN             0.0

#define VOUT_OUT_MAX            36.30
#define VOUT_OUT_MIN            0.0


#define CIN_IN_MAX              1023.0
#define CIN_IN_MIN              0.0

#define CIN_OUT_MAX             55.000
#define CIN_OUT_MIN             0.0

#define COUT_IN_MAX             1023.0
#define COUT_IN_MIN             0.0

#define COUT_OUT_MAX            91.666
#define COUT_OUT_MIN            0.0

#define VOLTAGE_PRECISION       2
#define CURRENT_PRECISION       4

#define DATA_X                  100
#define LABEL_X                 0

#define TEXT_HEIGHT             10
#define TEXT_WIDTH              50


#define SCREEN_LABEL_Y          10
#define ENC_Y                   20

#define VIN_Y                   30
#define CIN_Y                   40

#define VOUT_Y                  60
#define COUT_Y                  70


//#define DEBOUNCE_INTERVAL 200
#define DEBOUNCE_INTERVAL 100
Bounce button_ok     = Bounce();
Bounce button_adj    = Bounce();
ClickEncoder encoder = ClickEncoder(ENC_A, ENC_B);
Adafruit_ST7735 tft  = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

float cin_value = 0;
float prev_cin_value = -100;

float cout_value = 0;
float prev_cout_value = -100;

float vin_value = 0;
float prev_vin_value = -100;

float vout_value = 0;
float prev_vout_value = -100;

bool prev_enable = true;
bool pwr_enable = false;

enum STATE_T {
  READY = 0,
  IDLE,
  PREPARE_EDIT,
  EDIT
};

enum STATE_T state = READY;

int prev_enc_value = -1;
int enc_value = 0;

uint16_t prev_pot_vreg_val = 0xFFFF;
uint16_t pot_vreg_val = 0x0000;

uint16_t prev_pot_cin_val = 0xFFFF;
uint16_t pot_cin_val = 0x0000;

void timer_isr(){
  encoder.service();
}

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(57600);
  Wire.begin();
  encoder.setAccelerationEnabled(true);

  //Push Buttons
  pinMode     (BUTTON_OK,     INPUT);
  pinMode     (BUTTON_ADJ,    INPUT);

  button_ok.attach(BUTTON_OK);
  button_adj.attach(BUTTON_ADJ);

  button_ok.interval(DEBOUNCE_INTERVAL);
  button_adj.interval(DEBOUNCE_INTERVAL);

  //Power Signal
  pinMode     (POWER_ENABLE,  OUTPUT);
  digitalWrite(POWER_ENABLE,  LOW);

  //LEDs
  pinMode     (LED_BLUE,      OUTPUT);
  digitalWrite(LED_BLUE,      HIGH);


  //Setup Timer
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timer_isr);

  //Setup the LCD
  tft.initR(INITR_BLACKTAB);  //Initialize the Chip with a black tab
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(3);

  tft.setTextColor(ST7735_BLUE);
  tft.setCursor(40, 50);
  tft.print("Cospan Design");
  tft.setCursor(20, 60);
  tft.print("USB Lab Power Supply");
  delay(200);
  tft.fillScreen(ST7735_BLACK);

}

void loop() {
  //Update Debounce Signals
  button_ok.update();
  button_adj.update();
  enc_value -= encoder.getValue();

  state_machine();

  delay(10);
  cin_value = read_cin();
  cout_value = read_cout();
  vin_value = read_vin();
  vout_value = read_vout();
}

float adj_prec (float value, float precision){
    return (floor((value * pow(10, precision) + 0.5)) / pow(10, precision));
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max, float precision){
  return adj_prec((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, precision);
}

float read_cin(void){
  return map_float(analogRead(CIN_SENSE), CIN_IN_MIN, CIN_IN_MAX, CIN_OUT_MIN, CIN_OUT_MAX, CURRENT_PRECISION);
}

float read_cout(void){
  return map_float(analogRead(COUT_SENSE), COUT_IN_MIN, COUT_IN_MAX, COUT_OUT_MIN, COUT_OUT_MAX, CURRENT_PRECISION);
}

float read_vin(void){
  return map_float(analogRead(VIN_SENSE), VIN_IN_MIN, VIN_IN_MAX, VIN_OUT_MIN, VIN_OUT_MAX, VOLTAGE_PRECISION);
}

float read_vout(void){
  return map_float(analogRead(VOUT_SENSE), VOUT_IN_MIN, VOUT_IN_MAX, VOUT_OUT_MIN, VOUT_OUT_MAX, VOLTAGE_PRECISION);
}

uint16_t read_vreg_value(void){
  uint16_t value;
  Wire.beginTransmission(DIGITAL_POT_ADDR);
  Wire.write(DIGITAL_POT_VREG_READ);
  Wire.endTransmission();

  Wire.requestFrom(DIGITAL_POT_ADDR, 2);
  value = 0;
  while (Wire.available()){
    value = (value << 8 | Wire.read());
  }

  Serial.print("VREG: ");
  Serial.println(value, HEX);
  return value;
}

uint16_t read_cin_value(void){
  uint16_t value;
  Wire.beginTransmission(DIGITAL_POT_ADDR);
  Wire.write(DIGITAL_POT_CIN_READ);
  Wire.endTransmission();

  Wire.requestFrom(DIGITAL_POT_ADDR, 2);
  value = 0;
  while (Wire.available()){
    value = (value << 8 | Wire.read());
  }

  Serial.print("CIN: ");
  Serial.println(value, HEX);
  return value;
}

void set_cin_value(uint16_t value){
  uint8_t byte_value;
  Wire.beginTransmission(DIGITAL_POT_ADDR);
  byte_value = DIGITAL_POT_CIN_WRITE;
  byte_value |= (0x03 | (value >> 8));
  Wire.write(byte_value);
  Wire.write((uint8_t)(0xFF | value));
  Wire.endTransmission();
}

void increment_voltage(void){
  uint16_t value;
  value = read_vreg_value();
  if (value < MAX_DIGITAL_POT_VALUE){
    Serial.println("Incrementing");
    Wire.beginTransmission(DIGITAL_POT_ADDR);
    Wire.write(DIGITAL_POT_VREG_INC);
    Wire.endTransmission();
  }
  else {
    display_max_voltage_error();
  }
}

void decrement_voltage(void){
  uint8_t value;
  value = read_vreg_value();
  if (value > MIN_DIGITAL_POT_VALUE){
    Serial.println("Decrementing");
    Wire.beginTransmission(DIGITAL_POT_ADDR);
    Wire.write(DIGITAL_POT_VREG_DEC);
    Wire.endTransmission();
  }
  else {
    display_min_voltage_error();
  }
}

void enable_power(bool enable){
  if (enable){
    Serial.println("Enable");
    pinMode(POWER_ENABLE, INPUT);
    pwr_enable = true;
  }
  else{
    Serial.println("Disable");
    pinMode(POWER_ENABLE, OUTPUT);
    pwr_enable = false;
  }
}

bool is_enabled(){
  return pwr_enable;
}

void state_machine(void){
  switch(state){
    case READY:
      prev_vin_value = -1;
      prev_cin_value = -1;
      prev_enable = !is_enabled();

      tft.fillScreen(ST7735_BLACK);
      tft.setTextColor(ST7735_GREEN);

      tft.setCursor(LABEL_X, SCREEN_LABEL_Y);
      tft.print("IDLE");

      tft.setCursor(LABEL_X, VIN_Y);
      tft.print("Input Voltage:");

      tft.setCursor(LABEL_X, CIN_Y);
      tft.print("Input Current:");

      tft.setCursor(LABEL_X, VOUT_Y);
      tft.print("Output Voltage:");

      tft.setCursor(LABEL_X, COUT_Y);
      tft.print("Output Current:");

      state = IDLE;
      break;
    case IDLE:
      if (button_adj.fell()){
        state = PREPARE_EDIT;
        enable_power(false);
      }
      if (button_ok.fell()){
        enable_power(!is_enabled());
      }
      update_idle_screen();
      break;

    case PREPARE_EDIT:
      prev_vin_value = -1;
      prev_cin_value = -1;
      prev_enable = !is_enabled();

      tft.fillScreen(ST7735_WHITE);
      tft.setTextColor(ST7735_BLUE);

      tft.setCursor(LABEL_X, SCREEN_LABEL_Y);
      tft.print("EDIT");

      tft.setCursor(LABEL_X, VIN_Y);
      tft.print("Input Voltage:");

      tft.setCursor(LABEL_X, CIN_Y);
      tft.print("Input Current:");

      tft.setCursor(LABEL_X, VOUT_Y);
      tft.print("Output Voltage:");

      tft.setCursor(LABEL_X, COUT_Y);
      tft.print("Output Current:");


      update_edit_screen();
      state = EDIT;
      break;

    case EDIT:
      if (button_ok.fell()){
        state = READY;
      }
      update_edit_screen();
      break;
    default:
      state = READY;
      break;
  }
}

void print_float_data(uint8_t y_pos, float *prev_value, float value, uint16_t back_color, uint16_t fore_color, uint8_t precision){
  if (*prev_value != value){
    tft.fillRect(DATA_X, y_pos, TEXT_WIDTH, TEXT_HEIGHT, back_color);
    tft.setTextColor(fore_color);
    tft.setCursor(DATA_X, y_pos);
    tft.print(value, precision);
    *prev_value = value;
  }
}

void print_int_data(uint8_t y_pos, int *prev_value, int value, uint16_t back_color, uint16_t fore_color){
  if (*prev_value != value){
    tft.fillRect(DATA_X, y_pos, TEXT_WIDTH, TEXT_HEIGHT, back_color);
    tft.setTextColor(fore_color);
    tft.setCursor(DATA_X, y_pos);
    tft.print(value);
    *prev_value = value;
  }
}

void update_idle_screen(void){
  tft.setTextColor(ST7735_BLUE);
  if (enc_value > prev_enc_value){
    increment_voltage();
  }
  if (enc_value < prev_enc_value){
    decrement_voltage();
  }

  print_int_data(ENC_Y,   &prev_enc_value, enc_value, ST7735_BLACK, ST7735_BLUE);

  //Input Voltage/Current
  print_float_data(VIN_Y,   &prev_vin_value, vin_value, ST7735_BLACK, ST7735_BLUE, VOLTAGE_PRECISION);
  print_float_data(CIN_Y,   &prev_cin_value, cin_value, ST7735_BLACK, ST7735_BLUE, CURRENT_PRECISION);

  //Output Voltage/Current
  print_float_data(VOUT_Y,  &prev_vout_value, vout_value, ST7735_BLACK, ST7735_BLUE, VOLTAGE_PRECISION);
  print_float_data(COUT_Y,  &prev_cout_value, cout_value, ST7735_BLACK, ST7735_BLUE, CURRENT_PRECISION);

  if (prev_enable != is_enabled()) {
    Serial.println("Change Power Enable");
    tft.fillRect(100, 90, 50, 10, ST7735_BLACK);
    tft.setCursor(100, 90);
    if (is_enabled()){
      tft.setTextColor(ST7735_RED);
      tft.print("Enable");
    }
    else {
      tft.setTextColor(ST7735_MAGENTA);
      tft.print("Disable");
    }
    prev_enable = is_enabled();
  }
}

void update_edit_screen(void){
  tft.setTextColor(ST7735_BLUE);

  print_int_data(ENC_Y,   &prev_enc_value, enc_value, ST7735_WHITE, ST7735_BLUE);

  //Input Voltage/Current
  print_float_data(VIN_Y,   &prev_vin_value, vin_value, ST7735_WHITE, ST7735_BLUE, VOLTAGE_PRECISION);
  print_float_data(CIN_Y,   &prev_cin_value, cin_value, ST7735_WHITE, ST7735_BLUE, CURRENT_PRECISION);

  //Output Voltage/Current
  print_float_data(VOUT_Y,  &prev_vout_value, vout_value, ST7735_WHITE, ST7735_BLUE, VOLTAGE_PRECISION);
  print_float_data(COUT_Y,  &prev_cout_value, cout_value, ST7735_WHITE, ST7735_BLUE, CURRENT_PRECISION);

  if (prev_enable != is_enabled()) {
    Serial.println("Change Power Enable");
    tft.fillRect(100, 90, 50, 10, ST7735_WHITE);
    tft.setCursor(100, 90);
    if (digitalRead(POWER_ENABLE)){
      tft.setTextColor(ST7735_RED);
      tft.print("Enable");
    }
    else {
      tft.setTextColor(ST7735_MAGENTA);
      tft.print("Disable");
    }
    prev_enable = is_enabled();
  }
}

void display_max_voltage_error(){
  switch (state){
    case IDLE:
      tft.fillRect(100, 10, 50, 10, ST7735_BLACK);
      break;
    case EDIT:
      tft.fillRect(100, 10, 50, 10, ST7735_WHITE);
      break;
    default:
      break;
  }
  tft.setTextColor(ST7735_RED);
  tft.setCursor(100, 10);
  tft.print("MAX");
}

void display_min_voltage_error(){
  switch (state){
    case IDLE:
      tft.fillRect(100, 10, 50, 10, ST7735_BLACK);
      break;
    case EDIT:
      tft.fillRect(100, 10, 50, 10, ST7735_WHITE);
      break;
    default:
      break;
  }
  tft.setTextColor(ST7735_RED);
  tft.setCursor(100, 10);
  tft.print("MIN");
}

