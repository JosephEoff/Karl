#include <Servo.h>
#include <SPI.h> 

String message_buffer;
int servo_x_port=5;
int servo_y_port=6;
int min_X=30;
int max_X=150;
int min_Y=30;
int max_Y=150;
int range_X=max_X-min_X;
int range_Y=max_Y-min_Y;
boolean invert_X=false;
boolean invert_Y=false;
//Home position for the dish.  This should be the resting position for the dish.
int Home_X=30;
int Home_Y=150;
int current_X=Home_X;
int current_Y=Home_Y;

//Analog input port to read signal strength from
int signalStrengthPort=0;
Servo servoX;
Servo servoY;

#define LNBP_ENABLE 4 //digital pin 4 for LNB power on.
#define ENT_LNBP_BAND_SELECT 3
#define VSel_POLARIZATION_SELECT 2
byte Polarization=1; //High=Horizontal, low=Vertical
byte Band=0; //Low=Low band, High=Highband
#define PolarizationString "PSVertical,Horizontal"
#define BandsString "BS10.70GHz–11.70GHz,11.70GHz–12.75GHz"

int settlingTime_mSeconds=5; //time for the dish to settle after moving one degree
int slowMove_mSeconds=20;  //pause time between steps when making large moves

byte SamplingRates[]={116,72,64,56,48,40,32,24,16,8};
#define NUMBEROFSAMPLINGRATES 10 
byte SelectedSamplingRate_Index=3;
#define SamplingRateString "6.875Hz,13.75Hz,27.5Hz,55Hz,110Hz,220Hz,440Hz,880Hz,1.76kHz,3.52kHz"


/*
  Interface between Arduino DM board and Linear Tech LTC2440 24-bit ADC
  Oct. 24 2012 John Beale

   LTC2440  <---------->  Arduino
   10: /EXT : ground (use external serial clock)
   11: /CS <- to digital pin 10  (SS pin)
   12: MISO -> to digital pin 12 (MISO pin)
   13: SCLK <- to digital pin 13 (SCK pin)
   15: BUSY -> to digital pin 9 (low when result ready)

   1,8,9,16: GND :  all grounds must be connected
   2: Vcc :   +5V supply
   3: REF+ :  +2.5V reference
   4: REF- :  GND
   5: IN+  :  Input+
   6: IN-  :  Input-
   7: SDI  : +5V  (select 6.9 Hz output rate, or GND for 880 Hz rate)
   14: Fo : GND  (select internal 9 MHz oscillator)

*/
#define VREF (5)    // ADC voltage reference
#define PWAIT (10)   // milliseconds delay between readings
#define SLAVESELECT 10  // digital pin 10 for CS/
#define BUSYPIN 9     // digital pin 9 for BUSY or READY/
#define UNIT "UdBm"


void setup() {
  servoX.attach(servo_x_port);
  servoY.attach(servo_y_port);
  set_servo(servoX,Home_X);
  set_servo(servoY,Home_Y);
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  pinMode (SLAVESELECT, OUTPUT);
  pinMode(LNBP_ENABLE, OUTPUT);
  pinMode(ENT_LNBP_BAND_SELECT,OUTPUT);
  pinMode(VSel_POLARIZATION_SELECT,OUTPUT);
  pinMode(BUSYPIN, INPUT);
  digitalWrite(LNBP_ENABLE,HIGH);
  digitalWrite(VSel_POLARIZATION_SELECT,Polarization);
  digitalWrite(ENT_LNBP_BAND_SELECT,Band);
  digitalWrite(SLAVESELECT,LOW);   // take the SS pin low to select the chip
  delayMicroseconds(1);
  digitalWrite(SLAVESELECT,HIGH);   // take the SS pin high to start new ADC conversion

  SPI.begin(); // initialize SPI, covering MOSI,MISO,SCK signals
  SPI.setBitOrder(MSBFIRST);  // data is clocked in MSB first
  SPI.setDataMode(SPI_MODE0);  // SCLK idle low (CPOL=0), MOSI read on rising edge (CPHI=0)
  SPI.setClockDivider(SPI_CLOCK_DIV16);  // set SPI clock at 1 MHz. Arduino xtal = 16 MHz, LTC2440 max = 20 MHz
  for (int i=0;i<2;i++) {  // throw away the first few readings, which seem to be way off
    SpiRead();
  }

}

void loop() {
  if (Serial.available() > 0){
    //delay(1);
    int numChar = Serial.available();
    byte received_byte=0;
    while (numChar--){
      received_byte=Serial.read();
      //execute on carriage return
      if (received_byte==13){
        handle_message();
      }
      else {
        //discard line feeds
        if (received_byte!=10) {
          message_buffer=message_buffer + char(received_byte);
        }
      }
    }    
  }
}

void handle_message(){
  boolean handled=false;
  if (message_buffer.length()>=2) {
    if (message_buffer.startsWith("G")){
      handled=handle_command();
      if (handled) {
        send_OK();
      }
      else{
        send_UNK(); 
       handled=true; 
      }  
    }
    if (message_buffer.startsWith("Q")){
      handled=handle_query(); 
      if (!handled){
        send_UNK();
        handled=true;
      } 
    }
    if (message_buffer.startsWith("S")){
      handled=handle_setting(); 
       if (handled) {
        send_OK();
      }
      else{
        send_UNK(); 
       handled=true; 
      }  
    }
  }
  if (!handled){
    send_UNK();
  }
  message_buffer="";
  Serial.flush();
}

boolean handle_query(){
  boolean handled=false;
  int value=0;

  if (message_buffer.equals("QHX")) {
    value=translate_ServoAngle_to_X(Home_X);
    Serial.println("HX"+String(value));
    handled=true;
  }
  
  if (message_buffer.equals("QHY")) {
    value=translate_ServoAngle_to_Y(Home_Y);
    Serial.println("HY"+String(value));
    handled=true;
  }
  
  if (message_buffer.equals("QRX")) {
    Serial.println("RX"+String(range_X));
    handled=true;
  }
  if (message_buffer.equals("QRY")) {
    Serial.println("RY"+String(range_Y));
    handled=true;
  }
  
  if (message_buffer.equals("QPX")) {
    value=translate_ServoAngle_to_X(current_X);
    Serial.println("PX"+String(value));
    handled=true;
  }
  
  if (message_buffer.equals("QPY")) {
    value=translate_ServoAngle_to_Y(current_Y);
    Serial.println("PY"+String(value));
    handled=true;
  }
  
  if (message_buffer.equals("QP")) {
    Serial.println("P"+String(Polarization));
    handled=true;
  }
  
  if (message_buffer.equals("QPS")) {
    Serial.println(PolarizationString);
    handled=true;
  }
  
  if (message_buffer.equals("QB")) {
    Serial.println("B"+String(Band));
    handled=true;
  }

  if (message_buffer.equals("QBS")) {
    Serial.println(BandsString);
    handled=true;
  }
  
  if (message_buffer.equals("QR")) {
    Serial.println("RS"+String(SelectedSamplingRate_Index));
    handled=true;
  }
  
  if (message_buffer.equals("QRS")) {
    Serial.println("RR"+String(SamplingRateString));
    handled=true;
  }
  
  if (message_buffer.startsWith("QS")) {
    value=get_value_from_buffer();
    read_signalstrength(value);
    handled=true;
  }
  
  if (message_buffer.startsWith("QU")) {
    Serial.println(UNIT);
    handled=true;
  }
  return handled;
}

boolean handle_command(){
  boolean ok=false;
  if (message_buffer.equals("GH")) {
    current_X=move_dish(servoX,Home_X,current_X);
    current_Y=move_dish(servoY,Home_Y, current_Y); 
    ok=true;
  }
  
  if (message_buffer.startsWith("GX")){
    int gotox=get_value_from_buffer();
    gotox=translate_X_to_ServoAngle(gotox);    
    current_X=move_dish(servoX,gotox,current_X);
    ok=true;
  }
  
  if (message_buffer.startsWith("GY")){
    int gotoy=get_value_from_buffer();
    gotoy=translate_Y_to_ServoAngle(gotoy);    
    current_Y=move_dish(servoY,gotoy,current_Y);
    ok=true;
  }
  return ok;
}
boolean handle_setting(){
  boolean ok=false;
  if (message_buffer.startsWith("SS")) {
    int si=get_value_from_buffer();
    if (si>=0 and si<NUMBEROFSAMPLINGRATES) {
      SelectedSamplingRate_Index=si;
      ok=true;
    }    
  }
  if (message_buffer.startsWith("SB")) {
    int value=get_value_from_buffer();
    if (value>=0 and value<2) {
      Band=value;
      digitalWrite(ENT_LNBP_BAND_SELECT,Band);
      ok=true;
    }    
  }
  if (message_buffer.startsWith("SP")) {
    int value=get_value_from_buffer();
    if (value>=0 and value<2) {
      Polarization=value;
      digitalWrite(VSel_POLARIZATION_SELECT,Polarization);
      ok=true;
    }    
  }
  
  return ok;
}

int get_value_from_buffer(){
  int value=-1;
  if (message_buffer.length()>2){
    value=message_buffer.substring(2).toInt();
  }
  return value;
}

int translate_ServoAngle_to_X(int XAngle){
  XAngle=XAngle-min_X;
  if (invert_X) {
    XAngle=max_X-XAngle-min_X;
  }
  return XAngle;
}

int translate_ServoAngle_to_Y(int YAngle){
  YAngle=YAngle-min_Y;
  if (invert_Y) {
    YAngle=max_Y-YAngle-min_Y;
  }
  return YAngle;
}

int translate_X_to_ServoAngle(int X){
  if (invert_X) {
    X=max_X-X;
  }
  else{
    X=X+min_X;
  }
  
  if (X<min_X) {
    X=min_X;
  }
  if (X>max_X) {
    X=max_X;
  }
  return X;
}

int translate_Y_to_ServoAngle(int Y){
  if (invert_Y) {
    Y=max_Y-Y;
  }
  else{
    Y=Y+min_Y;
  }
  if (Y<min_Y) {
    Y=min_Y;
  }
  if (Y>max_Y) {
    Y=max_Y;
  }
  return Y;
}

void send_UNK(){
  Serial.println("UNK");
  Serial.flush();
}

void send_OK(){
  Serial.println("OK");
  Serial.flush();
}

int move_dish(Servo servo,int targetangle, int current_angle){
  int delaytime=settlingTime_mSeconds;
  if (targetangle<0){
    targetangle=0;
  }
  if (targetangle>180) {
    targetangle=180;
  }
  if (abs(targetangle-current_angle)>1){
    delaytime=slowMove_mSeconds;
  }
  while (targetangle!=current_angle){
    if (current_angle>targetangle){
      current_angle -=1; 
    }
    if (current_angle<targetangle){
      current_angle +=1; 
    }    
    set_servo(servo,current_angle);
    delay(delaytime);
  }
  return current_angle;
}

void set_servo(Servo servo,int angle){
  if (angle>=0 && angle<180){
    servo.write(angle);
  }
}

void read_signalstrength(int signalstrength_Readcount){
  if (signalstrength_Readcount>=1 && signalstrength_Readcount<=2048){
    int count=signalstrength_Readcount;
    float portvalue=0;
    while (count--){
      portvalue=portvalue+SpiRead();//analogRead(signalStrengthPort);
    }
    portvalue=portvalue/signalstrength_Readcount;
    portvalue=dBFromVolts(portvalue);
    Serial.print("S");
    Serial.print(portvalue,12);
    Serial.println("");
  }
}

float dBFromVolts(float volts){
  float dB=-70.0+(volts-0.5)*52.6315789474;
  return dB;
}


// =================================================================
// SpiRead() -- read 4 bytes from LTC2440 chip via SPI, return Volts
// =================================================================

float SpiRead(void) {

  long result = 0;
  byte sig = 0;    // sign bit
  byte b;
  float v;

  while (digitalRead(BUSYPIN)==HIGH) {}  // wait until ADC result is ready
 
  digitalWrite(SLAVESELECT,LOW);   // take the SS pin low to select the chip
  delayMicroseconds(1);  // probably not needed, only need 25 nsec delay

  b = SPI.transfer(SamplingRates[SelectedSamplingRate_Index]);   // B3   27.5 Samples per second
  if ((b & 0x20) ==0) sig=1;  // is input negative ?
  b &=0x1f;    // discard bits 25..31
  result = b;
  result <<= 8; 
  b = SPI.transfer(0xff);   // B2
  result |= b;
  result = result<<8;
  b = SPI.transfer(0xff);   // B1
  result |= b;
  result = result<<8;
  b = SPI.transfer(0xff);   // B0
  result |= b;
   
  digitalWrite(SLAVESELECT,HIGH);   // take the SS pin high to bring MISO to hi-Z and start new conversion
 
  if (sig) result |= 0xf0000000;    // if input is negative, insert sign bit (0xf0.. or 0xe0... ?)       
  v = result;
  v = v / 16.0;                             // scale result down , last 4 bits are "sub-LSBs"
  v = v * VREF / (2 * 16777216); // +Vfullscale = +Vref/2, max scale (2^24 = 16777216)
  return(v); 
}




