#include <Servo.h>
String message_buffer;
int servo_x_port=9;
int servo_y_port=10;
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

int settlingTime_mSeconds=5; //time for the dish to settle after moving one degree
int slowMove_mSeconds=20;  //pause time between steps when making large moves

void setup() {
  servoX.attach(servo_x_port);
  servoY.attach(servo_y_port);
  set_servo(servoX,Home_X);
  set_servo(servoY,Home_Y);
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  //Serial.println("Servo controller ready!");
  //Serial.flush();

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
  if (message_buffer.startsWith("QS")) {
    value=get_value_from_buffer();
    read_signalstrength(value);
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
    unsigned long portvalue=0;
    while (count--){
      portvalue=portvalue+analogRead(signalStrengthPort);
    }
    Serial.println("S" + String(portvalue));
  }
}






