#include <Servo.h>

String command_buffer;
int servo_x_port=9;
int servo_y_port=10;
//Park position for the dish.  This should be the resting position for the dish.
int Park_X=30;
int Park_Y=120;
int current_X=Park_X;
int current_Y=Park_Y;
int signalstrength_Readcount=64;
Servo servoX;
Servo servoY;

void setup() {
  servoX.attach(servo_x_port);
  servoY.attach(servo_y_port);
  set_servo(servoX,Park_X);
  set_servo(servoY,Park_Y);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Servo controller ready!");
  Serial.flush();

}

void loop() {
  if (Serial.available() > 0){
    delay(20);
    int numChar = Serial.available();
    byte received_byte=0;
    while (numChar--){
      received_byte=Serial.read();
      //execute on carriage return
      if (received_byte==13){
        execute_command();
      }
      else {
        //discard line feeds
        if (received_byte!=10) {
          command_buffer=command_buffer + char(received_byte);
        }
      }
    }    
  }
}

void execute_command(){
  int value=0;
  if (command_buffer.length()>=2){
    value=command_buffer.substring(1).toInt();
    if (command_buffer.startsWith("X")){
      current_X=move_dish(servoX,value,current_X);  
    }
    if (command_buffer.startsWith("Y")){
      current_Y=move_dish(servoY,value, current_Y);  
    }
    if (command_buffer.startsWith("R")){
      read_signalstrength(value);
    }
    if (command_buffer.startsWith("P")){
      current_X=move_dish(servoX,value,current_X);
      current_Y=move_dish(servoY,value, current_Y); 
    }
  }
  command_buffer="";
}

int move_dish(Servo servo,int targetangle, int current_angle){
  if (targetangle<0){
    targetangle=0;
  }
  if (targetangle>180) {
    targetangle=180;
  }
  while (targetangle!=current_angle){
    if (current_angle>targetangle){
      current_angle -=1;      
    }
    if (current_angle<targetangle){
      current_angle +=1;      
    }    
    set_servo(servo,current_angle);
    delay(20);
  }
  return current_angle;
}

void set_servo(Servo servo,int angle){
  if (angle>=0 && angle<180){
    servo.write(angle);
  }
}

void read_signalstrength(int port){
  if (port>=0 && port<=5){
    int bytestoread=signalstrength_Readcount;
    int portvalue=0;
    while (bytestoread--){
      portvalue=portvalue+analogRead(port);
    }
    Serial.print("A" + String(portvalue)+ char(13)+ char(10));
  }
}

