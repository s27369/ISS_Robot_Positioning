// #define PIN_DIRECTION_FWD 14
// #define PIN_DIRECTION_REV 15
// #define PIN_DIRECTION_PWM 5
#define DIRECTION_SWITCH_DELAY 20
#define PIN_ENCODER_INTERRUPT_L 2
#define PIN_ENCODER_INTERRUPT_P 3

#define MTR1_PIN_FWD 14
#define MTR1_PIN_REV 15
#define MTR1_PIN_PWM 5

#define MTR2_PIN_FWD 17
#define MTR2_PIN_REV 16
#define MTR2_PIN_PWM 6

#define SERIAL_BAUD_RATE 9600
#define SPEED_MOTOR_STOPPED 0
#define SPEED_MOTOR_TEST 150
#define MEASUREMENT_TIME_MS 1000

enum direction{FORWARD, REVERSE, DISABLED};
static int id_counter=0;

class Motor{
  public:
  
  const int GPIO1, GPIO2, PWM;
  int counter, id, speed;
  
  Motor( int pin_fwd, int pin_rev, int pin_pwm) 
    : GPIO1(pin_fwd), GPIO2(pin_rev), PWM(pin_pwm){
      //lista inicjalizacyjna
      id = getUniqueId();
      speed=0;
  }

  int getUniqueId(){
    return ++id_counter;
  }

  void initialize(){
    pinMode(GPIO1, OUTPUT);
    digitalWrite(GPIO1, LOW);

    pinMode(GPIO2, OUTPUT);
    digitalWrite(GPIO2, LOW);

    pinMode(PWM, OUTPUT);
    analogWrite(PWM, 0);

    
    counter = 0;

    set_direction(FORWARD);
    // set_speed(SPEED_MOTOR_TEST);
  }

  void set_speed(unsigned char value){
    analogWrite(PWM, value);
    speed=value;
  }
  
  void set_direction(enum direction dir){
    switch(dir){
      case FORWARD:
        digitalWrite(GPIO2, LOW);
        delay(DIRECTION_SWITCH_DELAY);
        digitalWrite(GPIO1, HIGH); 
        break; 
      
      case REVERSE:
        digitalWrite(GPIO1, LOW);
        delay(DIRECTION_SWITCH_DELAY);
        digitalWrite(GPIO2, HIGH); 
        break;
      
      case DISABLED:
        digitalWrite(GPIO1, LOW);
        digitalWrite(GPIO2, LOW); 
        break;
    }
  }
  
  void stop(){
    set_direction(DISABLED);
    set_speed(0);
  }

  void increment_counter(){
    counter++;
  }
  

  void print_counter(){
    Serial.print("id: ");
    Serial.print(counter);
  }

  int get_counter(){
    return counter;
  }
  void reset_counter(){
    counter=0;
  }

  int get_speed(){
    return speed;
  }
};

class Vehicle{

  public:
    Motor& motor_l, &motor_r;
    int
    dist_l, dist_r,
    t_delta,
    rotation_now;
    int motor_speed;
    const int MOTOR_TEST_SPEED = 150;

  Vehicle( Motor& m_l, Motor& m_r)
    : motor_l(m_l), motor_r(m_r){
      t_delta=1000;
      motor_l.set_speed(0);
      motor_r.set_speed(0);
      motor_speed=motor_l.get_speed();
  }

  void set_speed(int speed){
      motor_r.set_speed(speed);
      motor_l.set_speed(speed);
      motor_speed=speed;
  }

  void stop(){
    set_speed(0);
    motor_l.set_direction(DISABLED);
    motor_r.set_direction(DISABLED);
  }

  void move(int dist){
    //set direction
    direction dir = dist>=0?FORWARD:REVERSE;
    motor_l.set_direction(dir);
    motor_r.set_direction(dir);
    //movement
    set_speed(MOTOR_TEST_SPEED);
    int traveled=0;
    int temp=0;
    while(traveled<dist){
      delay(100);
      temp=motor_l.get_counter();
      traveled += temp;
      // Serial.println(traveled);
    }
    stop();
    //reset
    motor_l.get_counter();
    motor_r.get_counter();
  }

  void turn(int dist){
    stop();
    if (dist>=0){
      motor_l.set_direction(FORWARD);
      motor_r.set_direction(REVERSE);
    }else{
      motor_l.set_direction(REVERSE);
      motor_r.set_direction(FORWARD);
    }
    set_speed(MOTOR_TEST_SPEED);

    int traveled=0;
    int temp=0;
    while(traveled<dist){
      temp=motor_l.get_counter();
      traveled += temp;
    }
    stop();
    motor_l.get_counter();
    motor_r.get_counter();
  }

  void set_direction(direction dir){
    motor_l.set_direction(dir);
    motor_r.set_direction(dir);
  }

  void measure_speed(){
    Serial.println("---START---");

    for(int i=80; i<256; i+=20){
      //movement
      set_direction(FORWARD);
      set_speed(i);
      int traveled_l=0, traveled_r=0;
      delay(t_delta);
      traveled_l = motor_l.get_counter();
      traveled_r = motor_r.get_counter();
      stop();

      // Serial.print();
      // Serial.print(motor_speed);
      Serial.print(i);
      Serial.print(",");
      Serial.print(traveled_l);
      Serial.print(",");
      Serial.print(traveled_r);
      //reset
      motor_l.reset_counter();
      motor_r.reset_counter();
      delay(3000);
    }
    Serial.println("---END---");
  }

  void test_motors() {
    motor_l.set_direction(FORWARD);
    motor_l.set_speed(150);
    delay(2000);
    motor_l.set_speed(0);

    motor_r.set_direction(FORWARD);
    motor_r.set_speed(150);
    delay(2000);
    motor_r.set_speed(0);
  }


  void handle_command(String command){
      String s = command.substring(0, command.indexOf(' '));
      int start_pos = command.indexOf(' ')+1;
      int dist = command.substring(start_pos).toInt();
      if (s.equals("M")){
        move(dist);
        return;
      }
      else if (s.equals("T")){
        turn(dist);
        return;
      }
      else if (s.equals("?")){
        // test_motors();
        measure_speed();
        return;
      }
      Serial.println("Incorrect command");
  }

};

Motor motor1 = Motor(MTR1_PIN_FWD, MTR1_PIN_REV, MTR1_PIN_PWM);
Motor motor2 = Motor(MTR2_PIN_FWD, MTR2_PIN_REV, MTR2_PIN_PWM);
Vehicle vehicle = Vehicle(motor1, motor2);

void encoder_handler_1(){//LEFT
  motor1.increment_counter();
}

void encoder_handler_2(){//RIGHT
  motor2.increment_counter();
}



void setup()
{
  delay(1000);
  Serial.begin(SERIAL_BAUD_RATE);
  motor1.initialize(); //hardware
  motor2.initialize(); //hardware
  pinMode(PIN_ENCODER_INTERRUPT_L, INPUT);
  pinMode(PIN_ENCODER_INTERRUPT_P, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_INTERRUPT_L), encoder_handler_1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_INTERRUPT_P), encoder_handler_2, RISING);
}

String inputString = "";
bool stringComplete = false;

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void loop() {
  serialEvent();

  if (stringComplete) {
    vehicle.handle_command(inputString);
    inputString = "";
    stringComplete = false;
  }
}