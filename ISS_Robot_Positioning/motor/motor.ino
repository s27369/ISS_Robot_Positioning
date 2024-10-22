// #define PIN_DIRECTION_FWD 14
// #define PIN_DIRECTION_REV 15
// #define PIN_DIRECTION_PWM 5
#define DIRECTION_SWITCH_DELAY 20
#define PIN_ENCODER_INTERRUPT 2

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
    set_speed(SPEED_MOTOR_TEST);
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
    Serial.print("Motor#");
    Serial.print(id);
    Serial.print(" Interrupts per second = ");
    Serial.println(counter);
    counter=0;
  }

  int get_counter(){
    int c = counter;
    counter=0;
    return c;
  }

  int get_speed(){
    return speed;
  }

};

class Vehicle{
  
  //delta_rotation = (dist_r-dist_l)/l
  //dist_total = (dist_r+dist_l)/2
  //turn_angle = (L/2)*(dist_l+dist_r)/(dist_r-dist_l)
  //new_x = posX_prev-turn_angle*sin(rotation_prev)+turn_angle*sin(rotation_prev+delta_rotation)
  //new_y = posY_prev+turn_angle*cos(rotation_prev)-turn_angle*cos(rotation_prev+delta_rotation)
  //new_rotation = rotation_prev+delta_rotation

  
  public:
    Motor motor_l, motor_r;
    float 
    posX_now, posY_now, 
    posX_prev, posY_prev, 
    dist_l, dist_r,
    t_delta, l,
    rotation_now, rotation_prev;
    int motor_speed;
    const int MOTOR_TEST_SPEED = 50;

  Vehicle( Motor m_l, Motor m_r) 
    : motor_l(m_l), motor_r(m_r){
      posX_now=0;
      posY_now=0;
      posX_prev=0;
      posY_prev=0;
      dist_l=0;
      dist_r=0; 
      t_delta=0; 
      l=0;
      rotation_now=0;
      rotation_prev=0;
      motor_l.set_speed(0);
      motor_r.set_speed(0);
      motor_speed=motor_l.get_speed();
  }

  float get_delta_rotation(){
    return (dist_r-dist_l)/l;
  }
  float get_dist_total(){
    return (dist_r+dist_l)/2;
  }
  float get_turn_angle(){
    return (l/2)*(dist_l+dist_r)/(dist_r-dist_l);
  }
  float get_new_x(){
    return posX_prev-get_turn_angle()*sin(rotation_prev)+get_turn_angle()*sin(get_new_rotation());
  }
  float get_new_y(){
    return posY_prev+get_turn_angle()*cos(rotation_prev)-get_turn_angle()*cos(get_new_rotation());
  }
  float get_new_rotation(){
    return rotation_prev+get_delta_rotation();
  }

  void move(float dist){
    //s=v*t
    //v=s/t
    //t=s/v
    //calculate time
    int t = (dist/MOTOR_TEST_SPEED)*1000;
    Serial.println(t);
    //set direction
    direction dir = dist>=0?FORWARD:REVERSE;
    motor_l.set_direction(dir);
    motor_r.set_direction(dir);
    //movement
    set_speed(MOTOR_TEST_SPEED);
    delay(t);
    stop();
    //increment motor counters
    if (dist>=0){
      dist_l += motor_l.get_counter();
      dist_r += motor_r.get_counter();
    }else{
      dist_l -= motor_l.get_counter();
      dist_r -= motor_r.get_counter();
    }//dobrze???
    //get new x, y
    posX_prev = posX_now;
    posY_prev = posY_now;

    // posX_now = get_new_x();
    // posY_now = get_new_Y();
    //dobrze???
    //

    print_location_info();
  }

  void turn(float radians){

    //delta_rotation = (dist_r-dist_l)/l
    //dist_r = delta_rotation*l+dist_l
    //dist_l = -(delta_rotation*l-dist_r)
    float d;
    if (radians>=0){
      //r
      d = get_delta_rotation()*l+dist_l;
    }else{
      d = -(get_delta_rotation()*l-dist_r);
    }

    //get new x, y
    posX_prev = posX_now;
    posY_prev = posY_now;
    posX_now = get_new_x();
    posY_now = get_new_y();

    print_location_info();
  }

  void set_speed(int speed){
      motor_l.set_speed(speed);
      motor_r.set_speed(speed);
      motor_speed=speed;
  }

  void stop(){
    set_speed(0);
    motor_l.set_direction(DISABLED);
    motor_r.set_direction(DISABLED);
  }

  void print_location_info(){
    Serial.print(posX_now);
    Serial.print(" ");
    Serial.print(posY_now);
    Serial.print(" ");
    Serial.println(rotation_now);
  }
  
  void handle_command(String command){
    if (command.equals("R")){
      posX_now=0;
      posY_now=0;
      posX_prev=0;
      posY_prev=0;
      rotation_now=0;
      rotation_prev=0;
      return;
    }else{
      String s = command.substring(0, command.indexOf(' '));
      if (s.equals("M")){
        int start_pos = command.indexOf(' ')+1;
        float dist = command.substring(start_pos).toFloat();
        move(dist);
        return;
      }
      else if (s.equals("T")){
        int start_pos = command.indexOf(' ')+1;
        float radians = command.substring(start_pos).toFloat();
        turn(radians);
        return;
      }
    }
    Serial.println("Incorrect command");
  }

};

Motor motor1 = Motor(MTR1_PIN_FWD, MTR1_PIN_REV, MTR1_PIN_PWM);
Motor motor2 = Motor(MTR2_PIN_FWD, MTR2_PIN_REV, MTR2_PIN_PWM);
Vehicle vehicle = Vehicle(motor1, motor2);

void encoder_handler_1(){
  motor1.increment_counter();
}

void encoder_handler_2(){
  motor2.increment_counter();
}



void setup() 
{
  delay(1000);
  Serial.begin(SERIAL_BAUD_RATE);
  motor1.initialize(); //hardware
  motor2.initialize(); //hardware
  
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_INTERRUPT), encoder_handler_1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_INTERRUPT), encoder_handler_2, RISING);
  
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

void loop()
{
  if (stringComplete) {
    vehicle.handle_command(inputString);

    inputString = "";
    stringComplete = false;
  }
  serialEvent();
}
