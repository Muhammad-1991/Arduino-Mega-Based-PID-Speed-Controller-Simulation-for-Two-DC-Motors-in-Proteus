//---------PIN DEFINITION FOR MOTOR 'A'--------//
#define ENCA_A 21   //Encoder A Pin of Motor A
#define ENCB_A 20   //Encoder B Pin of Motor A  
#define PWM_A   2   //Output to ENA
#define IN1_A   3   //Output to IN1
#define IN2_A   4   //Output to IN2

//---------PIN DEFINITION FOR MOTOR 'B'--------//
#define ENCA_B 18   //Encoder A Pin of Motor B
#define ENCB_B 19   //Encoder B Pin of Motor B  
#define PWM_B   5   //Output to ENB
#define IN1_B   6   //Output to IN3
#define IN2_B   7   //Output to IN4 

//// This alternate version of the code does not require
//// atomic.h. Instead, interrupts() and noInterrupts()
//// are used. Please use this code if your
//// platform does not support ATOMIC_BLOCK.
//// Global Variables
long  prevT = 0;          //Previous Time
float eprev_A = 0;        //Previous Error in Velocity of Motor A
float eprev_B = 0;        //Previous Error in Velocity of Motor B

int   posPrev_A = 0;      //Previous Position of Motor A
int   posPrev_B = 0;      //Previous Position of Motor B

//// Use the "volatile" directive for variables
//// used in an interrupt
volatile int pos_A_i = 0; //
volatile int pos_B_i = 0; //

float Va_Filt = 0;        //Filtered Velocity of Motor A
float Va_Prev = 0;        //Filtered Velocity of Motor A

float Vb_Filt = 0;        //Filtered Velocity of Motor B
float Vb_Prev = 0;        //Previous Velocity of Motor B

float eintegral_A = 0;    //Integral Part/Value of PID Controller for Motor A
float eintegral_B = 0;    //Integral Part/Value of PID Controller for Motor B

void setup()
{
  Serial.begin(9600);

  pinMode(ENCA_A, INPUT);/////
  pinMode(ENCB_A, INPUT);   //
  pinMode(PWM_A, OUTPUT);   //  Define Pin Mode for Motor A
  pinMode(IN1_A, OUTPUT);   //
  pinMode(IN2_A, OUTPUT);/////

  pinMode(ENCA_B, INPUT);/////
  pinMode(ENCB_B, INPUT);   //
  pinMode(PWM_B, OUTPUT);   //  Define Pin Mode for Motor B
  pinMode(IN1_B, OUTPUT);   //
  pinMode(IN2_B, OUTPUT);/////

  /////////Interrupts to Read Output from the Encoders///////////
  attachInterrupt(digitalPinToInterrupt(ENCA_A),
                  readEncoder_A, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCA_B),
                  readEncoder_B, RISING);
}

//
void loop() {
  // Set a target
  float vt = 125 * (sin(prevT / 1e6));
  //50 * (sin(prevT/0.5e6)>0) + 50 * (sin(prevT/1e6)>0) + 50 * (sin(prevT/2e6)>0) + 50 * (sin(prevT/4e6)>0)
  //-50 * (sin(prevT/1.5e6)>0) - 50 * (sin(prevT/3e6)>0) - 50 * (sin(prevT/2.5e6)>0) + 50 * (sin(prevT/5e6)>0);

  // Read the Positions of both the motors
  int pos_A = 0;
  int pos_B = 0;
  noInterrupts();   // disable interrupts temporarily while reading
  pos_A = pos_A_i;
  pos_B = pos_B_i;
  interrupts();     // turn interrupts back on

  // Compute velocities of both of the Motors
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  float velocity_A = (pos_A - posPrev_A) / deltaT;
  float velocity_B = (pos_B - posPrev_B) / deltaT;
  posPrev_A = pos_A;
  posPrev_B = pos_B;
  prevT = currT;
  // Convert count/s to RPM
  float Va = (velocity_A / 48.0) * 60.0;
  float Vb = (velocity_B / 48.0) * 60.0;
  
  // Apply Low-pass filters to the velocities of motors (25 Hz cutoff)
  Va_Filt = 0.854 * Va_Filt + 0.0728 * Va + 0.0728 * Va_Prev;
  Va_Prev = Va;
  Vb_Filt = 0.854 * Vb_Filt + 0.0728 * Vb + 0.0728 * Vb_Prev;
  Vb_Prev = Vb;
  //

  // Compute the control signals u_A & u_B
  float kp = 22.50;
  float kd = 04.75;
  float ki = 36.75;

  //Calculate the function for both the motors
  float e_A = vt - Va_Filt;
  float de_Adt = (e_A - eprev_A) / (deltaT);
  eintegral_A = eintegral_A + e_A * deltaT;
  float u_A = kp * e_A + kd * de_Adt + ki * eintegral_A;

  float e_B = vt - Vb_Filt;
  float de_Bdt = (e_B - eprev_B) / (deltaT);
  eintegral_B = eintegral_B + e_B * deltaT;
  float u_B = kp * e_B + kd * de_Bdt + ki * eintegral_B;

  // Set 'Motor A' Direction and Speed //
  //Direction 
  int dir_A = 1;
  if ( u_A < 0 ) {
    dir_A = -1;
  }
  //Speed
  int pwr_A = (int) fabs(u_A);
  if (pwr_A > 255) {
    pwr_A = 255;
  }

  // Set 'Motor B' Direction and Speed //
  //Direction
  int dir_B = 1;
  if ( u_B < 0 ) {
    dir_B = -1;
  }
  //Speed
  int pwr_B = (int) fabs(u_B);
  if (pwr_B > 255) {
    pwr_B = 255;
  }

  //Compute the voltage applied across Motor A
  float voltage_A = (float) pwr_A*dir_A;
  voltage_A = 12.0 * voltage_A/255.0;
  //Compute the voltage applied across Motor B
  float voltage_B = (float) pwr_B*dir_B;
  voltage_B = 12.0 * voltage_B/255.0;

  eprev_A = e_A;
  eprev_B = e_B;

  setMotor_A(dir_A, pwr_A, PWM_A, IN1_A, IN2_A);
  setMotor_B(dir_B, pwr_B, PWM_B, IN1_B, IN2_B);

  ////////////////////////////////////////////////////////////////////////////////
  Serial.print("Target_Velocity:");   Serial.print(vt);        Serial.print(", ");
  Serial.print("Voltage_A:");         Serial.print(voltage_A); Serial.print(", ");
  Serial.print("Voltage_B:");         Serial.print(voltage_B); Serial.print(", ");
  Serial.print("Velocity_Motor_A:");  Serial.print(Va_Filt);   Serial.print(", ");
  Serial.print("Velocity_Motor_B:");  Serial.print(Vb_Filt);   Serial.print(", ");
  Serial.println();
  ////////////////////////////////////////////////////////////////////////////////

  delay(1);
}

////////////////////////////Function Definitions//////////////////////////

void setMotor_A(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal); // Motor speed
  if (dir == -1) {
    // Turn one way
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == 1) {
    // Turn the other way
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    // Or dont turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void setMotor_B(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal); // Motor speed
  if (dir == -1) {
    // Turn Motor B one way
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == 1) {
    // Turn the other way
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    // Or dont turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

///////////////Interrupt Service Routines////////////////
////////////////////ISR for Motor A//////////////////////
void readEncoder_A() {

  int increment = 0;
  if (digitalRead(ENCB_A)) {
    // If ENCB_A is high, increment forward
    increment = 1;
  }
  else {
    // Otherwise, increment backward
    increment = -1;
  }
  pos_A_i = pos_A_i + increment;
}

//////////////////ISR for Motor B////////////////////////
void readEncoder_B() {

  int increment = 0;
  if (digitalRead(ENCB_B)) {
    // If ENCB_B is high, increment forward
    increment = 1;
  }
  else {
    // Otherwise, increment backward
    increment = -1;
  }
  pos_B_i = pos_B_i + increment;
}
