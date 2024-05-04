  #include "Timer.h"
  #include "math.h"

// SECTION 1: CONTROL PARAMETERS - to be modified by students
  //Controller sample time
  double T_sample = 0.001;//seconds
  
  // A matrix
  double a11 = 0;
  double a12 = 0;
  double a13 = 1;
  double a14 = 0;
  double a21 = 0;
  double a22 = 0;
  double a23 = 0;
  double a24 = 1;
  double a31 = 0;
  double a32 = 2.1005;
  double a33 = -7.2168;
  double a34 = 0;
  double a41 = 0;
  double a42 = 36.0401;
  double a43 = -21.8557;
  double a44 = 0;
  
  // B matrix
  double b1 = 0;
  double b2 = 0;
  double b3 = 1.6089;
  double b4 = 4.8725;
  
  //Observer states
  double L_11 = 82.7832;
  double L_21 = -21.8557;
  double L_31 = 1402.5736;
  double L_41 = -1809.2865;
  
  double L_12 = -1609.5581;
  double L_22 = 90.0000;
  double L_32 = 2.1005;
  double L_42 = 2036.0401;
  
  // Controller's gains
  double k1 = 44.7214;
  double k2 = -136.7657;
  double k3 = 39.1388;
  double k4 = -18.6723;

  //Signal Generator parameters
  double a_sg= 0.02; //signal generator amplitude (m)
  double f_sg= 0.05; // signal generator frequency (Hz)
  double T_sg= 1/f_sg; //signal generator period
  double x_sg= -a_sg; //signal generator initial 

// SECTION 2: INITIALIZATIONS - do not modify
  //timer
  Timer t;

  int sign; //used to determine lifting clockwise or counterclockwise
  int calibrated = 0; //Calibration variable for pendulum angle
  boolean play = true; //used to stop experiment when pendulum falls
    
  //Cart and Pendulum Encoder pins
  enum PinAssignments {
    //Cart Encoder signals A and B
    encoderPinA = 2,// brown wire, blk wire gnd 
    encoderPinB = 3,//green wire, red wire 5v
  
    //Pendulum Encoder signals A and B
    encoderPinA_P = 18, //lab Din5 blk wire
    encoderPinB_P = 19, //lab Din5 orange wire
  };

  //Declare volatile int to retrieve encoderPos from memory 
  //This is required for variables that may have a change during a interrupt
  //function which may cause a mismatch between ram and memory data
  volatile int encoderPos = 0; //count cart ticks
  volatile int encoderPos_P = 0; //count pendulum ticks
  
  //Boolean variables for determining encoder position changes
  boolean A_set = false;
  boolean B_set = false;
  boolean A_set_P = false;
  boolean B_set_P = false;
  
  //Constants to map encoder clicks to position and angle respectively
  const float pi = 3.14;
  double K_encoder = 2.2749*0.00001;
  double K_encoder_P = 2*pi*0.000244140625;

  //Motor Controlling pins
  int PWM_A   = 11;//Pin to determine the magnitude of applied motor voltage
  int DIR_A   = 8;//Pin to determine the polarity of applied motor voltage

  //Motor computed duty cycle
  int Duty_cycle; 
  //Initial controller's voltage value
  double u =0;

  //Data trasnfer rate
  double T_plot = 0.03;//in seconds
  //Total elapsed time
  double T_tot = 0;

  //Observer initial states
  double x1_hat = 0;
  double x2_hat = 0;
  double x3_hat = 0;
  double x4_hat = 0;
  
  //derivative of observer states
  double x1_hat_dot;
  double x2_hat_dot;
  double x3_hat_dot;
  double x4_hat_dot;
  
// SECTION 3: SET-UP FUNCTION - do not modify
void setup() {
  
  //Set up Cart Encoder pins - Arduino input
  pinMode(encoderPinA, INPUT); 
  pinMode(encoderPinB, INPUT); 
  digitalWrite(encoderPinA, HIGH);  // turn on pullup resistor
  digitalWrite(encoderPinB, HIGH);  // turn on pullup resistor
  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(1, doEncoderB, CHANGE);
  
  //Set up Pendulum Encoder pins - Arduino input
  pinMode(encoderPinA_P, INPUT); 
  pinMode(encoderPinB_P, INPUT); 
  digitalWrite(encoderPinA_P, HIGH);  // turn on pullup resistor
  digitalWrite(encoderPinB_P, HIGH);  // turn on pullup resistor
  attachInterrupt(5, doEncoderA_P, CHANGE); //int.5 on pin18
  attachInterrupt(4, doEncoderB_P, CHANGE); //int.4 on pin 19

  //Set up serial link rate
  Serial.begin (115200);
  
  //Motor pins
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT); 
  
  //Clock setup to increase PWM frequency controlling applied voltage on PIN 11
  TCCR1A = _BV(COM1A1) | _BV(WGM21) | _BV(WGM20);
  TCCR1B = _BV(CS10);
  OCR1A = 0;
   
  // Perform takeReading every T_sample (t.every takes in ms)
  t.every(T_sample*1000,takeReading);
  // Perform Plot every T_plot (t.every takes in ms)
  t.every(T_plot*1000,Plot);
  //Square wave signal generator
  t.every(T_sg*1000/2,signalGenerator); 
}


// SECTION 4: MAIN PROCESSING LOOP - do not modify
void loop(){ 

  //Stop experiment when pendulum falls (safety)
  if((play == false) || (calibrated==1 && abs(encoderPos_P)>=500)){
    doAfter();
  }  
  
  // Update the direction of motor based on the sign of Duty_cycle  
  else {
    if (Duty_cycle>0){
      digitalWrite(DIR_A,HIGH);
      OCR1A = Duty_cycle;
    }
 
    else if (Duty_cycle<=0){
      digitalWrite(DIR_A,LOW);
      OCR1A = -Duty_cycle ;
    }
  }
  // Update timer to trigger scheduled functions below
  t.update();
}


// SECTION 5: CONTROL INPUT COMPUTATION
void takeReading(){
  if (calibrated==1){
    // Total experiment time update   
    T_tot = T_tot+T_sample;   //T_tot in seconds, for tracking 

    // OBSERVER - to be modified by students
    x1_hat_dot = a11*x1_hat + a12*x2_hat + a13*x3_hat + a14*x4_hat + b1*u + L_11*(encoderPos*K_encoder-x_sg-x1_hat) + L_12*(encoderPos_P*K_encoder_P-x2_hat);
    x2_hat_dot = a21*x1_hat + a22*x2_hat + a23*x3_hat + a24*x4_hat + b2*u + L_21*(encoderPos*K_encoder-x_sg-x1_hat) + L_22*(encoderPos_P*K_encoder_P-x2_hat);
    x3_hat_dot = a31*x1_hat + a32*x2_hat + a33*x3_hat + a34*x4_hat + b2*u + L_31*(encoderPos*K_encoder-x_sg-x1_hat) + L_32*(encoderPos_P*K_encoder_P-x2_hat);
    x4_hat_dot = a41*x1_hat + a42*x2_hat + a43*x3_hat + a44*x4_hat + b4*u + L_41*(encoderPos*K_encoder-x_sg-x1_hat) + L_42*(encoderPos_P*K_encoder_P-x2_hat);

    // Update old states of the observer using the Euler Method
    x1_hat = x1_hat + x1_hat_dot*T_sample;
    x2_hat = x2_hat + x2_hat_dot*T_sample;
    x3_hat = x3_hat + x3_hat_dot*T_sample;
    x4_hat = x4_hat + x4_hat_dot*T_sample;
    
    // CONTROLLER - to be modified by students
    u = k1*x1_hat + k2*x2_hat + k3*x3_hat + k4*x4_hat;

    //Mapping between required voltage and 11.75V motor shield
    Duty_cycle = round(u/11.75*1024);

    //Saturation to not exceed motor voltage limits of 5.875 V
    if (Duty_cycle>512 ){
      Duty_cycle=512; //motor moves left
    }
     
    else if (Duty_cycle<-512 ){
      Duty_cycle=-512;//motor moves right
    }   
  }

//Measure lift up direction

    else if(abs(encoderPos_P)>=2048) {
        if(encoderPos_P<0){
          sign=-1;
        }
        else{
          sign=+1;
        }
      encoderPos_P=encoderPos_P-sign*2048;
      calibrated=1;
      encoderPos = 0; //negative moves right
    }
}  

// SECTION 6: INTERRUPTS/FUNCTIONS  - do not modify

// Interrupt on A changing state
void doEncoderA(){
  // Test transition
  A_set = digitalRead(encoderPinA) == HIGH;
  // and adjust counter + if A leads B
  encoderPos += (A_set != B_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoderB(){
  // Test transition
  B_set = digitalRead(encoderPinB) == HIGH;
  // and adjust counter + if B follows A 
  encoderPos += (A_set == B_set) ? +1 : -1;
}
 
//Pendulum
// Interrupt on A changing state
void doEncoderA_P(){
  // Test transition
  A_set_P = digitalRead(encoderPinA_P) == HIGH;
  // and adjust counter + if A leads B
  encoderPos_P += (A_set_P != B_set_P) ? +1 : -1; 
}

// Interrupt on B changing state
void doEncoderB_P(){
    // Test transition
    B_set_P = digitalRead(encoderPinB_P) == HIGH;
    // and adjust counter + if B follows A
    encoderPos_P += (A_set_P == B_set_P) ? +1 : -1;
  }

// Signal generator
void signalGenerator(){
  x_sg=-x_sg;
}

//Function to output controller states to serial link with 5 decimal places
void Plot(){
  Serial.println(encoderPos*K_encoder,5);
  Serial.println(encoderPos_P*K_encoder_P,5);
  Serial.println(x_sg,5);
  Serial.println(x3_hat,5);
  Serial.println(x4_hat,5);
}


//Function to stop experiment
void doAfter(){  
  //Turn off applied voltage to stop experiment
  OCR1A = 0; 
  play = false;
  // delay(90000000);
}
