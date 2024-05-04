  #include "Timer.h"
  #include "math.h"

// SECTION 1: CONTROL PARAMETERS - to be modified by students
  //Controller sample time
  double T_sample = 0.001;//seconds
  
  // A matrix
  double a11 = ;
  double a12 = ;
  double a21 = ;
  double a22 = ;
  
  // B matrix
  double b1 = ;
  double b2 = ;

  //Controller gains - K matrix
  double k1 = ;
  double k2 = ;
  
  //Observer gains - L matrix
  double L1 = ;
  double L2 = ;

  //Signal Generator parameters
  double a_sg= ; //signal generator amplitude (m)
  double f_sg= ; // signal generator frequency (Hz)
  double T_sg= 1/f_sg; //signal generator period
  double x_sg= -a_sg; //signal generator initial condition
  
// SECTION 2: INITIALIZATIONS - do not modify
  //timer
  Timer t;
  
  boolean play = true; //used to stop experiment
    
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
  double z1_hat = 0;
  double z2_hat = 0;

  //derivative of observer states
  double z1_hat_dot;
  double z2_hat_dot;
  
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
  
  // Update the direction of motor based on the sign of Duty_cycle  
    if (Duty_cycle>0){
      digitalWrite(DIR_A,HIGH);
      OCR1A = Duty_cycle;
    }
 
    else if (Duty_cycle<=0){
      digitalWrite(DIR_A,LOW);
      OCR1A = -Duty_cycle ;
    }
  // Update timer to trigger scheduled functions below
  t.update();
}


// SECTION 5: CONTROL INPUT COMPUTATION
void takeReading(){
    // Total experiment time update   
    T_tot = T_tot+T_sample;   //T_tot in seconds, for tracking

    // OBSERVER - to be modified by students

    z1_hat_dot = ;
    z2_hat_dot = ;

    // Update old states of the observer using the Euler Method
    z1_hat = z1_hat + z1_hat_dot*T_sample;
    z2_hat = z2_hat + z2_hat_dot*T_sample;
    
    // CONTROLLER - to be modified by students
    u = ; 

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

// SECTION 6: INTERRUPTS/FUNCTIONS - do not modify

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
  
//Function to output controller states to serial link
void Plot(){
  Serial.println(encoderPos*K_encoder,5);
  Serial.println(x_sg,5);
}


//Function to stop experiment
void doAfter(){  
  //Turn off applied voltage to stop experiment
  OCR1A = 0; 
  play = false;
}
