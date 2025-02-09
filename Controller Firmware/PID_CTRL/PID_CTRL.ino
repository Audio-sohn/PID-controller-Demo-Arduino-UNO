


// define this if serial debug output is needed
#define DEBUG
#define DEBUG_SPEED 1000

//============================================================================
// FW version
//============================================================================

#define FW_STRING "FIRMWARE VERSION 0.0.1"

//============================================================================
// Pinout configs 
//============================================================================

#define PIN_MOTOR_ENA 11
#define PIN_MOTOR_A   8
#define PIN_MOTOR_B   9

#define PIN_AIN_FB    14 // A0
#define PIN_AIN_SP    15 // A1
#define PIN_AIN_SP_P  16 // A2
#define PIN_AIN_SP_I  17 // A3
#define PIN_AIN_SP_D  18 // A4

#define PIN_DIN_PW    7 // Active HI
#define PIN_DIN_INV   6 // Active HI
#define PIN_DIN_MANA  5 // SW Up -> HI
#define PIN_DIN_MANB  4 // SW Dn -> HI


//============================================================================
// Config Parameters for calculations
//============================================================================

#define LOOP_SPEED_MS 10
#define DISP_ACCURACY 6 // decimal places to be printed in serial mon
#define ERROR_THRESH 5
#define MASTER_GAIN 2.0

const double PARAM_MAXGAIN_P = 2.0;
const double PARAM_MINGAIN_P = 0.5;

const double PARAM_MAXGAIN_I = 0.1;
const double PARAM_MINGAIN_I = 0.0;

const double PARAM_MAXGAIN_D = 1000.0;
const double PARAM_MINGAIN_D = 0.0;

//============================================================================
// variable setup
//============================================================================

int param_p{};
int param_i{};
int param_d{};
int param_sp{};
int meas_fb{};

struct aINS {

  int fb{};
  int sp{};
  int p{};
  int i{};
  int d{};

};

struct dINS {

  bool pw{};
  bool inv{};
  bool mana{};
  bool manb{};

};

struct params {

  double p{};
  double i{};
  double d{};

};

// make struct for analog ins
aINS AINS;

// init struct for digital ins
dINS DINS;

// init struct for scaled values
params par;


//============================================================================
// Input reading 
void readAInputs(aINS* AINS ) {
//============================================================================

  AINS->fb = analogRead(PIN_AIN_FB);
  
  // invert setpoint
  if (DINS.inv) {

    AINS->sp = 1024 - analogRead(PIN_AIN_SP);

  } else {

    AINS->sp = analogRead(PIN_AIN_SP);
  
  }

  AINS->p = analogRead(PIN_AIN_SP_P);
  AINS->i = analogRead(PIN_AIN_SP_I);
  AINS->d = analogRead(PIN_AIN_SP_D);

}

//============================================================================
// Input Reading
void readDInputs(dINS* DINS) {
//============================================================================


  DINS->pw = digitalRead(PIN_DIN_PW);
  DINS->inv = digitalRead(PIN_DIN_INV);
  DINS->mana = digitalRead(PIN_DIN_MANA);
  DINS->manb = digitalRead(PIN_DIN_MANB);

}

//============================================================================
// map analog ins to floats for usage in contorller math
void scaleInputs(aINS* AINS, params* par) {
//============================================================================

  par->p = PARAM_MINGAIN_P + (static_cast<double>(AINS->p) / 1024.0) * PARAM_MAXGAIN_P;
  par->i = PARAM_MINGAIN_I + (static_cast<double>(AINS->i) / 1024.0) * PARAM_MAXGAIN_I;
  par->d = PARAM_MINGAIN_D + (static_cast<double>(AINS->d) / 1024.0) * PARAM_MAXGAIN_D;

}

//============================================================================
// Check for polarization of two numbers, if both have the same sign -> true 
//============================================================================

bool checkSign(double x, double y) {

  return !((x > 0) ^ (y > 0));

}


//============================================================================
// PID Controller service routine
int servController(aINS* AINS, params* par){
//============================================================================
  

  static double itgr_error{};
  static double last_error{};
  static unsigned long millis_last{};

  // calculate the distance from FB to SP 
  double error = AINS->sp - AINS->fb;


  // only act if error threshold is reached
  // this is needed to keep the controller constantly fighting against backlash 
  if (static_cast<int>(abs(error)) > ERROR_THRESH) {

  
    // PROPORTIONAL error
    double p_err = par->p * error;

    // INTEGRATED error
    itgr_error += error;
    double i_err = par->i * itgr_error;

    // DERIVATIVE ERROR 
    double timeDiff = static_cast<double>(millis() - millis_last);
    
    // calculate and scale current rate of change
    double d_err = par->d * (error - last_error) / timeDiff;
    
    // make sure that derivative error is always opposing integrated error
    // needs to be done to maintain dampening function
    // if error changes very fast, both i_err and d_err will have the same direction(thats bad) 
    if (checkSign(i_err, d_err)) {

      d_err = 0;

    }

    // update timestamp 
    millis_last = millis();


    // Serial.print("D_ERR: ");
    // Serial.print(d_err);
    // Serial.print("I ERR: ");
    // Serial.println(i_err);

    // routine over: overwrite the last error with the current one
    last_error = error;
    
    // sum all the errors together
    int corrector = static_cast<int>((p_err + i_err + d_err) * MASTER_GAIN); 
    
    return corrector;

  } else {
    // if error threshold is not reached:
    // drain integral to avoid small errors accumulating in there
    itgr_error = 0;

    // return 0 to to nothing
    return 0;

  }
}



//============================================================================
// Motor driver
void driveMotor(int sp) {
//============================================================================

  //TODO
  // richtung setzen

  if (sp < 0) {

    digitalWrite(PIN_MOTOR_A, 1);
    digitalWrite(PIN_MOTOR_B, 0);
    
  } else {

    digitalWrite(PIN_MOTOR_A, 0);
    digitalWrite(PIN_MOTOR_B, 1);
    
  }

  // vorzeichen entfernen

  sp = abs(sp);
  
  // setpoint skalieren
  int sp_scaled = map(sp, 0, 1024, 0, 255);
   
  // Analog out schreiben

  analogWrite(PIN_MOTOR_ENA, sp_scaled);

}

void setup() {
  
  // setup comms 
  Serial.begin(115200);

  // give out SW information
  Serial.println(FW_STRING);

  // setup output pins 
  pinMode(PIN_MOTOR_ENA, OUTPUT);
  pinMode(PIN_MOTOR_A, OUTPUT);
  pinMode(PIN_MOTOR_B, OUTPUT);

  // setup input pins
  for (int i = 4 ; i < 19; i++) {

    pinMode(i, INPUT);

    // jump to second block of pins
    i = i == 7 ? 14 : i;

  } 

  
}

void loop() {

  static long millis_last{};

  // INPUT

  readAInputs(&AINS);
  readDInputs(&DINS);

  // PROCESS

  scaleInputs(&AINS, &par);
  
  // OUTPUT

  if (digitalRead(PIN_DIN_PW)) {

    // service controller routine and drive motor
    driveMotor(servController(&AINS, &par));

    // print controller output for debugging purposes
    #ifdef DEBUG

    if (millis() - millis_last > DEBUG_SPEED) {
    
      Serial.println("==============================================");
      Serial.println();
      Serial.print("CONTROLLER OUTPUT: ");
      Serial.print(servController(&AINS, &par));
      Serial.print(" ABSOLUTE ERROR: ");
      Serial.println(AINS.sp - AINS.fb);
      Serial.println();
      
    }

    #endif

  } else {
    driveMotor(0);
  }
  
  // Print struct values for debug purposes
  #ifdef DEBUG

  if (millis() - millis_last > DEBUG_SPEED) {

    // Print Analog input values
    Serial.println("==============================================");      
    Serial.print("AINS - FB:  ");
    Serial.print(AINS.fb);
    Serial.print(" SP: ");
    Serial.print(AINS.sp);
    Serial.print(" P: ");
    Serial.print(AINS.p);
    Serial.print(" I: ");
    Serial.print(AINS.i);
    Serial.print(" D: ");
    Serial.print(AINS.d);
    Serial.println();

    

    // Print Calculated gains

    Serial.print("GAINS - P:  ");
    Serial.print(par.p, DISP_ACCURACY);
    Serial.print(" I: ");
    Serial.print(par.i, DISP_ACCURACY);
    Serial.print(" D: ");
    Serial.print(par.d, DISP_ACCURACY);
    Serial.println();



    millis_last = millis();
  } 
  
  #endif

  
  // slow down loop
  delay(LOOP_SPEED_MS);



}
