

#define DEBUG

// Pinout configs 

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

// Config Parameters for calculations

const double PARAM_MAXGAIN_P = 2.0;
const double PARAM_MINGAIN_P = 0.5;

const double PARAM_MAXGAIN_I = 0.05;
const double PARAM_MINGAIN_I = 0.0;

const double PARAM_MAXGAIN_D = 5.0;
const double PARAM_MINGAIN_D = 0.0;




// variable setup

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
// Eingaberoutine
void readAInputs(aINS* AINS ) {
//============================================================================

  AINS->fb = analogRead(PIN_AIN_FB);
  AINS->sp = analogRead(PIN_AIN_SP);
  AINS->p = analogRead(PIN_AIN_SP_P);
  AINS->i = analogRead(PIN_AIN_SP_I);
  AINS->d = analogRead(PIN_AIN_SP_D);

}

//============================================================================
// Eingaberoutine
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
// PID Controller service routine
int servController(aINS* AINS, params* par){
//============================================================================
  

  static double itgr_error{};
  static double last_error{};
  static unsigned long micros_last{};

  // calculate the distance from FB to SP 
  double error = AINS->sp - AINS->fb;

  // proportional error
  double p_err = par->p * error;

  // integrated error
  itgr_error += error;
  double i_err = par->i * itgr_error;

  // derivative error 
  
  int timeDiff = static_cast<int>(micros() - micros_last);
  double d_err = par->d * (last_error - error) / timeDiff; 
  micros_last = micros();
  

  // routine over: overwrite the last error with the current one
  last_error = error;
  
  int corrector = static_cast<int>(p_err + i_err + d_err); 

  return corrector;
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
  Serial.println("SOFTWARE VERSION 0.0.0");

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

  // EINGABE 

  readAInputs(&AINS);
  readDInputs(&DINS);

  // VERARBEITUNG

  scaleInputs(&AINS, &par);
  
  // AUSGABE

  if (digitalRead(PIN_DIN_PW)) {
    driveMotor(servController(&AINS, &par));
  } else {
    driveMotor(0);
  }

  
  static long millis_last{};

  delay(100);

  #ifdef DEBUG


  if (millis() - millis_last > 1000) {

    // Ains printen

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

    

    // Gains printen

    Serial.print("GAINS - P:  ");
    Serial.print(par.p);
    Serial.print(" I: ");
    Serial.print(par.i);
    Serial.print(" D: ");
    Serial.print(par.d);
    Serial.println();

    millis_last = millis();
  } 
  
  #endif




}
