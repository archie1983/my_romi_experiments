#include "pin_names_and_constants.h"

/**
 * This class will control motor movement.
 */
class Motor {
  public:
    /**
     * Runs the motor forward for 1 second at half PWM power.
     */
    void goForward_1Second() {
      turnMotor(127);
      delay(1000);
      stopMotor();
    }

    static Motor* getRightMotor() {
      return rightMotor;
    }

    static Motor* getLeftMotor() {
      return leftMotor;
    }

    /**
     * Increases the encoder pulse count. I'd really like this to be a private function,
     * but we can't have that if we want to call it from the ISR.
     */
    void incEncPulseCnt() {
      encoder_pulse_cnt++;
    }

    /**
     * Decreases the encoder pulse count. I'd really like this to be a private function,
     * but we can't have that if we want to call it from the ISR.
     */
    void decEncPulseCnt() {
      encoder_pulse_cnt--;
    }
  
  private:
    /**
     * Constructor with a set of pins- directions and run control for the motor.
     * It doesn't have to be public, because we'll only create instances of motor
     * within motor.h and those instances will be available through a static function.
     * 
     * We also pass a function pointer to set up the encoder.
     */
    Motor(byte pinDirection, byte pinRun, void (*encoder_setup) (void)) {
      this->pinDirection = pinDirection;
      this->pinRun = pinRun;

      pinMode(pinDirection, OUTPUT);
      pinMode(pinRun, OUTPUT);

      encoder_setup();
    }
    
    /**
     * References of the left motor and the right motor. We'll initialise them too within motor.h
     * and they will be available through a public static function.
     */
    static Motor* leftMotor;
    static Motor* rightMotor;
    /**
     * direction pin
     */
    byte pinDirection;

    /**
     * Motor run pin
     */
    byte pinRun;

    /**
     * Pulse count of the encoder that is connected to this motor.
     */
    volatile int encoder_pulse_cnt = 0;

    /**
     * Moves the motor to go forward or backward with a given PWM.
     * 
     * @power Positive value means: go forward, negative: go back. The absolute
     * value will be used as the PWM - passed directly to analogWrite().
     * 
     * NOTE: Deadband seems to be 0..8 both forwards and backwards for both motors. At power = 9
     * it just barely moves, but not with lower power. 
     * 
     * Going forward, left motor seems to be able to move at power = 8, but only sometimes, so 
     * I guess it has a little bit narrower deadband (but only if running in air,
     * not on ground). Going backwards, both motors barely move at power -8 in the air, but not on ground.
     * 
     */
    void turnMotor(int power) {
      if (power > 0) {
        digitalWrite(pinDirection, HIGH);
      } else {
        digitalWrite(pinDirection, LOW);
      }
  
      /**
       * There's no point to turn the motors at all if the the power is less than 9 as
       * that is within deadband.
       */
      if (abs(power) > 8) {
        /**
         * If we don't take abs value here, then it looks like values greater than 255 overflow
         * and negative values are treated as unsigned byte, rather than a negative value, 
         * so -1 will come out as 255. Looks like analogWrite function takes in a paramater of unsigned byte.
         */
        analogWrite(pinRun, abs(power));
      }
    }

    /**
     * Stops the motor.
     */
    void stopMotor() {
      analogWrite(pinRun, 0);
      digitalWrite(pinDirection, LOW);
    }
};

/**
 * Below are interrupt routines, setup procedures and variables for the encoders:
 */
void setupRightEncoder() {
  // Setup pins for right encoder
  pinMode( RIGHT_ENCODER_XOR, INPUT );
  pinMode( RIGHT_ENCODER_PHASE_B, INPUT );

  // Now to set up PE6 as an external interupt (INT6), which means it can
  // have its own dedicated ISR vector INT6_vector

  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit low, preserve other bits
  EIMSK = EIMSK & ~(1<<INT6);
  //EIMSK = EIMSK & B1011111; // Same as above.

  // Page 89, 11.1.2 External Interrupt Control Register B – EICRB
  // Used to set up INT6 interrupt
  //EICRB |= (1 << ISC60) | (1 << ISC61);  // using header file names, push 1 to bit ISC60 and ISC61 to get triggered on rising edge only
  EICRB |= (1 << ISC60);  // using header file names, push 1 to bit ISC60 to get triggered on both rising and falling edges.
  //EICRB |= B00010000; // does same as above

  // Page 90, 11.1.4 External Interrupt Flag Register – EIFR
  // Setting a 1 in bit 6 (INTF6) clears the interrupt flag.
  EIFR |= ( 1 << INTF6 );
  //EIFR |= B01000000;  // same as above

  // Now that we have set INT6 interrupt up, we can enable
  // the interrupt to happen
  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit high, preserve other bits
  EIMSK |= ( 1 << INT6 );
  //EIMSK |= B01000000; // Same as above
}

void setupLeftEncoder() {
    // Setting up LEFT_ENCODER_XOR:
    // The Romi board uses the pin PE2 (port E, pin 2) which is
    // very unconventional.  It doesn't have a standard
    // arduino alias (like d6, or a5, for example).
    // We set it up here with direct register access
    // Writing a 0 to a DDR sets as input
    // DDRE = Data Direction Register (Port)E
    // We want pin PE2, which means bit 2 (counting from 0)
    // PE Register bits [ 7  6  5  4  3  2  1  0 ]
    // Binary mask      [ 1  1  1  1  1  0  1  1 ]
    //    
    // By performing an & here, the 0 sets low, all 1's preserve
    // any previous state.
    DDRE = DDRE & ~(1<<DDE6);
    //DDRE = DDRE & B11111011; // Same as above. 

    // We need to enable the pull up resistor for the pin
    // To do this, once a pin is set to input (as above)
    // You write a 1 to the bit in the output register
    PORTE = PORTE | (1<< PORTE2 );
    //PORTE = PORTE | 0B00000100;

    // Left encoder uses conventional pin 26
    pinMode(LEFT_ENCODER_XOR, INPUT);
    digitalWrite(LEFT_ENCODER_XOR, HIGH); // Encoder 0 xor

    // Enable pin-change interrupt on A8 (PB4) for left encoder, and disable other
    // pin-change interrupts.
    // Note, this register will normally create an interrupt a change to any pins
    // on the port, but we use PCMSK0 to set it only for PCINT4 which is A8 (PB4)
    // When we set these registers, the compiler will now look for a routine called
    // ISR( PCINT0_vect ) when it detects a change on the pin.  PCINT0 seems like a
    // mismatch to PCINT4, however there is only the one vector servicing a change
    // to all PCINT0->7 pins.
    // See Manual 11.1.5 Pin Change Interrupt Control Register - PCICR

    // Page 91, 11.1.5, Pin Change Interrupt Control Register 
    // Disable interrupt first
    PCICR = PCICR & ~( 1 << PCIE0 );
    // PCICR &= B11111110;  // Same as above

    // 11.1.7 Pin Change Mask Register 0 – PCMSK0
    PCMSK0 |= (1 << PCINT4);

    // Page 91, 11.1.6 Pin Change Interrupt Flag Register – PCIFR
    PCIFR |= (1 << PCIF0);  // Clear its interrupt flag by writing a 1.

    // Enable
    PCICR |= (1 << PCIE0);
}

/**
 * Instantiating our motors.
 */
Motor* Motor::rightMotor = new Motor(RIGHT_MOTOR_DIR, RIGHT_MOTOR_RUN, setupRightEncoder);
Motor* Motor::leftMotor = new Motor(LEFT_MOTOR_DIR, LEFT_MOTOR_RUN, setupLeftEncoder);
 
 /**
 * To avoid reading the pins twice- for each comparison, we'll read them once and store in these variables.
 * Also we'll need to infer phaseA state and keep its previous value to be able to count steps.
 */
volatile boolean right_encoder_xored_signal = false;
volatile boolean right_encoder_phase_b_signal = false;
volatile boolean right_encoder_phase_b_signal_prev = false;
volatile boolean right_encoder_phase_a_signal = false; //# we'll be inferring this 
volatile boolean right_encoder_phase_a_signal_prev = false; //# we'll be inferring this 

/**
 * Ditto for left encoder.
 */
volatile boolean left_encoder_xored_signal = false;
volatile boolean left_encoder_phase_b_signal = false;
volatile boolean left_encoder_phase_b_signal_prev = false;
volatile boolean left_encoder_phase_a_signal = false; //# we'll be inferring this 
volatile boolean left_encoder_phase_a_signal_prev = false; //# we'll be inferring this 

/**
 * Right wheel encoder pin phaseA and phaseB XOR-ed. We can use that as a detection for wheel position change.
 * We'll need to read one of the original phaseB or phaseB pins with the digitalRead() function to decide whether
 * the movement was forward or backward.
 */
ISR( INT6_vect ) {
  /**
   * From the lecture notes:
   * If:
   * 
   * Channel A goes from LOW to HIGH
   * and Channel B is LOW
   * You are going CLOCKWISE!
   * Or if:
   * 
   * Channel A goes from LOW to HIGH
   * and Channel B is HIGH
   * You are going COUNTER-CLOCKWISE!
   * 
   * But because instead of phaseA signal, we have an XOR-ed signal of phaseA and phaseB,
   * we have to infer the phaseA signal from the info that we get.
   */

  /*
   * Reading the actual current signals
   */
  right_encoder_phase_b_signal = digitalRead(RIGHT_ENCODER_PHASE_B);
  right_encoder_xored_signal = digitalRead(RIGHT_ENCODER_XOR);

  /*
   * Inferring the phase A current state from the two signals above.
   */
  right_encoder_phase_a_signal = right_encoder_phase_b_signal ^ right_encoder_xored_signal;

  /**
   * If we've seen a transition of phase A from 0 to 1
   * AND phase B==0, then we're going CW
   */
  if (!right_encoder_phase_a_signal_prev && right_encoder_phase_a_signal && !right_encoder_phase_b_signal) {
    Motor::getRightMotor()->incEncPulseCnt(); //# going CW
  } else if (!right_encoder_phase_a_signal_prev && right_encoder_phase_a_signal && right_encoder_phase_b_signal) {
    Motor::getRightMotor()->decEncPulseCnt(); //# going CCW
  }
  right_encoder_phase_a_signal_prev = right_encoder_phase_a_signal;
}

/**
 * Left wheel encoder pin phaseA and phaseB XOR-ed. We can use that as a detection for wheel position change.
 * We'll need to read one of the original phaseB or phaseB pins with the digitalRead() function to decide whether
 * the movement was forward or backward.
 */
ISR( PCINT0_vect ) {
  /*
   * Reading the actual current signals
   */
  left_encoder_xored_signal = digitalRead(LEFT_ENCODER_XOR);

  // Now the phaseB signal.
  // Mask for a specific pin from the port.
  // Non-standard pin, so we access the register
  // directly.  
  // Reading just PINE would give us a number
  // composed of all 8 bits.  We want only bit 2.
  // B00000100 masks out all but bit 2
  left_encoder_phase_b_signal = PINE & (1<<PINE2);
  //boolean newE0_B = PINE & B00000100;  // Does same as above.

  /*
   * Inferring the phase A current state from the two signals above.
   */
  left_encoder_phase_a_signal = left_encoder_phase_b_signal ^ left_encoder_xored_signal;

  /**
   * If we've seen a transition of phase A from 0 to 1
   * AND phase B==0, then we're going CW
   */
  if (!left_encoder_phase_a_signal_prev && left_encoder_phase_a_signal && !left_encoder_phase_b_signal) {
    Motor::getLeftMotor()->incEncPulseCnt(); //# going CW
  } else if (!left_encoder_phase_a_signal_prev && left_encoder_phase_a_signal && left_encoder_phase_b_signal) {
    Motor::getLeftMotor()->decEncPulseCnt(); //# going CCW
  }
  left_encoder_phase_a_signal_prev = left_encoder_phase_a_signal;
}
