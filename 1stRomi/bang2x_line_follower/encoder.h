#ifndef _ENCODER_
#define _ENCODER_

#include "pin_names_and_constants.h"
#include "threshold_callback.h"

class Encoder {
  public:
    /**
     * Static function to access the left encoder
     */
    static Encoder* getLeftEncoder() {
      return leftEncoder;
    }

    /**
     * Static function to access the right encoder
     */
    static Encoder* getRightEncoder() {
      return rightEncoder;
    }

    /**
     * Increases the encoder pulse count. I'd really like this to be a private function,
     * but we can't have that if we want to call it from the ISR.
     */
    void incEncPulseCnt() {
      encoder_pulse_cnt++;

      calculate_wheel_speed(1);

      /*
       * If we have a running threshold, then let's update its counter and act on it if it's
       * the time.
       */
      if (threshold_triggered_functionality != NULL && threshold_triggered_functionality->isThresholdActive()) {
        /*
         * Operate the threshold
         */
        threshold_triggered_functionality->decreaseCounter();

        /*
         * If the threshold has done its job and is no longer active, then let's remove it.
         */
        if (!threshold_triggered_functionality->isThresholdActive()) {
          threshold_triggered_functionality = NULL;
        }
      }
    }

    /**
     * Decreases the encoder pulse count. I'd really like this to be a private function,
     * but we can't have that if we want to call it from the ISR.
     */
    void decEncPulseCnt() {
      encoder_pulse_cnt--;

      calculate_wheel_speed(-1);

      /*
       * If we have a running threshold, then let's update its counter and act on it if it's
       * the time.
       */
      if (threshold_triggered_functionality != NULL && threshold_triggered_functionality->isThresholdActive()) {
        /*
         * Operate the threshold
         */
        threshold_triggered_functionality->increaseCounter();

        /*
         * If the threshold has done its job and is no longer active, then let's remove it.
         */
        if (!threshold_triggered_functionality->isThresholdActive()) {
          threshold_triggered_functionality = NULL;
        }
      }
    }

    /**
     * Returns our current pulse count for this encoder.
     */
    long getPulseCount() {
      return encoder_pulse_cnt;
    }

    /**
     * Returns the speed of the wheel.
     * 
     * Gives roughly the following values for given motor powers without PID:
     * 
     * power | clicks per second
     * --------------------------
     *  25   |      77 - 90
     *  100  |     280 - 300   
     *  200  |     590 - 636
     *  255  |     800 - 875
     */
    long getWheelSpeed() {
      return wheel_speed;
    }

    /**
     * Sets a threshold of the given counts and the function that needs to be run
     * after the threshold has been reached.
     */
    void setThreshold(ThresholdCallback *threshold_triggered_functionality) {
      this->threshold_triggered_functionality = threshold_triggered_functionality;
    }
    
  private:
    /**
     * Constructor is private because we will only invoke it within encoder.h and have
     * the relevant instances returnable from static functions.
     * 
     * We pass a function pointer to set up the encoder, because left and right encoder will have different
     * setup functions.
     */
    Encoder(void (*encoder_setup) (void)) {
      encoder_setup();
      encoder_pulse_cnt = 0;
    }

    /**
     * Pulse count of the encoder that is connected to this motor.
     * volatile because it's operated on in the functions that are only called from ISR.
     */
    volatile long encoder_pulse_cnt = 0;

    /**
     * We'll need this timestamp (coming from micros()) to determine speed
     * when pulse count is changing.
     * volatile because it's operated on in the functions that are only called from ISR.
     */
    volatile unsigned long timestamp_of_last_pulse = 0;

    /**
     * Speed of the wheel at the time of last encoder pulse. This is in terms of pulses 
     * per second. If rad/s or m/s are required, then a separate function needs to be
     * created.
     * volatile because it's operated on in the functions that are only called from ISR.
     */
    volatile long wheel_speed = 0;

    /**
     * References of the left encoder and the right encoder. We'll initialise them too within encoder.h
     * and they will be available through a public static function.
     */
    static Encoder* leftEncoder;
    static Encoder* rightEncoder;

    /**
     * A pointer to the motro control so that we can do something with the motor when the threshold has been reached.
     */
    ThresholdCallback *threshold_triggered_functionality = NULL;
    
    /**
     * Calculates line speed and multiplies it by the passed coefficient before storing.
     * The coefficient is typically either 1 for forward motion or -1 for backward motion,
     * but other values can be used depending on usecase.
     */
    void calculate_wheel_speed(int coefficient) {
      /*
       * Claculating the speed of this wheel
       */
       unsigned long current_timestamp = micros();
       
       /*
        * So 1 million microseconds (for 1 second) divided by the number of microseconds 
        * between the last two pulses gives us number of pulses in second.
        */
       wheel_speed = ((long)coefficient) * USECONDS_IN_1_SECOND / (long)(current_timestamp - timestamp_of_last_pulse);
       timestamp_of_last_pulse = current_timestamp;
    }
};

/**
 * Declaring function prototypes just so that I can instantiate encoders around here and not at the end
 * of the file.
 */
void setupRightEncoder();
void setupLeftEncoder();

/**
 * Instantiating our encoders.
 */
Encoder* Encoder::rightEncoder = new Encoder(setupRightEncoder);
Encoder* Encoder::leftEncoder = new Encoder(setupLeftEncoder);

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
    Encoder::getRightEncoder()->decEncPulseCnt(); //# going CW - a.k.a backwards
  } else if (!right_encoder_phase_a_signal_prev && right_encoder_phase_a_signal && right_encoder_phase_b_signal) {
    Encoder::getRightEncoder()->incEncPulseCnt(); //# going CCW - a.k.a forwards
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
    Encoder::getLeftEncoder()->decEncPulseCnt(); //# going CW - a.k.a backwards
  } else if (!left_encoder_phase_a_signal_prev && left_encoder_phase_a_signal && left_encoder_phase_b_signal) {
    Encoder::getLeftEncoder()->incEncPulseCnt(); //# going CCW - a.k.a forwards
  }
  left_encoder_phase_a_signal_prev = left_encoder_phase_a_signal;
}

#endif
