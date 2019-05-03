
// Volatile Global variables used by Encoder ISR.
volatile long left_encoder_count; // used by encoder to count the rotation
volatile bool old_left_A;  // used by encoder to remember prior state of A
volatile bool old_left_B;  // used by encoder to remember prior state of B
volatile long last_count_left;
volatile float left_speed;

volatile long right_encoder_count; // used by encoder to count the rotation
volatile bool old_right_A;  // used by encoder to remember prior state of A
volatile bool old_right_B;  // used by encoder to remember prior state of B
volatile long last_count_right;
volatile float right_speed;







// extern tells this class that these
// are declared as globals else where
extern bool   use_speed_controller;
extern float left_speed_demand;
extern float right_speed_demand;
extern PID    LeftSpeedControl;
extern PID    RightSpeedControl;
extern Motor  LeftMotor;
extern Motor  RightMotor;


// This ISR handles just Encoder 1
// ISR to read the Encoder1 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.
ISR( INT6_vect ) 
{
  // First, Read in the new state of the encoder pins.
    // Standard pins, so standard read functions.
    boolean new_e_right_B = digitalRead( ENCODER_RIGHT_B_PIN );
    boolean new_e_right_A = digitalRead( ENCODER_RIGHT_A_PIN );

    // Some clever electronics combines the
    // signals and this XOR restores the
    // true value.
    new_e_right_A ^= new_e_right_B;

    // Create a bitwise representation of our states
    // We do this by shifting the boolean value up by
    // the appropriate number of bits, as per our table
    // header:
    //
    // State :  (bit3)  (bit2)  (bit1)  (bit0)
    // State :  New A,  New B,  Old A,  Old B.
    byte state = 0;
    state = state | ( new_e_right_A  << 3 );
    state = state | ( new_e_right_B  << 2 );
    state = state | ( old_right_A  << 1 );
    state = state | ( old_right_B  << 0 );


    // This is an inefficient way of determining
    // the direction.  However it illustrates well
    // against the lecture slides.
    switch( state ) 
    {
        case 0:     break; // No movement.
        case 1:     right_encoder_count--; break;  // clockwise?
        case 2:     right_encoder_count++; break;  // anti-clockwise?
        case 3:     break;  // Invalid
        case 4:     right_encoder_count++; break; // anti-clockwise?
        case 5:     break;  // No movement.
        case 6:     break;  // Invalid
        case 7:     right_encoder_count--; break;  // clockwise?
        case 8:     right_encoder_count--; break; // clockwise?
        case 9:     break;  // Invalid
        case 10:    break;  // No movement.
        case 11:    right_encoder_count++; break; // anti-clockwise?
        case 12:    break;  // Invalid
        case 13:    right_encoder_count++; break; // anti-clockwise?
        case 14:    right_encoder_count--; break;  // clockwise?
        case 15:    break;  // No movement.
    }

    // Save current state as old state for next call.
    old_right_A = new_e_right_A;
    old_right_B = new_e_right_B;

}


// This ISR handles just Encoder 0
// ISR to read the Encoder0 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.


ISR( PCINT0_vect ) {
 
    // First, Read in the new state of the encoder pins.

    // Mask for a specific pin from the port.
    // Non-standard pin, so we access the register
    // directly.  
    // Reading just PINE would give us a number
    // composed of all 8 bits.  We want only bit 2.
    // B00000100 masks out all but bit 2
    boolean new_e_left_B = PINE & (1<<PINE2);
    //boolean new_e_right_B = PINE & B00000100;  // Does same as above.

    // Standard read fro the other pin.
    boolean new_e_left_A = digitalRead( ENCODER_LEFT_A_PIN ); // 26 the same as A8

    // Some clever electronics combines the
    // signals and this XOR restores the 
    // true value.
    new_e_left_A ^= new_e_left_B;

    // Create a bitwise representation of our states
    // We do this by shifting the boolean value up by
    // the appropriate number of bits, as per our table
    // header:
    //
    // State :  (bit3)  (bit2)  (bit1)  (bit0)
    // State :  New A,  New B,  Old A,  Old B.
    byte state = 0;                   
    state = state | ( new_e_left_A  << 3 );
    state = state | ( new_e_left_B  << 2 );
    state = state | ( old_left_A  << 1 );
    state = state | ( old_left_B  << 0 );

    // This is an inefficient way of determining
    // the direction.  However it illustrates well
    // against the lecture slides.  
    switch ( state ) 
    {
        case 0:    break; // No movement.
        case 1:    left_encoder_count--; break;  // clockwise?
        case 2:    left_encoder_count++; break;  // anti-clockwise?
        case 3:    break;  // Invalid
        case 4:    left_encoder_count++; break;  // anti-clockwise?
        case 5:    break;  // No movement.
        case 6:    break;  // Invalid
        case 7:    left_encoder_count--; break;// clockwise?
        case 8:    left_encoder_count--; break;  // clockwise?
        case 9:    break;  // Invalid
        case 10:   break;  // No movement.
        case 11:   left_encoder_count++; break;// anti-clockwise?
        case 12:   break;  // Invalid
        case 13:   left_encoder_count++; break;// anti-clockwise?
        case 14:   left_encoder_count--;  break;// clockwise?
        case 15:   break;  // No movement.
    }
  
     
    // Save current state as old state for next call.
    old_left_A = new_e_left_A;
    old_left_B = new_e_left_B; 
}


/*
   This setup routine enables interrupts for
   encoder1.  The interrupt is automatically
   triggered when one of the encoder pin changes.
   This is really convenient!  It means we don't
   have to check the encoder manually.
*/
void setupRightEncoder() 
{

    // Initialise our count value to 0.
    right_encoder_count = 0;
    last_count_right = 0;
    right_speed = 0;

    // Initialise the prior A & B signals
    // to zero, we don't know what they were.
    old_right_A = 0;
    old_right_B = 0;

    // Setup pins for right encoder 
    pinMode( ENCODER_RIGHT_A_PIN, INPUT );
    pinMode( ENCODER_RIGHT_B_PIN, INPUT );

    // Now to set up PE6 as an external interupt (INT6), which means it can
    // have its own dedicated ISR vector INT6_vector

    // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
    // Disable external interrupts for INT6 first
    // Set INT6 bit low, preserve other bits
    EIMSK = EIMSK & ~(1<<INT6);
    //EIMSK = EIMSK & B1011111; // Same as above.
  
    // Page 89, 11.1.2 External Interrupt Control Register B – EICRB
    // Used to set up INT6 interrupt
    EICRB |= ( 1 << ISC60 );  // using header file names, push 1 to bit ISC60
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

void setupLeftEncoder() 
{

    // Initialise our count value to 0.
    left_encoder_count = 0;
    last_count_left = 0;
    left_speed = 0;
    // Initialise the prior A & B signals
    // to zero, we don't know what they were.
    old_left_A = 0;
    old_left_B = 0;

    // Setting up left encoder:
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
    PORTE = PORTE | (1 << PORTE2 );
    //PORTE = PORTE | 0B00000100;

    // Encoder0 uses conventional pin 26
    pinMode( ENCODER_LEFT_A_PIN, INPUT );
    digitalWrite( ENCODER_LEFT_A_PIN, HIGH ); // Encoder 0 xor

    // Enable pin-change interrupt on A8 (PB4) for encoder0, and disable other
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

//This starts a 100Hz timer interrupt.
void startTimer()
{
    // Clear Timer 3 registers
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3 = 0;

    // 100 Hz (16000000/((624+1)*256))
    OCR3A = 624;
    // CTC
    TCCR3B |= (1 << WGM32);
    // Prescaler 256
    TCCR3B |= (1 << CS32);
    // Output Compare Match A Interrupt Enable
    TIMSK3 |= (1 << OCIE3A);
}

ISR(TIMER3_COMPA_vect)
{

    /*
     * Calculate Speeds
     */
    signed int left_delta = left_encoder_count - last_count_left;
    signed int right_delta = right_encoder_count - last_count_right;

    last_count_left = left_encoder_count;
    last_count_right = right_encoder_count;

    left_speed =  left_delta;
    right_speed = right_delta;

    if (use_speed_controller)
    {
        float left_motor_demand = LeftSpeedControl.update(left_speed_demand, left_speed);
        float right_motor_demand = RightSpeedControl.update(right_speed_demand, right_speed);
      
        LeftMotor.setPower(left_motor_demand);
        RightMotor.setPower(right_motor_demand);
    }
}

