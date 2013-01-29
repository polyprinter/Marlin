/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"
#include "speed_lookuptable.h"



//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced


//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it impossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
static long counter_x,       // Counter variables for the Bresenham line tracer
            counter_y, 
            counter_z,       
            counter_e;
volatile static unsigned long step_events_completed; // The number of step events executed in the current block
#ifdef EXTRUDER_ADVANCE
  static long advance_rate, unadvance_rate, advance, final_advance = 0;
  static long old_advance = 0;
#endif
static long e_steps[3];
static long acceleration_time, deceleration_time;
//static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for deceleration start point
static char step_loops;
static unsigned short OCR1A_nominal;

volatile long endstops_trigsteps[3]={0,0,0};
volatile long endstops_stepsTotal,endstops_stepsDone;
static volatile bool endstop_x_hit=false;
static volatile bool endstop_y_hit=false;
static volatile bool endstop_z_hit=false;

static bool old_x_min_endstop=false;
static bool old_x_max_endstop=false;
static bool old_y_min_endstop=false;
static bool old_y_max_endstop=false;
static bool old_z_min_endstop=false;
static bool old_z_max_endstop=false;

static bool check_endstops = true;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};
volatile char count_direction[NUM_AXIS] = { 1, 1, 1, 1};

//===========================================================================
//=============================functions         ============================
//===========================================================================

  #define CHECK_ENDSTOPS  if(check_endstops)

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %A1, %A2 \n\t" \
"add %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r0 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (charIn1), \
"d" (intIn2) \
: \
"r26" \
)

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 48bit result
#define MultiU24X24toH16(intRes, longIn1, longIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"mov r27, r1 \n\t" \
"mul %B1, %C2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %C1, %C2 \n\t" \
"add %B0, r0 \n\t" \
"mul %C1, %B2 \n\t" \
"add %A0, r0 \n\t" \
"adc %B0, r1 \n\t" \
"mul %A1, %C2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %B2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %C1, %A2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %A2 \n\t" \
"add r27, r1 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r27 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (longIn1), \
"d" (longIn2) \
: \
"r26" , "r27" \
)

// Some useful constants

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)


void checkHitEndstops()
{
 if( endstop_x_hit || endstop_y_hit || endstop_z_hit) {
   SERIAL_ECHO_START;
   SERIAL_ECHOPGM(MSG_ENDSTOPS_HIT);
   if(endstop_x_hit) {
     SERIAL_ECHOPAIR(" X:",(float)endstops_trigsteps[X_AXIS]/axis_steps_per_unit[X_AXIS]);
   }
   if(endstop_y_hit) {
     SERIAL_ECHOPAIR(" Y:",(float)endstops_trigsteps[Y_AXIS]/axis_steps_per_unit[Y_AXIS]);
   }
   if(endstop_z_hit) {
     SERIAL_ECHOPAIR(" Z:",(float)endstops_trigsteps[Z_AXIS]/axis_steps_per_unit[Z_AXIS]);
   }
   SERIAL_ECHOLN("");
   endstop_x_hit=false;
   endstop_y_hit=false;
   endstop_z_hit=false;
 }
}

void endstops_hit_on_purpose()
{
  endstop_x_hit=false;
  endstop_y_hit=false;
  endstop_z_hit=false;
}

void enable_endstops(bool check)
{
  check_endstops = check;
}

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates 
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.

void st_wake_up() {
  //  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();  
}

void step_wait(){
    for(int8_t i=0; i < 6; i++){
    }
}

const int16_t MAX_INTERRUPT_RATE_HZ = 10000;
//const int16_t MAX_INTERRUPT_RATE_HZ = 16000;   // testing showed a 24Khz capability when only X and Y operating. Try this to see if it helps.
#define TIMER_RATE_HZ 2000000

const int16_t MIN_TIMER_COUNT = TIMER_RATE_HZ / MAX_INTERRUPT_RATE_HZ;

FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
  unsigned short timer;
  //if ( step_rate > MAX_STEP_FREQUENCY ) step_rate = MAX_STEP_FREQUENCY;
  
  if ( step_rate > 3*MAX_INTERRUPT_RATE_HZ ) { // If steprate > 30kHz >> step 4 times
    step_rate = (step_rate >> 2);
    step_loops = 4;
  }
  if ( step_rate > 2*MAX_INTERRUPT_RATE_HZ ) { // If steprate > 20kHz >> step 3 times
	  step_rate = step_rate / 3;
	  step_loops = 3;
	  }
  else if ( step_rate > MAX_INTERRUPT_RATE_HZ ) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1);
    step_loops = 2;
  }
  else {
    step_loops = 1;
  } 
  
  if ( step_rate < (F_CPU/500000) ) {
    step_rate = (F_CPU/500000);
  }
  else {
    // why? 
    step_rate -= (F_CPU/500000); // Correct for minimal speed
  }
  
  if(step_rate >= (8*256)){ // higher step rate 
    unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
    MultiU16X8toH16(timer, tmp_step_rate, gain);
    timer = (unsigned short)pgm_read_word_near(table_address) - timer;
  }
  else { // lower step rates
    unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate)>>1) & 0xfffc;
    timer = (unsigned short)pgm_read_word_near(table_address);
    timer -= (((unsigned short)pgm_read_word_near(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
  }

  if ( timer < (MIN_TIMER_COUNT-1)  ) {
    //(20kHz this should never happen)
     MYSERIAL.print( MSG_STEPPER_TO_HIGH ); 
    MYSERIAL.print( step_rate );  
    MYSERIAL.print( ' ' );  
    MYSERIAL.println( timer ); 
    timer = MIN_TIMER_COUNT; 
 }  

  return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
FORCE_INLINE void trapezoid_generator_reset() {
  #ifdef EXTRUDER_ADVANCE
    advance        = current_block->initial_advance;
	 advance_rate   = current_block->advance_rate;
	 unadvance_rate = current_block->unadvance_rate;
    final_advance  = current_block->final_advance;
    // Do E steps + advance steps
    e_steps[current_block->active_extruder] += ((advance >> 8) - old_advance);
    old_advance = advance >> 8;  
  #endif
  deceleration_time = 0;
  // step_rate to timer interval
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  OCR1A = acceleration_time;
  
//    SERIAL_ECHO_START;
//    SERIAL_ECHOPGM("advance :");
//    SERIAL_ECHO(current_block->advance/256.0);
//    SERIAL_ECHOPGM("advance rate :");
//    SERIAL_ECHO(current_block->advance_rate/256.0);
//    SERIAL_ECHOPGM("initial advance :");
//  SERIAL_ECHO(current_block->initial_advance/256.0);
//    SERIAL_ECHOPGM("final advance :");
//    SERIAL_ECHOLN(current_block->final_advance/256.0);
    
}

#ifdef EXTRUDER_ADVANCE
extern float extruder_advance_k;
#ifndef ADVANCE_HAS_OWN_INTERRUPT_SERVICE_ROUTINE
void HandleExtruderAdvance();
#endif
#endif

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
ISR(TIMER1_COMPA_vect)
{    
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      current_block->busy = true;
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      counter_e = counter_x;
      step_events_completed = 0; 
      
      #ifdef Z_LATE_ENABLE 
        if(current_block->steps_z > 0) {
          enable_z();
          OCR1A = 2000; //1ms wait
          return;
        }
      #endif
      
//      #ifdef EXTRUDER_ADVANCE
//      e_steps[current_block->active_extruder] = 0;
//      #endif
    } 
    else {
        OCR1A=2000; // 1kHz.
    }    
  } 

#ifdef DYNAMIC_ADVANCE_OPTION
  bool doAdvance = ( extruder_advance_k > 0  );
#endif

  if (current_block != NULL) {
    // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits;

    // Set direction en check limit switches
    if ((out_bits & (1<<X_AXIS)) != 0) {   // stepping along -X axis
      #if !defined COREXY  //NOT COREXY
      WRITE(X_DIR_PIN, INVERT_X_DIR);
      #endif
      count_direction[X_AXIS]=-1;
      CHECK_ENDSTOPS
      {
        #if X_MIN_PIN > -1
          bool x_min_endstop=(READ(X_MIN_PIN) != X_ENDSTOPS_INVERTING);
          if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
            endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
            endstop_x_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_x_min_endstop = x_min_endstop;
        #endif
      }
    }
    else { // +direction 
      #if !defined COREXY  //NOT COREXY
      WRITE(X_DIR_PIN,!INVERT_X_DIR);
      #endif
      
      count_direction[X_AXIS]=1;
      CHECK_ENDSTOPS 
      {
        #if X_MAX_PIN > -1
          bool x_max_endstop=(READ(X_MAX_PIN) != X_ENDSTOPS_INVERTING);
          if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0)){
            endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
            endstop_x_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_x_max_endstop = x_max_endstop;
        #endif
      }
    }

    if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
      #if !defined COREXY  //NOT COREXY
      WRITE(Y_DIR_PIN,INVERT_Y_DIR);
      #endif
      count_direction[Y_AXIS]=-1;
      CHECK_ENDSTOPS
      {
        #if Y_MIN_PIN > -1
          bool y_min_endstop=(READ(Y_MIN_PIN) != Y_ENDSTOPS_INVERTING);
          if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_y_min_endstop = y_min_endstop;
        #endif
      }
    }
    else { // +direction
      #if !defined COREXY  //NOT COREXY
    WRITE(Y_DIR_PIN,!INVERT_Y_DIR);
      #endif
      count_direction[Y_AXIS]=1;
      CHECK_ENDSTOPS
      {
        #if Y_MAX_PIN > -1
          bool y_max_endstop=(READ(Y_MAX_PIN) != Y_ENDSTOPS_INVERTING);
          if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)){
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_y_max_endstop = y_max_endstop;
        #endif
      }
    }

    
    #ifdef COREXY  //coreXY kinematics defined
      if((current_block->steps_x >= current_block->steps_y)&&((out_bits & (1<<X_AXIS)) == 0)){  //+X is major axis
        WRITE(X_DIR_PIN, !INVERT_X_DIR);
        WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
      }
      if((current_block->steps_x >= current_block->steps_y)&&((out_bits & (1<<X_AXIS)) != 0)){  //-X is major axis
        WRITE(X_DIR_PIN, INVERT_X_DIR);
        WRITE(Y_DIR_PIN, INVERT_Y_DIR);
      }      
      if((current_block->steps_y > current_block->steps_x)&&((out_bits & (1<<Y_AXIS)) == 0)){  //+Y is major axis
        WRITE(X_DIR_PIN, !INVERT_X_DIR);
        WRITE(Y_DIR_PIN, INVERT_Y_DIR);
      }        
      if((current_block->steps_y > current_block->steps_x)&&((out_bits & (1<<Y_AXIS)) != 0)){  //-Y is major axis
        WRITE(X_DIR_PIN, INVERT_X_DIR);
        WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
      }  
    #endif //coreXY
    
    
    if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
      WRITE(Z_DIR_PIN,INVERT_Z_DIR);
      
	  #ifdef Z_DUAL_STEPPER_DRIVERS
        WRITE(Z2_DIR_PIN,INVERT_Z_DIR);
      #endif
      
      count_direction[Z_AXIS]=-1;
      CHECK_ENDSTOPS
      {
        #if Z_MIN_PIN > -1
          bool z_min_endstop=(READ(Z_MIN_PIN) != Z_ENDSTOPS_INVERTING);
          if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_z_min_endstop = z_min_endstop;
        #endif
      }
    }
    else { // +direction
      WRITE(Z_DIR_PIN,!INVERT_Z_DIR);

	  #ifdef Z_DUAL_STEPPER_DRIVERS
        WRITE(Z2_DIR_PIN,!INVERT_Z_DIR);
      #endif

      count_direction[Z_AXIS]=1;
      CHECK_ENDSTOPS
      {
        #if Z_MAX_PIN > -1
          bool z_max_endstop=(READ(Z_MAX_PIN) != Z_ENDSTOPS_INVERTING);
          if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_z_max_endstop = z_max_endstop;
        #endif
      }
    }
#ifdef DYNAMIC_ADVANCE_OPTION
	 if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
		 if ( ! doAdvance ) REV_E_DIR();
		 count_direction[E_AXIS]=-1;
		 }
	 else { // +direction
		 if ( ! doAdvance ) NORM_E_DIR();
		 count_direction[E_AXIS]=1;
		 }
#else
      if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
#ifndef EXTRUDER_ADVANCE
       REV_E_DIR();
#endif //!EXTRUDER_ADVANCE
        count_direction[E_AXIS]=-1;
      }
      else { // +direction
#ifndef EXTRUDER_ADVANCE
       NORM_E_DIR();
#endif //!EXTRUDER_ADVANCE
       count_direction[E_AXIS]=1;
      }
    
#endif
    
	int8_t loops_completed = 0; // keep outside loop because advance needs to know how many loops were completed
   for( ; loops_completed < step_loops; ++loops_completed ) { // Take multiple steps per interrupt (For high speed moves) 
      #if !defined(__AVR_AT90USB1286__) && !defined(__AVR_AT90USB1287__)
      MSerial.checkRx(); // Check for serial chars.
      #endif 
      
      #if !defined COREXY      
      counter_x += current_block->steps_x;
      if (counter_x > 0) {
          WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
        counter_x -= current_block->step_event_count;
        count_position[X_AXIS]+=count_direction[X_AXIS];   
          WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
      }

      counter_y += current_block->steps_y;
      if (counter_y > 0) {
          WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
        counter_y -= current_block->step_event_count;
        count_position[Y_AXIS]+=count_direction[Y_AXIS];
          WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
      }
      #endif
  
      #ifdef COREXY
        counter_x += current_block->steps_x;        
        counter_y += current_block->steps_y;
        
        if ((counter_x > 0)&&!(counter_y>0)){  //X step only
          WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
          WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
          counter_x -= current_block->step_event_count; 
          count_position[X_AXIS]+=count_direction[X_AXIS];         
          WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
          WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
        }
        
        if (!(counter_x > 0)&&(counter_y>0)){  //Y step only
          WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
          WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
          counter_y -= current_block->step_event_count; 
          count_position[Y_AXIS]+=count_direction[Y_AXIS];
          WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
          WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
        }        
        
        if ((counter_x > 0)&&(counter_y>0)){  //step in both axes
          if (((out_bits & (1<<X_AXIS)) == 0)^((out_bits & (1<<Y_AXIS)) == 0)){  //X and Y in different directions
            WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
            counter_x -= current_block->step_event_count;             
            WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
            step_wait();
            count_position[X_AXIS]+=count_direction[X_AXIS];
            count_position[Y_AXIS]+=count_direction[Y_AXIS];
            WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
            counter_y -= current_block->step_event_count;
            WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
          }
          else{  //X and Y in same direction
            WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
            counter_x -= current_block->step_event_count;             
            WRITE(X_STEP_PIN, INVERT_X_STEP_PIN) ;
            step_wait();
            count_position[X_AXIS]+=count_direction[X_AXIS];
            count_position[Y_AXIS]+=count_direction[Y_AXIS];
            WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN); 
            counter_y -= current_block->step_event_count;    
            WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);        
          }
        }
      #endif //corexy

      counter_z += current_block->steps_z;
      if (counter_z > 0) {
        WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN);
        
		#ifdef Z_DUAL_STEPPER_DRIVERS
          WRITE(Z2_STEP_PIN, !INVERT_Z_STEP_PIN);
        #endif
        
        counter_z -= current_block->step_event_count;
        count_position[Z_AXIS]+=count_direction[Z_AXIS];
        WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);
        
		#ifdef Z_DUAL_STEPPER_DRIVERS
          WRITE(Z2_STEP_PIN, INVERT_Z_STEP_PIN);
        #endif
      }

      counter_e += current_block->steps_e;
      if (counter_e > 0) {
#ifdef DYNAMIC_ADVANCE_OPTION
			if ( ! doAdvance ) WRITE_E_STEP(!INVERT_E_STEP_PIN);
			counter_e -= current_block->step_event_count;
			count_position[E_AXIS] += count_direction[E_AXIS];
			if ( doAdvance ) {
				e_steps[current_block->active_extruder] += count_direction[E_AXIS];
			}
			else {
			    WRITE_E_STEP(INVERT_E_STEP_PIN);
			}
#else
#ifndef EXTRUDER_ADVANCE  //!EXTRUDER_ADVANCE
        WRITE_E_STEP(!INVERT_E_STEP_PIN);
#endif           //!EXTRUDER_ADVANCE
        counter_e -= current_block->step_event_count;
		 count_position[E_AXIS] += count_direction[E_AXIS];
#ifdef EXTRUDER_ADVANCE
			 e_steps[current_block->active_extruder] += count_direction[E_AXIS];
#else  //!EXTRUDER_ADVANCE
          WRITE_E_STEP(INVERT_E_STEP_PIN);
#endif //!EXTRUDER_ADVANCE
#endif
	  }

      step_events_completed += 1;  
      if(step_events_completed >= current_block->step_event_count) break;
		}  // end of looping

    // Calculate new timer value
    unsigned short timer;
    unsigned short step_rate;
    if (step_events_completed <= (unsigned long int)current_block->accelerate_until) {
      // still accelerating
      MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;
      
      // upper limit
      if(acc_step_rate > current_block->nominal_rate)
        acc_step_rate = current_block->nominal_rate;

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
      OCR1A = timer;
      acceleration_time += timer;
#ifdef EXTRUDER_ADVANCE
 #ifdef DYNAMIC_ADVANCE_OPTION
		if ( doAdvance ) {
 #endif
			// adjust advance based on the rate of change of advance that was set up
        advance += loops_completed * advance_rate;

        //if(advance > current_block->advance) advance = current_block->advance;
        // Do E steps + advance steps
        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
        old_advance = advance >>8;  
 #ifdef DYNAMIC_ADVANCE_OPTION
		}
 #endif
#endif
    } 
    else if (step_events_completed > (unsigned long int)current_block->decelerate_after) {   
      MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);
      
      if(step_rate > acc_step_rate) { // Check step_rate stays positive
        step_rate = current_block->final_rate;
      }
      else {
        step_rate = acc_step_rate - step_rate; // Decelerate from acceleration end point.
      }

      // lower limit
      if(step_rate < current_block->final_rate)
        step_rate = current_block->final_rate;

      // step_rate to timer interval
      timer = calc_timer(step_rate);
      OCR1A = timer;
      deceleration_time += timer;
      #ifdef EXTRUDER_ADVANCE
#ifdef DYNAMIC_ADVANCE_OPTION
		if ( doAdvance ) {
#endif
        advance -= loops_completed *unadvance_rate;
        //if(advance < final_advance) advance = final_advance;
        // Do E steps + advance steps
        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
        old_advance = advance >> 8;  
#ifdef DYNAMIC_ADVANCE_OPTION
			}
#endif
      #endif //EXTRUDER_ADVANCE
    }
    else {
      OCR1A = OCR1A_nominal;
    }

    // If current block is finished, reset pointer 
    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      plan_discard_current_block();
    }   
  } 

#ifdef EXTRUDER_ADVANCE
#ifndef ADVANCE_HAS_OWN_INTERRUPT_SERVICE_ROUTINE
#ifdef DYNAMIC_ADVANCE_OPTION
  if ( doAdvance ) {
#endif
  HandleExtruderAdvance();
#ifdef DYNAMIC_ADVANCE_OPTION
  }
#endif
#endif
#endif
}

#ifdef EXTRUDER_ADVANCE
 #ifdef ADVANCE_HAS_OWN_INTERRUPT_SERVICE_ROUTINE
  unsigned char old_OCR0A;
  // Timer interrupt for E. e_steps is set in the main routine;
  // Timer 0 is shared with millies
  ISR(TIMER0_COMPA_vect)
  {
	 // update the match register to re-interrupt at the right time
    old_OCR0A += 52; // ~10kHz interrupt (250000 / 26 = 9615kHz)
    OCR0A = old_OCR0A;
	 const int MAX_E_STEPS_PER_TIMER_TICK = 2; // don't generate a flurry of extra ticks. The stepper can only react to a limited number per millisecond.
 #else
void HandleExtruderAdvance()
	{
	// we only get called once per main step - no fixed timing, but the timing will be more frequent along with increasing speed of the head.
	// It's not necessarily the best, but it does end up allowing an acceleration type pattern to occur.
	//const int MAX_E_STEPS_PER_TIMER_TICK = 4; // don't generate a flurry of extra ticks. The stepper can only react to a limited number per millisecond.
	const int MAX_E_STEPS_PER_TIMER_TICK = 1; // don't generate a flurry of extra ticks. The stepper can only react to a limited number per millisecond.
 #endif
    // Set E direction (Depends on E direction + advance)
    for ( unsigned char i=0; i < MAX_E_STEPS_PER_TIMER_TICK; i++ ) {
      if (e_steps[0] != 0) {
			// direction changes may only occur when the step pin is going to be stable, or has been stable for a minimum time (200ns for Allegro A4982).
			// There's a bit of code before and after the direction setting here, so it's probably OK.
        WRITE(E0_STEP_PIN, INVERT_E_STEP_PIN);
        if ( e_steps[0] < 0 ) {
          WRITE(E0_DIR_PIN, INVERT_E0_DIR);
          e_steps[0]++;
        } 
        else if ( e_steps[0] > 0 ) {
          WRITE(E0_DIR_PIN, !INVERT_E0_DIR);
          e_steps[0]--;
        }
		  WRITE(E0_STEP_PIN, !INVERT_E_STEP_PIN);
      }
 #if EXTRUDERS > 1
      if (e_steps[1] != 0) {
        WRITE(E1_STEP_PIN, INVERT_E_STEP_PIN);
        if (e_steps[1] < 0) {
          WRITE(E1_DIR_PIN, INVERT_E1_DIR);
          e_steps[1]++;
          WRITE(E1_STEP_PIN, !INVERT_E_STEP_PIN);
        } 
        else if (e_steps[1] > 0) {
          WRITE(E1_DIR_PIN, !INVERT_E1_DIR);
          e_steps[1]--;
          WRITE(E1_STEP_PIN, !INVERT_E_STEP_PIN);
        }
      }
 #endif
 #if EXTRUDERS > 2
      if (e_steps[2] != 0) {
        WRITE(E2_STEP_PIN, INVERT_E_STEP_PIN);
        if (e_steps[2] < 0) {
          WRITE(E2_DIR_PIN, INVERT_E2_DIR);
          e_steps[2]++;
          WRITE(E2_STEP_PIN, !INVERT_E_STEP_PIN);
        } 
        else if (e_steps[2] > 0) {
          WRITE(E2_DIR_PIN, !INVERT_E2_DIR);
          e_steps[2]--;
          WRITE(E2_STEP_PIN, !INVERT_E_STEP_PIN);
        }
      }
 #endif
    }
  }
#endif // EXTRUDER_ADVANCE

void st_init()
{
  //Initialize Dir Pins
  #if X_DIR_PIN > -1
    SET_OUTPUT(X_DIR_PIN);
  #endif
  #if Y_DIR_PIN > -1 
    SET_OUTPUT(Y_DIR_PIN);
  #endif
  #if Z_DIR_PIN > -1 
    SET_OUTPUT(Z_DIR_PIN);

    #if defined(Z_DUAL_STEPPER_DRIVERS) && (Z2_DIR_PIN > -1)
      SET_OUTPUT(Z2_DIR_PIN);
    #endif
  #endif
  #if E0_DIR_PIN > -1 
    SET_OUTPUT(E0_DIR_PIN);
  #endif
  #if defined(E1_DIR_PIN) && (E1_DIR_PIN > -1)
    SET_OUTPUT(E1_DIR_PIN);
  #endif
  #if defined(E2_DIR_PIN) && (E2_DIR_PIN > -1)
    SET_OUTPUT(E2_DIR_PIN);
  #endif

  //Initialize Enable Pins - steppers default to disabled.

  #if (X_ENABLE_PIN > -1)
    SET_OUTPUT(X_ENABLE_PIN);
    if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
  #endif
  #if (Y_ENABLE_PIN > -1)
    SET_OUTPUT(Y_ENABLE_PIN);
    if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
  #endif
  #if (Z_ENABLE_PIN > -1)
    SET_OUTPUT(Z_ENABLE_PIN);
    if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);
    
    #if defined(Z_DUAL_STEPPER_DRIVERS) && (Z2_ENABLE_PIN > -1)
      SET_OUTPUT(Z2_ENABLE_PIN);
      if(!Z_ENABLE_ON) WRITE(Z2_ENABLE_PIN,HIGH);
    #endif
  #endif
  #if (E0_ENABLE_PIN > -1)
    SET_OUTPUT(E0_ENABLE_PIN);
    if(!E_ENABLE_ON) WRITE(E0_ENABLE_PIN,HIGH);
  #endif
  #if defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
    SET_OUTPUT(E1_ENABLE_PIN);
    if(!E_ENABLE_ON) WRITE(E1_ENABLE_PIN,HIGH);
  #endif
  #if defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
    SET_OUTPUT(E2_ENABLE_PIN);
    if(!E_ENABLE_ON) WRITE(E2_ENABLE_PIN,HIGH);
  #endif

  //endstops and pullups
  
    #if X_MIN_PIN > -1
      SET_INPUT(X_MIN_PIN); 
    #ifdef ENDSTOPPULLUP_XMIN
      WRITE(X_MIN_PIN,HIGH);
    #endif
    #endif
      
    #if Y_MIN_PIN > -1
      SET_INPUT(Y_MIN_PIN); 
    #ifdef ENDSTOPPULLUP_YMIN
      WRITE(Y_MIN_PIN,HIGH);
    #endif
    #endif
  
    #if Z_MIN_PIN > -1
      SET_INPUT(Z_MIN_PIN); 
    #ifdef ENDSTOPPULLUP_ZMIN
      WRITE(Z_MIN_PIN,HIGH);
    #endif
    #endif
      
    #if X_MAX_PIN > -1
      SET_INPUT(X_MAX_PIN); 
    #ifdef ENDSTOPPULLUP_XMAX
      WRITE(X_MAX_PIN,HIGH);
    #endif
    #endif
      
    #if Y_MAX_PIN > -1
      SET_INPUT(Y_MAX_PIN); 
    #ifdef ENDSTOPPULLUP_YMAX
      WRITE(Y_MAX_PIN,HIGH);
    #endif
    #endif
  
    #if Z_MAX_PIN > -1
      SET_INPUT(Z_MAX_PIN); 
    #ifdef ENDSTOPPULLUP_ZMAX
      WRITE(Z_MAX_PIN,HIGH);
    #endif
  #endif
 

  //Initialize Step Pins
  #if (X_STEP_PIN > -1) 
    SET_OUTPUT(X_STEP_PIN);
    WRITE(X_STEP_PIN,INVERT_X_STEP_PIN);
    if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
  #endif  
  #if (Y_STEP_PIN > -1) 
    SET_OUTPUT(Y_STEP_PIN);
    WRITE(Y_STEP_PIN,INVERT_Y_STEP_PIN);
    if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
  #endif  
  #if (Z_STEP_PIN > -1) 
    SET_OUTPUT(Z_STEP_PIN);
    WRITE(Z_STEP_PIN,INVERT_Z_STEP_PIN);
    if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);
    
    #if defined(Z_DUAL_STEPPER_DRIVERS) && (Z2_STEP_PIN > -1)
      SET_OUTPUT(Z2_STEP_PIN);
      WRITE(Z2_STEP_PIN,INVERT_Z_STEP_PIN);
      if(!Z_ENABLE_ON) WRITE(Z2_ENABLE_PIN,HIGH);
    #endif
  #endif  
  #if (E0_STEP_PIN > -1) 
    SET_OUTPUT(E0_STEP_PIN);
    WRITE(E0_STEP_PIN,INVERT_E_STEP_PIN);
    if(!E_ENABLE_ON) WRITE(E0_ENABLE_PIN,HIGH);
  #endif  
  #if defined(E1_STEP_PIN) && (E1_STEP_PIN > -1) 
    SET_OUTPUT(E1_STEP_PIN);
    WRITE(E1_STEP_PIN,INVERT_E_STEP_PIN);
    if(!E_ENABLE_ON) WRITE(E1_ENABLE_PIN,HIGH);
  #endif  
  #if defined(E2_STEP_PIN) && (E2_STEP_PIN > -1) 
    SET_OUTPUT(E2_STEP_PIN);
    WRITE(E2_STEP_PIN,INVERT_E_STEP_PIN);
    if(!E_ENABLE_ON) WRITE(E2_ENABLE_PIN,HIGH);
  #endif  

  #ifdef CONTROLLERFAN_PIN
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif
  
  // waveform generation = 0100 = CTC
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 
  
  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  OCR1A = 0x4000;
  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();  

#ifdef EXTRUDER_ADVANCE
  e_steps[0] = 0;
  e_steps[1] = 0;
  e_steps[2] = 0;
 #ifdef ADVANCE_HAS_OWN_INTERRUPT_SERVICE_ROUTINE
  // set up interrupt
  #if defined(TCCR0A) && defined(WGM01)
    TCCR0A &= ~(1<<WGM01);
    TCCR0A &= ~(1<<WGM00);
  #endif  
    TIMSK0 |= (1<<OCIE0A);
 #endif // ADVANCE_HAS_OWN_INTERRUPT_SERVICE_ROUTINE
#endif //EXTRUDER_ADVANCE
  
  enable_endstops(true); // Start with endstops active. After homing they can be disabled
  sei();
}


// Block until all buffered steps are executed
void st_synchronize()
{
    while( blocks_queued()) {
		 DoBackgroundProcessingTick();
  }
}

void st_set_position(const long &x, const long &y, const long &z, const long &e)
{
  CRITICAL_SECTION_START;
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void st_set_e_position(const long &e)
{
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

long st_get_position(uint8_t axis)
{
  long count_pos;
  CRITICAL_SECTION_START;
  count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

void finishAndDisableSteppers()
{
  st_synchronize(); 
  LCD_MESSAGEPGM(MSG_STEPPER_RELEASED);
  disable_x(); 
  disable_y(); 
  disable_z(); 
  disable_e0(); 
  disable_e1(); 
  disable_e2(); 
}

void quickStop()
{
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while(blocks_queued())
    plan_discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

