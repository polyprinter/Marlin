/*
  planner.c - buffers movement commands and manages the acceleration profile plan
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

/* The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis. */

/*  
 Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 
 s == speed, a == acceleration, t == time, d == distance
 
 Basic definitions:
 
 Speed[s_, a_, t_] := s + (a*t) 
 Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 
 Distance to reach a specific speed with a constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 
 Speed after a given distance of travel with constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 m -> Sqrt[2 a d + s^2]    
 
 DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 
 When to start braking (di) to reach a specified destionation speed (s2) after accelerating
 from initial speed s1 without ever stopping at a plateau:
 
 Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 
 IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 */

#include "Marlin.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"

#define RECALCULATE_PLANNER  // define to allow re-planning. This is normal.

//===========================================================================
//=============================public variables ============================
//===========================================================================

unsigned long minsegmenttime;
float max_feedrate[4]; // set the max speeds
#ifdef ORIGINAL_ESTIMATES
#else
float max_axis_jerk[NUM_STEPPERS]; // set the max jerks for easy processing
#endif
float axis_steps_per_unit[4];
unsigned long max_acceleration_units_per_sq_second[NUM_STEPPERS]; // Use M201 to override by software
float minimumfeedrate;
float acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT (maximum attempted setting) ACCELERATION for all moves. M204 SXXXX Individual axis accelerations may reduce this but wil not increase it.
float retract_acceleration; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
float max_xy_jerk; //speed than can be stopped at once, if i understand correctly.
float max_z_jerk;
float max_e_jerk;
float mintravelfeedrate;
unsigned long axis_steps_per_sqr_second[NUM_AXIS];

#ifdef EXTRUDER_ADVANCE
float extruder_advance_k; // defaulted elsewhere
#endif
#ifdef DEBUG_VARS
int extruder_debug_i = 0;
int extruder_debug_j = 0;
int extruder_debug_k = 0;
#endif

// The current position of the tool in absolute steps
long position[NUM_STEPPERS];   //rescaled from extern when axis_steps_per_unit are changed by gcode
static float previous_speed[NUM_STEPPERS]; // Speed of previous path line segment
#ifdef ORIGINAL_ESTIMATES
#else
static float previous_exit_speed[NUM_STEPPERS]; // Speed of previous path line segment's ending
#endif
static float previous_nominal_speed; // Nominal speed of previous path line segment

extern volatile int extrudemultiply; // Sets extrude multiply factor (in percent)

#ifdef AUTOTEMP
float autotemp_max=250;
float autotemp_min=210;
float autotemp_factor=0.1;
bool autotemp_enabled=false;
#endif

//===========================================================================
//=================semi-private variables, used in inline  functions    =====
//===========================================================================
block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
volatile unsigned char block_buffer_tail;           // Index of the block to process now

//===========================================================================
//=============================private variables ============================
//===========================================================================
#ifdef PREVENT_DANGEROUS_EXTRUDE
bool allow_cold_extrude=false;
#endif
#ifdef XY_FREQUENCY_LIMIT
// Used for the frequency limit
static unsigned char old_direction_bits = 0;               // Old direction bits. Used for speed calculations
static long x_segment_time[3]={
  0,0,0};                     // Segment times (in us). Used for speed calculations
static long y_segment_time[3]={
  0,0,0};
#endif

// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
static int8_t next_block_index(int8_t block_index) {
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { 
    block_index = 0; 
  }
  return(block_index);
}


// Returns the index of the previous block in the ring buffer
static int8_t prev_block_index(int8_t block_index) {
  if (block_index == 0) { 
    block_index = BLOCK_BUFFER_SIZE; 
  }
  block_index--;
  return(block_index);
}

//===========================================================================
//=============================functions         ============================
//===========================================================================
#ifdef EXTRUDER_ADVANCE
#ifdef FLOAT_ESTIMATES
#else
FORCE_INLINE uint32_t estimate_acceleration_distance_from0( uint32_t target_rate, uint32_t acceleration )
	{
	return ( ( target_rate * target_rate ) / ( 2 * acceleration ) );
	}
#endif
#endif

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
#ifdef FLOAT_ESTIMATES
FORCE_INLINE float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
  if (acceleration!=0) {
    return((target_rate*target_rate-initial_rate*initial_rate)/
      (2.0*acceleration));
  }
  else {
    return 0.0;  // acceleration was 0, set acceleration distance to 0
  }
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

FORCE_INLINE float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance) 
	{
	if (acceleration!=0) {
		return((2.0*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
			(4.0*acceleration) );
		}
	else {
		return 0.0;  // acceleration was 0, set intersection distance to 0
		}
	}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance) {
	return  sqrt(target_velocity*target_velocity-2*acceleration*distance);
	}


#else

// using uint should be good up till 65,535 steps/sec
/*FORCE_INLINE uint32_t estimate_acceleration_distance( uint32_t initial_rate, uint32_t target_rate, uint32_t acceleration )
	{
	if ( acceleration != 0 ) {

		if ( target_rate >= ( (uint32_t)1 << 16 ) ) {
			// it will overflow! can't use longs for the calculation in that case
			MYSERIAL.print( "rate high for long sqr" ); 
			MYSERIAL.print( ' ' );  
			MYSERIAL.println( target_rate );  
			}

		return ( ( target_rate * target_rate - initial_rate * initial_rate ) / ( 2 * acceleration ) );
		}
	else {
		return 0;  // acceleration was 0, set acceleration distance to 0
		}
	}
*/

// using uint should be good up till 65,535 steps/sec
FORCE_INLINE uint32_t estimate_acceleration_distance_precalc( uint32_t initial_rate_sq, uint32_t target_rate_sq, uint32_t acceleration_x2 )
	{
	return ( ( target_rate_sq - initial_rate_sq ) / acceleration_x2 );
	}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

/*FORCE_INLINE uint32_t intersection_distance( uint32_t initial_rate, uint32_t final_rate, uint32_t acceleration, uint32_t distance ) 
	{
	return ( 2 * acceleration * distance - initial_rate * initial_rate + final_rate * final_rate ) / ( 4 * acceleration );
	}
*/
// the result must be singed, because with Jerk involved, we may end up with a result telling us that it's impossible
// to decelerate to the target rate in time, or that it's impossible to accelerate to the target rate in time
/*FORCE_INLINE int32_t intersection_distance_precalc( uint32_t initial_rate_sq, uint32_t target_rate_sq, uint32_t acceleration_x2, uint32_t distance ) 
	{
	if ( ( (float)acceleration_x2 * distance ) >= ( (float)( (uint32_t)1 << 31 ) ) ) {
		// it will overflow! can't use longs for the calculation in that case
		// but this calc only gets done if there is no plateau, which is less likely when the distance is very large
		MYSERIAL.print( "accel_x2 * dist high for long" ); 
		MYSERIAL.print( ' ' );  
		MYSERIAL.println( acceleration_x2 );  
		MYSERIAL.print( ' ' );  
		MYSERIAL.println( distance );  
		}

	// with a very high initial rate, and a low final rate we may get a negative result if there's not actually enough distance
	// to accomplish the task
	return ( (int32_t)( acceleration_x2 * distance ) - initial_rate_sq + target_rate_sq ) / ( 2 * acceleration_x2 );
	}
*/

FORCE_INLINE int32_t intersection_distance_precalc( int32_t initial_rate_sq, int32_t target_rate_sq, int32_t acceleration_x2, int32_t distance ) 
	{
	if ( ( (float)acceleration_x2 * distance ) >= ( (float)( (uint32_t)1 << 31 ) ) ) {
		// it will overflow! can't use longs for the calculation in that case
		// but this calc only gets done if there is no plateau, which is less likely when the distance is very large
		MYSERIAL.print( "accel_x2 * dist high for long" ); 
		MYSERIAL.print( ' ' );  
		MYSERIAL.println( acceleration_x2 );  
		MYSERIAL.print( ' ' );  
		MYSERIAL.println( distance );  
		}

	// with a very high initial rate, and a low final rate we may get a negative result if there's not actually enough distance
	// to accomplish the task
	return ( (int32_t)( acceleration_x2 * distance ) - initial_rate_sq + target_rate_sq ) / ( 2 * acceleration_x2 );
	}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
// NOTE: we always call this with a negative accel so it cannot produce a neg sqrt
FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance) {
	if ( (target_velocity*target_velocity-2*acceleration*distance ) < 0 ) {
		MYSERIAL.print( "max_allowable_speed has neg. sqrt" ); 
		MYSERIAL.print( " vel:" );  
		MYSERIAL.println( target_velocity );  
		MYSERIAL.print( " accel:" );  
		MYSERIAL.println( acceleration );  
		MYSERIAL.print( " dist:" );  
		MYSERIAL.println( distance );  
		}

	return  sqrt(target_velocity*target_velocity-2*acceleration*distance);
	}

#endif

//#define trace_unachievables

#ifdef trace_unachievables
// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
static int bTrapezoidDetectedError = 0;
#endif

void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor) {
	// never allow speeds faster than the nominal rate for the block
	entry_factor = min( entry_factor, 1.0f );
	exit_factor  = min( exit_factor,  1.0f );

	unsigned long initial_rate = ceil(block->nominal_rate*entry_factor); // (step/min)
	unsigned long final_rate = ceil(block->nominal_rate*exit_factor); // (step/min)

	// Limit minimal step rate (Otherwise the timer will overflow.)
	if(initial_rate <120) {
	 initial_rate=120; 
	}
	if(final_rate < 120) {
	 final_rate=120;  
	}

	long acceleration = block->acceleration_st;
#ifdef ORIGINAL_TRAPEZOID
  int32_t accelerate_steps =
    ceil(estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration));
  int32_t decelerate_steps =
    floor(estimate_acceleration_distance(block->nominal_rate, block->final_rate, -acceleration));

#ifdef EXTRUDER_ADVANCE
 #ifdef ADVANCE_WITH_SQUARE_LAW
  long init_advance = block->advance*entry_factor*entry_factor; 
  long fin_advance  = block->advance*exit_factor*exit_factor;
 #else
  // it's linear
  long init_advance = block->advance * entry_factor; 
  long fin_advance  = block->advance * exit_factor;
 #endif
  long nominal_accel_steps = accelerate_steps;
  long nominal_decel_steps = decelerate_steps;
#endif // EXTRUDER_ADVANCE

  // Calculate the size of Plateau of Nominal Rate.
  int32_t plateau_steps = block->step_event_count - accelerate_steps-decelerate_steps;

  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if ( plateau_steps < 0 ) {
    accelerate_steps = ceil( intersection_distance(block->initial_rate, block->final_rate, acceleration, block->step_event_count) );
    accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
    accelerate_steps = min(accelerate_steps,block->step_event_count);
    plateau_steps = 0;
  }


  // block->accelerate_until = accelerate_steps;
  // block->decelerate_after = accelerate_steps+plateau_steps;
  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if ( block->busy == false ) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps + plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate   = final_rate;
#ifdef EXTRUDER_ADVANCE
    block->initial_advance = init_advance;
    block->final_advance   = fin_advance;
#endif // EXTRUDER_ADVANCE
  }
  CRITICAL_SECTION_END;
#else

	if ( acceleration == 0 ) {
		// skip all the calcs - we'd get errors
		CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
		if ( block->busy == false ) { // Don't update variables if block is busy.
			block->accelerate_until = 0;
			block->decelerate_after = block->step_event_count;
			block->initial_rate = initial_rate;
			block->final_rate   = final_rate;
#ifdef EXTRUDER_ADVANCE
			block->initial_advance = block->advance;
			block->final_advance   = block->advance;
			//block->advance_rate   = 0;		
			//block->unadvance_rate = 0;		
#endif //EXTRUDER_ADVANCE
			}
		CRITICAL_SECTION_END;
		}
	else {
	#ifdef FLOAT_ESTIMATES
		int32_t accelerate_steps = ceil(  estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration ) );
		int32_t decelerate_steps = floor( estimate_acceleration_distance(block->nominal_rate, block->final_rate,  -acceleration ) );
		int32_t plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;
	#else
		if ( block->nominal_rate >= ( (uint32_t)1 << 16 ) ) {
			// it will overflow! can't use longs for the calculation in that case
			MYSERIAL.print( "rate high for long sqr" ); 
			MYSERIAL.print( ' ' );  
			MYSERIAL.println( block->nominal_rate );  
			}

		// eliminate redundant calcs
		uint32_t nominal_sq = block->nominal_rate * block->nominal_rate;
		uint32_t initial_sq = block->initial_rate * block->initial_rate;
		uint32_t final_sq   = block->final_rate   * block->final_rate;
		uint32_t accel_x2 = acceleration * 2;

		#ifdef check_assumptions
		// see if there are any surprises
		if ( acceleration < 0 ) {
			MYSERIAL.println( "acceleration < 0" ); 
			}
		if ( initial_sq > nominal_sq ) {
			MYSERIAL.println( "initial_sq > nominal_sq" ); 
			MYSERIAL.print( " i:" );  
			MYSERIAL.print( block->initial_rate  );  
			MYSERIAL.print( " n:" );  
			MYSERIAL.print( block->nominal_rate  );  
			MYSERIAL.print( " a:" );  
			MYSERIAL.println( acceleration  );  
		}
		if ( final_sq > nominal_sq ) {
			MYSERIAL.println( "final_sq > nominal_sq" ); 
		}
		if ( initial_sq > final_sq ) {
			MYSERIAL.println( "initial_sq > final_sq" ); 
			MYSERIAL.print( " i:" );  
			MYSERIAL.print( block->initial_rate  );  
			MYSERIAL.print( " n:" );  
			MYSERIAL.print( block->nominal_rate  );  
			MYSERIAL.print( " f:" );  
			MYSERIAL.print( block->final_rate  );  
			MYSERIAL.print( " a:" );  
			MYSERIAL.println( acceleration  );  
		}
		#endif

		// Calculate the size of Plateau of Nominal Rate.
		uint32_t accelerate_steps = estimate_acceleration_distance_precalc( initial_sq, nominal_sq, accel_x2 );
		int32_t plateau_steps = block->step_event_count - accelerate_steps; // partial result
	 #ifdef EXTRUDER_ADVANCE_NEEDS_DECELSTEPS
		uint32_t decelerate_steps = estimate_acceleration_distance_precalc( final_sq,   nominal_sq, accel_x2 );   // make sure they're set up for a positive calc result
		plateau_steps -= decelerate_steps;
	 #else
		// can sometimes skip the calculation
		if ( plateau_steps > 0 ) { 
			// there might be a plateau - it depends upon the decleration distance
			uint32_t decelerate_steps = estimate_acceleration_distance_precalc( final_sq,   nominal_sq, accel_x2 );   // make sure they're set up for a positive calc result
			plateau_steps -= decelerate_steps;
			}
	 #endif
	#endif

	  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
	  // have to use intersection_distance() to calculate when to abort acceleration and start braking
	  // in order to reach the final_rate exactly at the end of this block.
	  if ( plateau_steps < 0 ) {
	#ifdef FLOAT_ESTIMATES
		 accelerate_steps = ceil( intersection_distance(block->initial_rate, block->final_rate, acceleration, block->step_event_count) );
		 accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
		 accelerate_steps = min( accelerate_steps, block->step_event_count );
	#else
		 //accelerate_steps = intersection_distance( block->initial_rate, block->final_rate, acceleration, block->step_event_count );
		 int32_t intersect_steps = intersection_distance_precalc( initial_sq, final_sq, accel_x2, block->step_event_count );
#ifdef trace_unachievables
		 const int32_t TRACE_THRESHOLD_ERROR = 1000;
		 const int32_t TRACE_THRESHOLD_ERROR_UPPER = (int32_t)( block->step_event_count + TRACE_THRESHOLD_ERROR );
		 const int32_t TRACE_THRESHOLD_ERROR_LOWER = ( -TRACE_THRESHOLD_ERROR );
		 if ( intersect_steps > (int32_t)( block->step_event_count + TRACE_THRESHOLD_ERROR ) || intersect_steps < ( -TRACE_THRESHOLD_ERROR ) ) {
			 bTrapezoidDetectedError = 1;
			 // it would take more distance than we apparently have to get to the final speed. Something's wrong with planning, or Jerk is being relied upon.
			 MYSERIAL.print( "intersection_distance_precalc got bad result" ); 
			 MYSERIAL.print( " intersect:" );  
			 MYSERIAL.print( intersect_steps );  
			 MYSERIAL.print( " TRACE_THRESHOLD_ERROR:" );  
			 MYSERIAL.print( TRACE_THRESHOLD_ERROR );  
			 MYSERIAL.print( " TRACE_THRESHOLD_ERROR_UPPER:" );  
			 MYSERIAL.print( TRACE_THRESHOLD_ERROR_UPPER );  
			 MYSERIAL.print( " TRACE_THRESHOLD_ERROR_LOWER:" );  
			 MYSERIAL.print( TRACE_THRESHOLD_ERROR_LOWER );  
			 MYSERIAL.print( " d:" );  
			 MYSERIAL.print( block->step_event_count  );  
			 MYSERIAL.print( " i2:" );  
			 MYSERIAL.print( initial_sq );  
			 MYSERIAL.print( " f2:" );  
			 MYSERIAL.print( final_sq );  
			 MYSERIAL.print( " 2a:" );  
			 MYSERIAL.println( accel_x2 );  
			 MYSERIAL.print( " asteps:" );  
			 MYSERIAL.println( accelerate_steps );  
			 //MYSERIAL.print( " decel steps:" );  
			 //MYSERIAL.print( decelerate_steps );  
		}
#endif

		 if ( intersect_steps > block->step_event_count ) {
			 accelerate_steps = block->step_event_count; // a likely good default - certainly if start and end speeds are the same
		 }
		 else {
			 accelerate_steps = max( intersect_steps, 0 ); // Check limits due to impossible tasks being assigned (bad planning? Jerk?)
		 }
	#endif
		 plateau_steps = 0;
	  }

#ifdef EXTRUDER_ADVANCE
#ifdef ADVANCE_WITH_SQUARE_LAW
	  long init_advance = block->advance*entry_factor*entry_factor; 
	  long fin_advance  = block->advance*exit_factor*exit_factor;
#else
	  // it's linear
	  long init_advance = block->advance * entry_factor; 
	  long fin_advance  = block->advance * exit_factor;
#endif
#endif // EXTRUDER_ADVANCE

	  // block->accelerate_until = accelerate_steps;
	  // block->decelerate_after = accelerate_steps+plateau_steps;
	  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
	  if ( block->busy == false ) { // Don't update variables if block is busy.
		  block->accelerate_until = accelerate_steps;
		  block->decelerate_after = accelerate_steps + plateau_steps;
		  block->initial_rate = initial_rate;
		  block->final_rate   = final_rate;
#ifdef EXTRUDER_ADVANCE
		  block->initial_advance = init_advance;
		  block->final_advance   = fin_advance;
		  // block.advance here is what it would be if full velocity is reached, so we use the nominal steps for each phase, and only as much of 
		  // each will be applied as there is time and distance for.
		  //block->advance_rate   = advance_rate;	
		 // block->unadvance_rate = unadvance_rate;		  
#endif //EXTRUDER_ADVANCE
		  }
	  CRITICAL_SECTION_END;
	  }
#endif // end of new estimates vs old
	  
#ifdef TRACE_REPLAN_ADVANCE
	  SERIAL_ECHO_START;
	  SERIAL_ECHOPGM("r advance :");  // replan
	  SERIAL_ECHO(   block->advance );
	  SERIAL_ECHOPGM(" advance rate :");
	  SERIAL_ECHO( block->advance_rate );
	  SERIAL_ECHOPGM(" unadvance rate :");
	  SERIAL_ECHOLN( block->unadvance_rate );
#endif
	}

                

// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal 
// velocities of the respective blocks.
//inline float junction_jerk(block_t *before, block_t *after) {
//  return sqrt(
//    pow((before->speed_x-after->speed_x), 2)+pow((before->speed_y-after->speed_y), 2));
//}

#ifdef RECALCULATE_PLANNER

#ifdef ORIGINAL_PLANNER
#else
// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_jerk_check_kernel( block_t *previous, block_t *current, block_t *next) {
	if(!current) { 
		return; 
		}

	if (next) {
		// If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
		// If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
		// check for maximum allowable speed reductions to ensure maximum possible planned speed.
		if (current->entry_speed != current->max_entry_speed) {

			// If nominal length true, max junction speed is guaranteed to be reached. Only compute
			// for max allowable speed if block is decelerating and nominal length is false.
			if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
				current->entry_speed = min( current->max_entry_speed, max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
				} 
			else {
				current->entry_speed = current->max_entry_speed;
				}
			current->recalculate_flag = true;

			}
		} // Skip last block. Already initialized and set for recalculation.
	}

#endif

// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!current) { 
    return; 
  }

  if (next) {
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed) {

      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
      if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
        current->entry_speed = min( current->max_entry_speed, max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
      } 
      else {
        current->entry_speed = current->max_entry_speed;
      }
      current->recalculate_flag = true;

    }
  } // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
void planner_reverse_pass() {
  uint8_t block_index = block_buffer_head;
  
  //Make a local copy of block_buffer_tail, because the interrupt can alter it
  CRITICAL_SECTION_START;
  unsigned char tail = block_buffer_tail;
  CRITICAL_SECTION_END
  
  if(((block_buffer_head-tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) {
    block_index = (block_buffer_head - 3) & (BLOCK_BUFFER_SIZE - 1);
    block_t *block[3] = { 
      NULL, NULL, NULL         };
    while(block_index != tail) { 
      block_index = prev_block_index(block_index); 
      block[2]= block[1];
      block[1]= block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
  }
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!previous) { 
    return; 
  }

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!previous->nominal_length_flag) {
    if (previous->entry_speed < current->entry_speed) {
#ifdef ORIGINAL_PLANNER
      double entry_speed = min( current->entry_speed,
      max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );

#else
      float entry_speed = min( current->entry_speed,  max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );
#endif
      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
void planner_forward_pass() {
  uint8_t block_index = block_buffer_tail;
  block_t *block[3] = { 
    NULL, NULL, NULL   };

  while(block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0],block[1],block[2]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;

  while(block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_speed,
        next->entry_speed/current->nominal_speed);
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index( block_index );
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if(next != NULL) {
    calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_speed,
    MINIMUM_PLANNER_SPEED/next->nominal_speed);
    next->recalculate_flag = false;
  }
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor) 
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant 
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if 
//     a. The speed increase within one block would require faster accelleration than the one, true 
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to 
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than 
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

void planner_recalculate() {   
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}
#endif

void clear_planning_vars() {
	for ( uint8_t a=0; a < NUM_STEPPERS; ++a ) {
		previous_speed[a] = 0.0;
#ifdef ORIGINAL_ESTIMATES
#else		
		previous_exit_speed[a] = 0.0;
#endif		
	}

	previous_nominal_speed = 0.0;
}

void plan_init() {
  block_buffer_head = 0;
  block_buffer_tail = 0;
  memset(position, 0, sizeof(position)); // clear position

  clear_planning_vars();
}




#ifdef AUTOTEMP
void getHighESpeed()
{
  static float oldt=0;
  if(!autotemp_enabled){
    return;
  }
  if(degTargetHotend0()+2<autotemp_min) {  //probably temperature set to zero.
    return; //do nothing
  }

  float high=0.0;
  uint8_t block_index = block_buffer_tail;

  while(block_index != block_buffer_head) {
    if((block_buffer[block_index].steps_x != 0) ||
      (block_buffer[block_index].steps_y != 0) ||
      (block_buffer[block_index].steps_z != 0)) {
      float se=(float(block_buffer[block_index].steps_e)/float(block_buffer[block_index].step_event_count))*block_buffer[block_index].nominal_speed;
      //se; mm/sec;
      if(se>high)
      {
        high=se;
      }
    }
    block_index = (block_index+1) & (BLOCK_BUFFER_SIZE - 1);
  }

  float g=autotemp_min+high*autotemp_factor;
  float t=g;
  if(t<autotemp_min)
    t=autotemp_min;
  if(t>autotemp_max)
    t=autotemp_max;
  if(oldt>t)
  {
    t=AUTOTEMP_OLDWEIGHT*oldt+(1-AUTOTEMP_OLDWEIGHT)*t;
  }
  oldt=t;
  setTargetHotend0(t);
}
#endif

void check_axes_activity() {
  unsigned char x_active = 0;
  unsigned char y_active = 0;  
  unsigned char z_active = 0;
  unsigned char e_active = 0;
  unsigned char fan_speed = 0;
  unsigned char tail_fan_speed = 0;
  block_t *block;

  if(block_buffer_tail != block_buffer_head) {
    uint8_t block_index = block_buffer_tail;
    tail_fan_speed = block_buffer[block_index].fan_speed;
    while(block_index != block_buffer_head) {
      block = &block_buffer[block_index];
      if(block->steps_x != 0) x_active++;
      if(block->steps_y != 0) y_active++;
      if(block->steps_z != 0) z_active++;
      if(block->steps_e != 0) e_active++;
      if(block->fan_speed != 0) fan_speed++;
      block_index = (block_index+1) & (BLOCK_BUFFER_SIZE - 1);
    }
  }
  else {
#if FAN_PIN > -1
    if (FanSpeed != 0){
      analogWrite(FAN_PIN,FanSpeed); // If buffer is empty use current fan speed
    }
#endif
  }
  if((DISABLE_X) && (x_active == 0)) disable_x();
  if((DISABLE_Y) && (y_active == 0)) disable_y();
  if((DISABLE_Z) && (z_active == 0)) disable_z();
  if((DISABLE_E) && (e_active == 0)) { 
    disable_e0();
    disable_e1();
    disable_e2(); 
  }
#if FAN_PIN > -1
  if((FanSpeed == 0) && (fan_speed ==0)) {
    analogWrite(FAN_PIN, 0);
  }

  if (FanSpeed != 0 && tail_fan_speed !=0) { 
    analogWrite(FAN_PIN,tail_fan_speed);
  }
#endif
#ifdef AUTOTEMP
  getHighESpeed();
#endif
}


float junction_deviation = 0.1;
// Add a new linear movement to the buffer. steps_x, _y and _z is the absolute position in 
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
void plan_buffer_line(const float &x, const float &y, const float &z, const float &e, float feed_rate, const uint8_t &extruder)
{
  // Calculate the buffer head after we push this byte
  int next_buffer_head = next_block_index(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot. 
  // Rest here until there is room in the buffer.
  while(block_buffer_tail == next_buffer_head) { 
	 DoBackgroundProcessingTick();
  }

  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  long target[4];
  target[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
  target[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
  target[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);     
  target[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);

#ifdef PREVENT_DANGEROUS_EXTRUDE
  if(target[E_AXIS]!=position[E_AXIS])
    if(degHotend(active_extruder)<EXTRUDE_MINTEMP && !allow_cold_extrude)
    {
      position[E_AXIS]=target[E_AXIS]; //behave as if the move really took place, but ignore E part
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
    }
#ifdef PREVENT_LENGTHY_EXTRUDE
  if(labs(target[E_AXIS]-position[E_AXIS])>axis_steps_per_unit[E_AXIS]*EXTRUDE_MAXLENGTH)
  {
    position[E_AXIS]=target[E_AXIS]; //behave as if the move really took place, but ignore E part
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
  }
#endif
#endif

  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];

  // Mark block as not busy (Not executed by the stepper interrupt)
  block->busy = false;

#ifdef ORIGINAL_PLANNER
	#undef ORIGINAL_PLANNER_DELTA
	#define ORIGINAL_PLANNER_DELTA  // must use original delta

 // Number of steps for each axis
  block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
  block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);
  block->steps_z = labs(target[Z_AXIS]-position[Z_AXIS]);
  block->steps_e = labs(target[E_AXIS]-position[E_AXIS]);
  block->steps_e *= extrudemultiply;
  block->steps_e /= 100;
  block->step_event_count = max(block->steps_x, max(block->steps_y, max(block->steps_z, block->steps_e)));

  // Bail if this is a zero-length block
  if (block->step_event_count <= dropsegments) { 
    return; 
  };

  block->fan_speed = FanSpeed;

  // Compute direction bits for this block 
  block->direction_bits = 0;
  if (target[X_AXIS] < position[X_AXIS]) { 
    block->direction_bits |= (1<<X_AXIS); 
  }
  if (target[Y_AXIS] < position[Y_AXIS]) { 
    block->direction_bits |= (1<<Y_AXIS); 
  }
  if (target[Z_AXIS] < position[Z_AXIS]) { 
    block->direction_bits |= (1<<Z_AXIS); 
  }
  if (target[E_AXIS] < position[E_AXIS]) { 
    block->direction_bits |= (1<<E_AXIS); 
  }

#else

  // Number of steps for each axis
  block->steps_x = target[X_AXIS]-position[X_AXIS];
  block->steps_y = target[Y_AXIS]-position[Y_AXIS];
  block->steps_z = target[Z_AXIS]-position[Z_AXIS];
  block->steps_e = target[E_AXIS]-position[E_AXIS];
  block->steps_e *= extrudemultiply;
  block->steps_e /= 100;

#ifdef ORIGINAL_PLANNER_DELTA
#else
  // use the current state of the calcs to set up the correct deltas, instead  of repeating essentially the same thing later
  delta_mm[X_AXIS] = block->steps_x / axis_steps_per_unit[X_AXIS];
  delta_mm[Y_AXIS] = block->steps_y / axis_steps_per_unit[Y_AXIS];
  delta_mm[Z_AXIS] = block->steps_z / axis_steps_per_unit[Z_AXIS];
  delta_mm[E_AXIS] = block->steps_e / axis_steps_per_unit[E_AXIS];
#endif
 
  // Compute direction bits for this block and make steps absolute
  block->direction_bits = 0;
  if ( block->steps_x < 0 ) { 
    block->direction_bits |= (1<<X_AXIS); 
	 block->steps_x = -block->steps_x;
  }
  if ( block->steps_y < 0 ) { 
    block->direction_bits |= (1<<Y_AXIS); 
	 block->steps_y = -block->steps_y;
  }
  if ( block->steps_z < 0 ) { 
    block->direction_bits |= (1<<Z_AXIS); 
	 block->steps_z = -block->steps_z;
  }
  if ( block->steps_e < 0 ) { 
    block->direction_bits |= (1<<E_AXIS); 
	 block->steps_e = -block->steps_e;
  }

  //block->step_event_count = max(block->steps_x, max(block->steps_y, max(block->steps_z, block->steps_e)));
  block->step_event_count = max(block->steps_z, max(block->steps_e, max(block->steps_x, block->steps_y))); // may be slightly faster to evaluate the xy first (deepest part of expression)

  // Bail if this is a zero-length block
  if ( block->step_event_count <= dropsegments) { 
	  return; 
	  };
	  
  block->fan_speed = FanSpeed;

#endif

   
//#define TRACE_PLANNING
#ifdef TRACE_PLANNING
  const uint32_t MAX_PLAUISIBLE_BLOCK_STEP_COUNT = max( X_MAX_LENGTH * axis_steps_per_unit[X_AXIS], Y_MAX_LENGTH * axis_steps_per_unit[Y_AXIS] );
  if ( block->step_event_count > MAX_PLAUISIBLE_BLOCK_STEP_COUNT ) {
	  // NOTE: homing events can trigger this
	  SERIAL_ECHO_START;
	  SERIAL_ECHOPGM("impossible #steps:");  
	  SERIAL_ECHO(   block->step_event_count );

	  SERIAL_ECHOPGM(" mm x:");
	  SERIAL_ECHO( x );
	  SERIAL_ECHOPGM(" y:");
	  SERIAL_ECHO( y );
	  SERIAL_ECHOPGM(" z:");
	  SERIAL_ECHO( z );
	  SERIAL_ECHOPGM(" e:");
	  SERIAL_ECHOLN( e );

	  SERIAL_ECHOPGM(" steps x:");
	  SERIAL_ECHO( block->steps_x );
	  SERIAL_ECHOPGM(" y:");
	  SERIAL_ECHO( block->steps_y );
	  SERIAL_ECHOPGM(" z:");
	  SERIAL_ECHO( block->steps_z );
	  SERIAL_ECHOPGM(" e:");
	  SERIAL_ECHOLN( block->steps_e );

	  SERIAL_ECHOPGM(" pos x:");
	  SERIAL_ECHO( position[ X_AXIS ] );
	  SERIAL_ECHOPGM(" y:");
	  SERIAL_ECHO( position[ Y_AXIS ] );
	  SERIAL_ECHOPGM(" z:");
	  SERIAL_ECHO( position[ Z_AXIS ] );
	  SERIAL_ECHOPGM(" e:");
	  SERIAL_ECHOLN( position[ E_AXIS ] );

  }
#endif



  block->active_extruder = extruder;

  //enable active axes
  if(block->steps_x != 0) enable_x();
  if(block->steps_y != 0) enable_y();
#ifndef Z_LATE_ENABLE
  if(block->steps_z != 0) enable_z();
#endif

  // Enable all
  if(block->steps_e != 0) { 
    enable_e0();
    enable_e1();
    enable_e2(); 
  }

  if (block->steps_e == 0) {
    if(feed_rate<mintravelfeedrate) feed_rate=mintravelfeedrate;
  }
  else {
    if(feed_rate<minimumfeedrate) feed_rate=minimumfeedrate;
  } 

  float delta_mm[4];
  #ifdef ORIGINAL_PLANNER_DELTA
  delta_mm[X_AXIS] = (target[X_AXIS]-position[X_AXIS])/axis_steps_per_unit[X_AXIS];
  delta_mm[Y_AXIS] = (target[Y_AXIS]-position[Y_AXIS])/axis_steps_per_unit[Y_AXIS];
  delta_mm[Z_AXIS] = (target[Z_AXIS]-position[Z_AXIS])/axis_steps_per_unit[Z_AXIS];
  delta_mm[E_AXIS] = ((target[E_AXIS]-position[E_AXIS])/axis_steps_per_unit[E_AXIS])*extrudemultiply/100.0;
  #else
	// done above
  #endif
  
  #ifdef ORIGINAL_PLANNER_SQRT
  
  if ( block->steps_x <=dropsegments && block->steps_y <=dropsegments && block->steps_z <=dropsegments ) {
    block->millimeters = fabs(delta_mm[E_AXIS]);
  } 
  else {
    block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS]));
  }
  float inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple divides 

  #else
  
 
  bool bIsOnlyExtrusion = false;
#define POW2( x ) ( x * x )
  if ( block->steps_x <= dropsegments && block->steps_y <= dropsegments && block->steps_z <= dropsegments ) {
    block->millimeters = fabs(delta_mm[E_AXIS]);
	 bIsOnlyExtrusion = true;
  } 
  else {
	  // try to eliminate a lot of the square roots. For mechanical parts there are a lot of X-only and Y-only moves and they don't need square roots.
	  if ( block->steps_x <= dropsegments ) {
		  // no X move
		  if ( block->steps_y <= dropsegments ) {
			  // no Y either
			  block->millimeters = fabs( delta_mm[Z_AXIS] );
		  }
		  else {
			  // no X but Y and Z. Very rare. OK to use sqrt.
			  if ( block->steps_z <= dropsegments) {
				  // no X nor Z
				  block->millimeters = fabs( delta_mm[Y_AXIS] );
				  }
			  else {
				  block->millimeters = sqrt( POW2(delta_mm[Y_AXIS]) + POW2(delta_mm[Z_AXIS]) );
			  }
		  }
	  }
	  else {
			// X does move
		  if ( block->steps_y <= dropsegments ) {
			  // but not Y
			  if ( block->steps_z <= dropsegments ) {
				  // or Z
				  // it's just X
				  block->millimeters = fabs( delta_mm[X_AXIS] );
				}
			  else {
				  // X and Z only (rare)
				  block->millimeters = sqrt( POW2(delta_mm[X_AXIS]) + POW2(delta_mm[Z_AXIS]) );
			  }
		  }
		  else {
			  // X and Y move, maybe Z
			  block->millimeters = sqrt( POW2(delta_mm[X_AXIS]) + POW2(delta_mm[Y_AXIS]) + POW2(delta_mm[Z_AXIS]) );
		  }
	  }


    //block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS]));
  }

  float inverse_millimeters = 1.0f / block->millimeters;  // Inverse millimeters to remove multiple divides 
#endif

    // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
  float inverse_second = feed_rate * inverse_millimeters;

  int moves_queued=(block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);

  // slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill

#ifdef ORIGINAL_PLANNER
#ifdef OLD_SLOWDOWN
  if(moves_queued < (BLOCK_BUFFER_SIZE * 0.5) && moves_queued > 1) feed_rate = feed_rate*moves_queued / (BLOCK_BUFFER_SIZE * 0.5); 
#endif

#ifdef SLOWDOWN
  //  segment time im micro seconds
  unsigned long segment_time = lround(1000000.0/inverse_second);
  if ((moves_queued > 1) && (moves_queued < (BLOCK_BUFFER_SIZE * 0.5))) {
    if (segment_time < minsegmenttime)  { // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
      inverse_second=1000000.0/(segment_time+lround(2*(minsegmenttime-segment_time)/moves_queued));
    }
  }
#endif
#else

#ifdef SLOWDOWN

 #ifdef OLD_SLOWDOWN

  #ifdef XY_FREQUENCY_LIMIT
  #else
  //  segment time in micro seconds
  unsigned long segment_time = lround(1000000.0/inverse_second);
  #endif

  if(moves_queued < (BLOCK_BUFFER_SIZE * 0.5) && moves_queued > 1) feed_rate = feed_rate*moves_queued / (BLOCK_BUFFER_SIZE * 0.5); 
  
  if (segment_time < minsegmenttime)  { // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
	  inverse_second=1000000.0/(segment_time+lround(2*(minsegmenttime-segment_time)/moves_queued));
  }

 #else

  //const float BUFFER_DRAIN_SLOWDOWN_THRESHOLD_DIVISOR = 2;
  //if ( (moves_queued > 1) &&  ( moves_queued < ( BLOCK_BUFFER_SIZE / BUFFER_DRAIN_SLOWDOWN_THRESHOLD_DIVISOR ) ) ) {
  const float BUFFER_DRAIN_SLOWDOWN_THRESHOLD_MOVES = 3;   // we'd like three moves in the buffer all the time
  
  if ( (moves_queued > 1) &&  ( moves_queued < BUFFER_DRAIN_SLOWDOWN_THRESHOLD_MOVES ) ) {
  #ifdef XY_FREQUENCY_LIMIT
  #else
	  //  segment time in micro seconds
	  unsigned long segment_time = lround(1000000.0/inverse_second);
  #endif
     if (segment_time < minsegmenttime)  { // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
        inverse_second=1000000.0/(segment_time+lround(2*(minsegmenttime-segment_time)/moves_queued));
     }
  }

 #endif

#endif
  //  END OF SLOW DOWN SECTION    
  
#endif  


  block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0

  // Calculate and limit speed in mm/sec for each axis
  float cur_nominal_axis_speed[4];
  float cur_nominal_axis_absSpeed[4];
  float speed_factor = 1.0; //factor <=1 do decrease speed
  for(int i=0; i < 4; i++) {
    cur_nominal_axis_speed[i] = delta_mm[i] * inverse_second;
	 cur_nominal_axis_absSpeed[i] = fabs( cur_nominal_axis_speed[i] );
	 if ( cur_nominal_axis_absSpeed[i] > max_feedrate[i] ) {
      speed_factor = min( speed_factor, max_feedrate[i] / cur_nominal_axis_absSpeed[i] );
	 }
  }

  // Max segement time in us.
#ifdef XY_FREQUENCY_LIMIT
#define MAX_FREQ_TIME (1000000.0/XY_FREQUENCY_LIMIT)

  // Check and limit the xy direction change frequency
  unsigned char direction_change = block->direction_bits ^ old_direction_bits;
  old_direction_bits = block->direction_bits;

  if((direction_change & (1<<X_AXIS)) == 0) {
    x_segment_time[0] += segment_time;
  }
  else {
    x_segment_time[2] = x_segment_time[1];
    x_segment_time[1] = x_segment_time[0];
    x_segment_time[0] = segment_time;
  }
  if((direction_change & (1<<Y_AXIS)) == 0) {
    y_segment_time[0] += segment_time;
  }
  else {
    y_segment_time[2] = y_segment_time[1];
    y_segment_time[1] = y_segment_time[0];
    y_segment_time[0] = segment_time;
  }
  long max_x_segment_time = max(x_segment_time[0], max(x_segment_time[1], x_segment_time[2]));
  long max_y_segment_time = max(y_segment_time[0], max(y_segment_time[1], y_segment_time[2]));
  long min_xy_segment_time =min(max_x_segment_time, max_y_segment_time);
  if(min_xy_segment_time < MAX_FREQ_TIME) speed_factor = min(speed_factor, speed_factor * (float)min_xy_segment_time / (float)MAX_FREQ_TIME);
#endif

  // Correct the speed  
  if( speed_factor < 1.0) {
    for(unsigned char i=0; i < 4; i++) {
      cur_nominal_axis_speed[i] *= speed_factor;
		cur_nominal_axis_absSpeed[i] = fabs( cur_nominal_axis_speed[i] );
    }
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.  
  float steps_per_mm = block->step_event_count/block->millimeters;
#ifdef ORIGINAL_PLANNER_SQRT
  if(block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0) {
#else
  if ( bIsOnlyExtrusion ) {
#endif  
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  else {
    block->acceleration_st = ceil(acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
    // Limit acceleration per axis
    if(((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > axis_steps_per_sqr_second[X_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_e / (float)block->step_event_count) > axis_steps_per_sqr_second[E_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_z / (float)block->step_event_count ) > axis_steps_per_sqr_second[Z_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
  }
  block->acceleration = block->acceleration_st / steps_per_mm;
  block->acceleration_rate = (long)((float)block->acceleration_st * 8.388608);

#if 0  // Use old jerk for now
  // Compute path unit vector
  double unit_vec[3];

  unit_vec[X_AXIS] = delta_mm[X_AXIS]*inverse_millimeters;
  unit_vec[Y_AXIS] = delta_mm[Y_AXIS]*inverse_millimeters;
  unit_vec[Z_AXIS] = delta_mm[Z_AXIS]*inverse_millimeters;

  // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
  // Let a circle be tangent to both previous and current path line segments, where the junction
  // deviation is defined as the distance from the junction to the closest edge of the circle,
  // colinear with the circle center. The circular segment joining the two paths represents the
  // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
  // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
  // path width or max_jerk in the previous grbl version. This approach does not actually deviate
  // from path, but used as a robust way to compute cornering speeds, as it takes into account the
  // nonlinearities of both the junction angle and junction velocity.
  double vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed

  // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
  if ((block_buffer_head != block_buffer_tail) && (previous_nominal_speed > 0.0)) {
    // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
    // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
    double cos_theta = - previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
      - previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
      - previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;

    // Skip and use default max junction speed for 0 degree acute junction.
    if (cos_theta < 0.95) {
      vmax_junction = min(previous_nominal_speed,block->nominal_speed);
      // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
      if (cos_theta > -0.95) {
        // Compute maximum junction velocity based on maximum acceleration and junction deviation
        double sin_theta_d2 = sqrt(0.5*(1.0-cos_theta)); // Trig half angle identity. Always positive.
        vmax_junction = min(vmax_junction,
        sqrt(block->acceleration * junction_deviation * sin_theta_d2/(1.0-sin_theta_d2)) );
      }
    }
  }
#endif
  
#ifdef ORIGINAL_PLANNER
  // Start with a safe speed
  float vmax_junction = max_xy_jerk/2; 
  float vmax_junction_factor = 1.0; 
  if(fabs(cur_nominal_axis_speed[Z_AXIS]) > max_z_jerk/2) 
    vmax_junction = min(vmax_junction, max_z_jerk/2);
  if(fabs(cur_nominal_axis_speed[E_AXIS]) > max_e_jerk/2) 
    vmax_junction = min(vmax_junction, max_e_jerk/2);
  vmax_junction = min(vmax_junction, block->nominal_speed);
  float safe_exit_speed = vmax_junction;

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float jerk = sqrt(pow((cur_nominal_axis_speed[X_AXIS]-previous_speed[X_AXIS]), 2)+pow((cur_nominal_axis_speed[Y_AXIS]-previous_speed[Y_AXIS]), 2));
    //    if((fabs(previous_speed[X_AXIS]) > 0.0001) || (fabs(previous_speed[Y_AXIS]) > 0.0001)) {
    vmax_junction = block->nominal_speed;
    //    }
    if (jerk > max_xy_jerk) {
      vmax_junction_factor = (max_xy_jerk/jerk);
    } 
    if(fabs(cur_nominal_axis_speed[Z_AXIS] - previous_speed[Z_AXIS]) > max_z_jerk) {
      vmax_junction_factor= min(vmax_junction_factor, (max_z_jerk/fabs(cur_nominal_axis_speed[Z_AXIS] - previous_speed[Z_AXIS])));
    } 
    if(fabs(cur_nominal_axis_speed[E_AXIS] - previous_speed[E_AXIS]) > max_e_jerk) {
      vmax_junction_factor = min(vmax_junction_factor, (max_e_jerk/fabs(cur_nominal_axis_speed[E_AXIS] - previous_speed[E_AXIS])));
    } 
    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
  }
  block->max_entry_speed = vmax_junction;


#else
  // the end result of all this pre-planning is a speed and a pair of scaling factors.
  // dead simple jerk - allocate half to this block, a theoretical half to any adjoining block
  // we have no idea whether the next block is a movement that even involves the master axis for this move.
  // It's quite possible that this block's master has zero velocity on the next block, and it's also possible
  // that it has exactly the opposite direction in the same axis, hence only being able to specify half the nominal jerk value


  float exit_scaling = 1.0f;
  // set up default entry and exit parameters too
  for ( uint8_t a=0; a < NUM_STEPPERS; ++a ) {
	  // assume this is the only block
	  float abs_speed_diff = abs( cur_nominal_axis_speed[a] );

	  // we should be able to enter and leave with full jerk, because normally any adjoining block will look at the direction and speed we leave at.
	  // If we're setting the safe default entry scaling from this, then that won't be true, so make sure we default
	  // the entry scaling to zero-ish because it's probably better to leave this block with some velocity for the next block
	  // to plan with, than hog it all on our entry and leave the next block with nothing.
	  // It also results in the proper calculation of the max allowed speed.
	  float jerk_limit = max_axis_jerk[a];
	  if ( abs_speed_diff > jerk_limit ) { 
		  float axis_jerk_factor = jerk_limit / abs_speed_diff;
		  exit_scaling = min( exit_scaling, axis_jerk_factor );
		  }
	  }

  float min_exit_scaling = MINIMUM_PLANNER_SPEED / block->nominal_speed;
  exit_scaling = max( exit_scaling, min_exit_scaling );
  float safe_exit_speed = exit_scaling * block->nominal_speed;  

  float entry_scaling   = 0;
  float best_entry_scaling = entry_scaling;


#ifdef ALLOW_JERK
  // Jerk tries to set the highest allowable entry speed. We can't do anything about exit speed here.
 #ifdef ALTERNATIVE_JERK
  #ifdef DEAD_SIMPLE_JERK
	// keep the defaulted values above
  #else

  // if this is the sole block in the queue, there's not much we can do for it.
  if ( moves_queued == 0 ) {
	  // same as DEAD_SIMPLE_JERK
  } else  {
	  // re-default - we are going to try to do a better job than the defaults
	  entry_scaling = 1;
	  best_entry_scaling = 1;

		// needs to be planned out according to what the axes can tolerate
	  // Plus we have the advantage of looking at what the previous block's speeds are
		// NOTE: previous algorithms have absolutely counted on a fresh block being re-planned
	  //        and have set an entry speed here based on the jerk relative to the previous block's nominal speed.
	  //        But we really have no way to know for sure that it will not already have been scheduled for execution
	  //        by the time we can re-plan this block.
	  //        So the only really safe thing to do is to tentatively plan for the worst case, and make note of the best case 
	  //        that we can aim for in the re-planning.
	  for ( uint8_t a=0; a < NUM_STEPPERS; ++a ) {
		  // change scaling to if needed to limit the jerk on this axis
		  // The trouble is, we can't just assume that the previous block has been planned to exit at its nominal speed. 
		  // We must assume that it will be left with its exit speed as it is currently planned. Since it knows nothing about us at this moment,
		  // it must have planned an exit speed by using this very code, and we have left that exit value recorded as previous_exit_speed.
		  // There's still no guarantee that this and the previous block have not been so very short that
		  // two (or any number) of legal jerks in quick succession will not add up to an unacceptable total jerk. But perhaps that can be covered
		  // by managing it some other way. (See if nominal is higher? Check length of this segment, set exit even lower if really short?)
		  float abs_speed_diff = abs( cur_nominal_axis_speed[a] - previous_exit_speed[a] );
		  float jerk_limit =  max_axis_jerk[a];
		  if ( abs_speed_diff > jerk_limit ) {
			  // if the previous speed was the opposite sign, our needed scaling is actually potentially zero
			  // allowable speed:
			  float axis_jerk_factor = jerk_limit / abs_speed_diff; // assume same direction  - very likely
			  // but check for a change in direction
			  bool oppositeDirections = ( cur_nominal_axis_speed[a] * previous_exit_speed[a] < 0 );
			  if ( oppositeDirections ) {
			   // do an absolute calculation of the allowed positive speed we can enter with, if the
			   // previous exit speed is taken as negative, and we add the allowed jerk to it.
				float jerk_limited_speed_our_direction = 0 - abs( previous_exit_speed[a] ) + max_axis_jerk[a];
				// now we can determine a scaling factor, watching out for whether it really wants to be negative (which we will not handle)
				if ( jerk_limited_speed_our_direction >= 0  ) {
					axis_jerk_factor = jerk_limited_speed_our_direction / abs( cur_nominal_axis_speed[a] );
				}
				else {
#ifdef DEBUG_VARS
					if ( extruder_debug_i > 0 ) {
						SERIAL_ECHO_START;
						SERIAL_ECHOPGM("clamped jerk, axis:");
						SERIAL_ECHO( a );
						SERIAL_ECHOPGM(" oppositeDirections:");
						SERIAL_ECHOLN( oppositeDirections ? "yes" : "no" );
					}
#endif
					axis_jerk_factor = 0; // nothing we can do about it.
				}

			 }
#ifdef DEBUG_VARS
			  if ( extruder_debug_i > 0 ) {
				  SERIAL_ECHO_START;
				  SERIAL_ECHOPGM(" axis:");
				  SERIAL_ECHO( a );
				  SERIAL_ECHOPGM(" jerk_limit:");
				  SERIAL_ECHO( jerk_limit );
				  SERIAL_ECHOPGM(" previous_exit_speed[a]:");
				  SERIAL_ECHO( previous_exit_speed[a] );
				  SERIAL_ECHOPGM(" abs_speed_diff:");
				  SERIAL_ECHO( abs_speed_diff );
				  SERIAL_ECHOPGM(" axis_jerk_factor:");
				  SERIAL_ECHOLN( axis_jerk_factor );
			  }
#endif
			 entry_scaling = min( entry_scaling, axis_jerk_factor );

		  }

		  // do a best-case scenario too so the re-planning process can aim for this.
		  // This is not the end of the story, though, because there is still another calculation of the max entry based on decelerating to the end speed in the block distance
		  float best_abs_speed_diff = abs( cur_nominal_axis_speed[a] - previous_speed[a] );
		  if ( best_abs_speed_diff > jerk_limit ) {
			  float axis_jerk_factor = jerk_limit / best_abs_speed_diff;
			  // if the previous speed was the opposite sign, our needed scaling is actually potentially zero
			  bool oppositeDirections = ( cur_nominal_axis_speed[a] * previous_speed[a] < 0 );
			  if ( oppositeDirections ) {
				  // do an absolute calculation of the allowed positive speed we can enter with, if the
				  // previous nominal speed is taken as negative, and we add the allowed jerk to it.
				  float jerk_limited_speed_our_direction = 0 - abs( previous_speed[a] ) + max_axis_jerk[a];
				  // now we can determine a scaling factor, watching out for whether it really wants to be negative (which we will not handle)
				  if ( jerk_limited_speed_our_direction >= 0  ) {
					  axis_jerk_factor = jerk_limited_speed_our_direction / abs( cur_nominal_axis_speed[a] );
				  }
				  else {
					  axis_jerk_factor = 0; // nothing we can do about it.
				  }
			  }

		     best_entry_scaling = min( best_entry_scaling, axis_jerk_factor );
		  }

	  }

  }

  float vmax_junction = block->nominal_speed * entry_scaling;

  #endif
#else
  // this seems to give terrible results.
  float vmax_junction = max_xy_jerk/2; 
  float vmax_junction_factor = 1.0; 

  if(fabs(cur_nominal_axis_speed[Z_AXIS]) > max_z_jerk/2) 
    vmax_junction = min(vmax_junction, max_z_jerk/2);

  if(fabs(cur_nominal_axis_speed[E_AXIS]) > max_e_jerk/2) 
    vmax_junction = min(vmax_junction, max_e_jerk/2);

  vmax_junction = min(vmax_junction, block->nominal_speed);
  safe_exit_speed = vmax_junction;

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float jerk = sqrt(pow((cur_nominal_axis_speed[X_AXIS]-previous_speed[X_AXIS]), 2)+pow((cur_nominal_axis_speed[Y_AXIS]-previous_speed[Y_AXIS]), 2));
    //    if((fabs(previous_speed[X_AXIS]) > 0.0001) || (fabs(previous_speed[Y_AXIS]) > 0.0001)) {
    vmax_junction = block->nominal_speed;
    //    }
    if (jerk > max_xy_jerk) {
      vmax_junction_factor = (max_xy_jerk/jerk);
    } 
    if(fabs(cur_nominal_axis_speed[Z_AXIS] - previous_speed[Z_AXIS]) > max_z_jerk) {
      vmax_junction_factor= min(vmax_junction_factor, (max_z_jerk/fabs(cur_nominal_axis_speed[Z_AXIS] - previous_speed[Z_AXIS])));
    } 
    if(fabs(cur_nominal_axis_speed[E_AXIS] - previous_speed[E_AXIS]) > max_e_jerk) {
      vmax_junction_factor = min(vmax_junction_factor, (max_e_jerk/fabs(cur_nominal_axis_speed[E_AXIS] - previous_speed[E_AXIS])));
    } 
    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
	 entry_scaling = vmax_junction_factor;
  }
 #endif

  block->entry_speed = vmax_junction;

#else
   // no Jerk at all
	float vmax_junction = MINIMUM_PLANNER_SPEED;
	float safe_exit_speed = vmax_junction;
#endif

#endif // not ORIGINAL_PLANNER

#ifdef ORIGINAL_PLANNER
  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  double v_allowable = max_allowable_speed(-block->acceleration,MINIMUM_PLANNER_SPEED,block->millimeters);
  block->entry_speed = min( vmax_junction, v_allowable ); // limit the entry speed to the deceleration limit
#else
	// Initialize block entry speed. Limit based on deceleration to safe_exit_speed.
	double v_allowable = max_allowable_speed( -block->acceleration, safe_exit_speed, block->millimeters );

	block->entry_speed = min( vmax_junction, v_allowable ); // limit the entry speed to the deceleration limit

	block->best_entry_speed = best_entry_scaling * block->nominal_speed;
	block->best_entry_speed = min( block->best_entry_speed, v_allowable ); // limit the entry speed to the deceleration limit

	block->max_entry_speed = block->best_entry_speed;  // see if this is really what's meant

	// adjust entry scaling
	entry_scaling = block->entry_speed / block->nominal_speed;
#endif

#ifdef trace_unachievables
	bTrapezoidDetectedError = 0;
#endif

#ifdef ORIGINAL_PLANNER
	// this was originally done well below here but should work out to the same thing
	calculate_trapezoid_for_block( block, block->entry_speed / block->nominal_speed,  safe_exit_speed / block->nominal_speed );

#else
	calculate_trapezoid_for_block( block, entry_scaling, exit_scaling );


#ifdef trace_unachievables
	if ( bTrapezoidDetectedError != 0 
#ifdef DEBUG_VARS
		||  extruder_debug_i > 0 
#endif
		) {
		SERIAL_ECHO_START;
		if ( bTrapezoidDetectedError != 0 ) SERIAL_ECHOPGM("bTrapezoidDetectedError ");
		SERIAL_ECHOPGM(" entry:");
		SERIAL_ECHO(block->entry_speed); 
		SERIAL_ECHOPGM(" max entry:");
		SERIAL_ECHO(block->max_entry_speed); 
		SERIAL_ECHOPGM(" best entry:");
		SERIAL_ECHO(block->best_entry_speed); 
		SERIAL_ECHOPGM(" v_allowable:");
		SERIAL_ECHO( v_allowable ); 
		SERIAL_ECHOPGM(" jun:");
		SERIAL_ECHO(vmax_junction);
		SERIAL_ECHOPGM(" nom:");
		SERIAL_ECHO(block->nominal_speed);


		SERIAL_ECHOPGM(" entry_scaling:");
		SERIAL_ECHO(entry_scaling); 
		SERIAL_ECHOPGM(" exit_scaling:");
		SERIAL_ECHO(exit_scaling); 
		SERIAL_ECHOPGM(" exit rate:");
		SERIAL_ECHO( block->final_rate ); 

		SERIAL_ECHO(" safe_exit_speed:"); 
		SERIAL_ECHOLN(safe_exit_speed);
	}
#endif
#endif

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  if (block->nominal_speed <= v_allowable) { 
    block->nominal_length_flag = true; 
  }
  else { 
    block->nominal_length_flag = false; 
  }
  block->recalculate_flag = true; // Always calculate trapezoid for new block

  // check some assumptions
  if ( block->entry_speed > block->nominal_speed 
	  || safe_exit_speed > block->nominal_speed ) {
		  SERIAL_ECHO_START;
		  SERIAL_ECHOPGM("block speed problem entry:");
		  SERIAL_ECHO(block->entry_speed); 
		  SERIAL_ECHOPGM(" max entry:");
		  SERIAL_ECHO(block->max_entry_speed); 
		  SERIAL_ECHOPGM(" jun:");
		  SERIAL_ECHO(vmax_junction);
		  SERIAL_ECHOPGM(" nom:");
		  SERIAL_ECHO(block->nominal_speed);
		  SERIAL_ECHOPGM(" exit:");
		  SERIAL_ECHOLN(safe_exit_speed);
	  }


  // Update previous path unit_vector and nominal speed
  memcpy(previous_speed, cur_nominal_axis_speed, sizeof(previous_speed)); // previous_speed[] = cur_nominal_axis_speed[]
  previous_nominal_speed = block->nominal_speed;


#ifdef EXTRUDER_ADVANCE
  block->advance_rate   = 0;
  block->unadvance_rate = 0;
  block->initial_advance = 0;
  block->final_advance = 0;

  // Calculate advance 
  if ( extruder_advance_k == 0 || (block->steps_e == 0) || bIsOnlyExtrusion ) {
    block->advance = 0;
  }
  else {
	 //float advance = (STEPS_PER_CUBIC_MM_E * EXTRUDER_ADVANCE_K) * (cur_nominal_axis_speed[E_AXIS] * cur_nominal_axis_speed[E_AXIS] * EXTRUSION_AREA * EXTRUSION_AREA ) * 256;
 #ifdef ADVANCE_WITH_SQUARE_LAW
	 float advance_stepsx256 = (STEPS_PER_CUBIC_MM_E * extruder_advance_k) * ( cur_nominal_axis_speed[E_AXIS] * cur_nominal_axis_speed[E_AXIS] * EXTRUSION_AREA ) * 256;
 #else
	 // it seems to over-advance if we use the square of the speed - higher speeds needed lower constants, by a large factor
	 float advance_stepsx256 = (STEPS_PER_CUBIC_MM_E * extruder_advance_k) * ( cur_nominal_axis_speed[E_AXIS] * EXTRUSION_AREA ) * 256;
 #endif
    block->advance = advance_stepsx256;

#ifdef FLOAT_ESTIMATES
	 long acc_dist = estimate_acceleration_distance( 0, block->nominal_rate, block->acceleration_st );
	 if ( acc_dist > 0 ) {
		 // this is a differential advance rate, only correct if the initial speed was zero. 
		 block->advance_rate   = advance_stepsx256 / (float)acc_dist;		// spread the advance evenly across the entire acceleration distance
		 block->unadvance_rate = block->advance_rate;
	 }
#else

	 // do a nominal calculation - if it's replanned they will be revised
	 if ( block->acceleration_st > 0 ) {
		 uint32_t acc_dist = estimate_acceleration_distance_from0( block->nominal_rate, block->acceleration_st );
		 if ( acc_dist != 0 ) {
			// this is a differential advance rate, only correct if the initial speed was zero. 
			block->advance_rate   = advance_stepsx256 / acc_dist;		// spread the advance evenly across the entire acceleration distance
			block->unadvance_rate = block->advance_rate;
		  }

     }
#endif

  }
//#define TRACE_ADVANCE  
 #ifdef TRACE_ADVANCE
  SERIAL_ECHO_START;
   SERIAL_ECHOPGM("i adv:");
   SERIAL_ECHO(block->advance);

   SERIAL_ECHOPGM(" nom:");
	SERIAL_ECHO(block->nominal_rate);

	//SERIAL_ECHOPGM(" acc:");
	//SERIAL_ECHO(block->acceleration_st);

	SERIAL_ECHOPGM(" bldist:");
	SERIAL_ECHO(block->millimeters);

	SERIAL_ECHOPGM( " steps:" );
	SERIAL_ECHO(     block->step_event_count );

	SERIAL_ECHOPGM( " steps_e:" );
	SERIAL_ECHO(      block->steps_e );

	SERIAL_ECHOPGM( " e/mm:" );
	SERIAL_ECHO(      (float)block->steps_e/(float)block->millimeters );

	//SERIAL_ECHOPGM(" accdist:");
	//{
	//long acc_distx = estimate_acceleration_distance( 0, block->nominal_rate, block->acceleration_st );
	//SERIAL_ECHO(acc_distx);
	//}

	SERIAL_ECHOPGM( " advrate:" );
   SERIAL_ECHOLN(    block->advance_rate);
 #endif
#endif // EXTRUDER_ADVANCE


  // Move buffer head
  block_buffer_head = next_buffer_head;

  // Update position
  memcpy(position, target, sizeof(target)); // position[] = target[]

#ifdef RECALCULATE_PLANNER
#ifdef DEBUG_VARS  
  if ( ! ( extruder_debug_k > 0  ) ) {  // setting k will TURN OFF re-planning.
#endif
  planner_recalculate();
#ifdef DEBUG_VARS  
  }
#endif
#endif
  st_wake_up();
}

void plan_set_position(const float &x, const float &y, const float &z, const float &e)
{
  position[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
  position[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
  position[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);     
  position[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);  
  st_set_position(position[X_AXIS], position[Y_AXIS], position[Z_AXIS], position[E_AXIS]);

  clear_planning_vars(); // Resets planner junction speeds. Assumes start from rest.
}

void plan_set_e_position(const float &e)
{
  position[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);  
  st_set_e_position(position[E_AXIS]);
}

uint8_t movesplanned()
{
  return (block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
}

void allow_cold_extrudes(bool allow)
{
#ifdef PREVENT_DANGEROUS_EXTRUDE
  allow_cold_extrude=allow;
#endif
}

