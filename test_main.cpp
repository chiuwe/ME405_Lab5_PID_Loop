//*************************************************************************************
/** \file test_main.cpp
 *    This file contains the main() code for a program which tests various features of
 *    the ME405/ME507/FreeRTOS software package. Tests include sending data from tasks
 *    to other tasks by several means and checking for errors in transmission; creating
 *    large numbers of tasks and getting them to all run at the same time; and testing
 *    time stamps by measuring the frequency of signals sent by a signal generator. 
 *
 *  Revisions:
 *    \li 09-30-2012 JRR Original file was a one-file demonstration with two tasks
 *    \li 10-05-2012 JRR Split into multiple files, one for each task plus a main one
 *    \li 10-30-2012 JRR A hopefully somewhat stable version with global queue 
 *                       pointers and the new operator used for most memory allocation
 *    \li 11-04-2012 JRR FreeRTOS Swoop demo program changed to a sweet test suite
 *
 *  License:
 *    This file is copyright 2012 by JR Ridgely and released under the Lesser GNU 
 *    Public License, version 2. It intended for educational use only, but its use
 *    is not limited thereto. */
/*    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 *    TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 *    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 *    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 *    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//*************************************************************************************


#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include <avr/wdt.h>                        // Watchdog timer header
#include <string.h>                         // Functions for C string handling

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues
#include "croutine.h"                       // Header for co-routines and such

#include "rs232int.h"                       // ME405/507 library for serial comm.
#include "time_stamp.h"                     // Class to implement a microsecond timer
#include "frt_task.h"                       // Header of wrapper for FreeRTOS tasks
#include "frt_text_queue.h"                 // Wrapper for FreeRTOS character queues
#include "frt_queue.h"                      // Header of wrapper for FreeRTOS queues
#include "frt_shared_data.h"                // Header for thread-safe shared data
#include "shares.h"                         // Global ('extern') queue declarations
#include "motor_driver.h"
#include "task_encoder.h"
#include "task_motor.h"
#include "task_user.h"
#include "task_P.h"



/** This is the number of tasks which will be instantiated from the task_multi class.
 *  These tasks don't to a whole lot except use up processor time and memory space in
 *  order to test how well the program works with lots of tasks.
 */
const uint8_t N_MULTI_TASKS = 4;


// Declare the queues which are used by tasks to communicate with each other here. 
// Each queue must also be declared 'extern' in a header file which will be read 
// by every task that needs to use that queue. The format for all queues except 
// the serial text printing queue is 'frt_queue<type> name (size)', where 'type' 
// is the type of data in the queue and 'size' is the number of items (not neces-
// sarily bytes) which the queue can hold

/** This is a print queue, descended from base_text_serial so that things can be 
*  printed into the queue using the "<<" operator and they'll come out the other
*  end as a stream of characters. It's used by tasks that send things to the
*  user interface task to be printed. 
*/
frt_text_queue* print_ser_queue;

/** This number represents where the motor is rotationally where 4000 is one full rotation
 */
shared_data<int32_t>* count;

/** TODO: This is the number of errors that have occured while monotoring the encoder
 */
shared_data<int32_t>* error;


frt_queue<uint32_t>* p_queue_1;

/** This shared data item allows a value to be posted by the source task and read by
 *  the sink task.
 */
shared_data<uint32_t>* p_share_1;

/** This shared data item allows a power value to be posted by user task and read by the 
 *  motor task.
 */
shared_data<int16_t>* power_1;

/** This shared data item allows a power value to be posted by user task and read by the 
 *  motor task.
 */
shared_data<int16_t>* power_2;

/** This shared data item allows a brake bool to be posted by user task and read by the 
 *  motor task.
 */
shared_data<bool>* brake_1;

/** This shared data item allows a break bool to be posted by user task and read by the 
 *  motor task.
 */
shared_data<bool>* brake_2;

/** This shared data item determines if motor task reads from the potentiometer. 
 *  Set by user task and read by the motor task.
 */
shared_data<bool>* pot_1;

/** This shared data item determines if motor task reads from the potentiometer. 
 *  Set by user task and read by the motor task.
 */
shared_data<bool>* pot_2;

/** This global variable will be written by the source task and read by the sink task.
 *  We expect the process to be corrupted by context switches now and then.
 */
uint32_t* p_glob_of_probs;

/** This shared data item is used by the time rate measurement task to make its
 *  measurements of how fast something is happening available to other tasks.
 */
shared_data<float>* p_rate_1;

shared_data<bool>* isCorrectPos;

shared_data<int32_t>* correctPos;

//=====================================================================================
/** The main function sets up the RTOS.  Some test tasks are created. Then the 
 *  scheduler is started up; the scheduler runs until power is turned off or there's a 
 *  reset.
 *  @return This is a real-time microcontroller program which doesn't return. Ever.
 */

int main (void)
{
	// Disable the watchdog timer unless it's needed later. This is important because
	// sometimes the watchdog timer may have been left on...and it tends to stay on
	MCUSR = 0;
	wdt_disable ();

	// Configure a serial port which can be used by a task to print debugging infor-
	// mation, or to allow user interaction, or for whatever use is appropriate.  The
	// serial port will be used by the user interface task after setup is complete and
	// the task scheduler has been started by the function vTaskStartScheduler()
	rs232 ser_port (9600, 1);
	ser_port << clrscr << PMS ("ME405/FreeRTOS Test Program") << endl;

	// Create the queues and other shared data items here
	print_ser_queue = new frt_text_queue (32, &ser_port, 10);
	count = new shared_data<int32_t>;
	error = new shared_data<int32_t>;
	p_queue_1 = new frt_queue<uint32_t> (20);
	p_share_1 = new shared_data<uint32_t>;
	power_1 = new shared_data<int16_t>;
	power_2 = new shared_data<int16_t>;
	brake_1 = new shared_data<bool>;
	brake_2 = new shared_data<bool>;
	pot_1 = new shared_data<bool>;
	pot_2 = new shared_data<bool>;

	isCorrectPos = new shared_data<bool>;
	correctPos = new shared_data<int32_t>;

	p_glob_of_probs = new uint32_t;

	p_rate_1 = new shared_data<float>;

   // creates two motor tasks.
   motor_driver *p_my_motor_driver1 = new motor_driver(&ser_port, &DDRC, 0x07, &DDRB, 0x40, &PORTC, 0x04, &TCCR1A, 0xA9, &TCCR1B, 0x0B, &OCR1B);
   motor_driver *p_my_motor_driver2 = new motor_driver(&ser_port, &DDRD, 0xE0, &DDRB, 0x20, &PORTD, 0x80, &TCCR1A, 0xA9, &TCCR1B, 0x0B, &OCR1A);

   new task_P ("P1", tskIDLE_PRIORITY + 1, 240, &ser_port, p_my_motor_driver1);
   
   // creates two motor tasks.
   new task_motor ("Motor1", tskIDLE_PRIORITY + 1, 240, 3, p_my_motor_driver1, brake_1, power_1, pot_1, 1, &ser_port);
   new task_motor ("Motor2", tskIDLE_PRIORITY + 1, 240, 4, p_my_motor_driver2, brake_2, power_2, pot_2, 0, &ser_port);

	// The user interface is at low priority; it could have been run in the idle task
	// but it is desired to exercise the RTOS more thoroughly in this test program.

   new task_encoder ("Encoder1", tskIDLE_PRIORITY + 1, 240, &ser_port, PE4, 0b01010101);
   new task_encoder ("Encoder2", tskIDLE_PRIORITY + 1, 240, &ser_port, PE5, 0b01010101);


	new task_user ("UserInt", tskIDLE_PRIORITY + 1, 240, &ser_port);

	// Print an empty line so that there's space between task hellos and help message
	ser_port << endl;


	// Here's where the RTOS scheduler is started up. It should never exit as long as
	// power is on and the microcontroller isn't rebooted
	vTaskStartScheduler ();
}

