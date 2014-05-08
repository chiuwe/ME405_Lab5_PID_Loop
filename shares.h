//*************************************************************************************
/** \file shares.h
 *    This file contains extern declarations for queues and other inter-task data
 *    communication objects used in a ME405/507/FreeRTOS project. 
 *
 *  Revisions:
 *    \li 09-30-2012 JRR Original file was a one-file demonstration with two tasks
 *    \li 10-05-2012 JRR Split into multiple files, one for each task plus a main one
 *    \li 10-29-2012 JRR Reorganized with global queue and shared data references
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


// This define prevents this .h file from being included multiple times in a .cpp file

#ifndef _SHARES_H_
#define _SHARES_H_

//-------------------------------------------------------------------------------------
// Externs:  In this section, we declare variables and functions that are used in all
// (or at least two) of the files in the data acquisition project. Each of these items
// will also be declared exactly once, without the keyword 'extern', in one .cpp file
// as well as being declared extern here. 

// This queue allows tasks to send characters to the user interface task for display.
extern frt_text_queue* print_ser_queue;

extern shared_data<int32_t>* count;

extern shared_data<int32_t>* error;

extern frt_queue<uint32_t>* p_queue_1;

/*  This shared data item allows a value to be posted by the source task and read by
 *  the sink task.
 */
extern shared_data<uint32_t>* p_share_1;

/*  This shared data item allows a power value to be posted by user task and read by the 
 *  motor task.
 */
extern shared_data<int16_t>* power_1;

/*  This shared data item allows a power value to be posted by user task and read by the 
 *  motor task.
 */
extern shared_data<int16_t>* power_2;

/*  This shared data item allows a brake bool to be posted by user task and read by the 
 *  motor task.
 */
extern shared_data<bool>* brake_1;

/*  This shared data item allows a break bool to be posted by user task and read by the 
 *  motor task.
 */
extern shared_data<bool>* brake_2;

/*  This shared data item determines if motor task reads from the potentiometer. 
 *  Set by user task and read by the motor task.
 */
extern shared_data<bool>* pot_1;

/*  This shared data item determines if motor task reads from the potentiometer. 
 *  Set by user task and read by the motor task.
 */
extern shared_data<bool>* pot_2;

extern shared_data<float>* p_rate_1;

/*
* Knows if the motor is in the right location, or needs to be moved via task_P
*/
extern shared_data<bool>* isCorrectPos;

extern shared_data<int32_t>* correctPos;




#endif // _SHARES_H_
