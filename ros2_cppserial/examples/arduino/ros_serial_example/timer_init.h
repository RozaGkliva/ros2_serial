#ifndef TIMER_INIT_H
  #define TIMER_INIT_H

/*
 * timer1 interrupt setup for loop timing 
 */

void initLoopTiming() {
  // =========================================================================
  // setup 16-bit timer 1 to control the loop timing at 100Hz:
  cli();                // disable interrupts
  TCCR1A = 0;           // set entire TCCR1A register to 0
  TCCR1B = 0;           // same for TCCR1B
  TCNT1  = 0;           // initialize counter value to 0
  // set compare match register for 100 Hz increments
  OCR1A = 19999;        // = 16000000 / (8 * 100) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for prescaler: 8
  TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();                // enable interrupts
  // =========================================================================
}

#endif// TIMER_INIT_H