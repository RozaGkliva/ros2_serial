/**
 * @file ros_serial_example.ino
 * @brief Example sketch that works with ros2_serial node to transmit data between an arduino and a ROS2 computer.
 *
 * Tested with Arduino Uno
 */

#include "config.h"
#include "timer_init.h"
#include <avr/wdt.h>

volatile bool wait = 0;             // loop timing variable
uint32_t main_loop_counter = 0;     // for assigning different timings to different operations

String receivedString = "";         // buffer to store incoming data
int desiredRPM = 0;                 // Variable to store the received RPM command

void sendStuffToROS() {
  int8_t int_value = 6;
  float float_value = 1.25;
  Serial.print("sync");             // Start marker
  // Serial.print(int_value);          // Send int8_t value
  // Serial.print(",");                // Separator
  // Serial.print(float_value, 2);     // Send float with 2 decimal places
  Serial.print(",desRPM:");
  Serial.print(desiredRPM);
  Serial.println();                 // End marker (newline)
}

void serialEvent() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      // Serial.print("got: ");
      // Serial.println(receivedString);
      processReceivedData(receivedString);
      receivedString = "";
    } else {
      receivedString += c;
    }
  }
}

// Process received packet
void processReceivedData(String data) {
  if (data.startsWith("rpm")) {  
    // Serial.println("got rpm cmd");
    int rpmValue = data.substring(3).toInt();
    if (rpmValue >= 0) {
      desiredRPM = rpmValue;
    }
  }
}

void setup() {
  initLoopTiming();

  Serial.begin(SERIAL_SPEED);

  wdt_enable (WDTO_1S); // enable watchdog timer with 1 sec timeout
}


void loop() {
  wait = 1;             // loop timing switch
  wdt_reset();          // reset the watchdog timer

  main_loop_counter++;

  // send data to ROS at 10Hz
  if (main_loop_counter % (MAIN_LOOP_FREQ / ROS_TRANSMIT_FREQ) == 0) {
    sendStuffToROS();
  }

  // reset the rpm value once a second
  // if (main_loop_counter % (MAIN_LOOP_FREQ / RESET_RPM_FREQ) == 0) {
  //   desiredRPM = 0;
  // }

  // if there is time left over in the after everything else is done...
  // it will wait here until the preallocated loop time elapses
  while (wait); 
}


ISR(TIMER1_COMPA_vect)     // timer compare interrupt service routine
{
  // this interrupt hits every 100ms. it is used for loop timing
  wait = 0;
}