///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the starting, stopping and take-off detection is managed.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void start_stop_takeoff(void) {
  Serial.print("Start: ");
  Serial.println(start);
  if (channel_3 < 1050 && channel_4 < 1050)start = 1;                              //For starting the motors: throttle low and yaw left (step 1).
  if (start == 1 && channel_3 < 1050 && channel_4 > 1450) {                        //When yaw stick is back in the center position start the motors (step 2).
    throttle = motor_idle_speed;                                                   //Set the base throttle to the motor_idle_speed variable.
    angle_pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    start = 2;                                                                     //Set the start variable to 2 to indicate that the quadcopter is started.
      //Reset the PID controllers for a smooth take-off.
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_output_roll = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_output_pitch = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
      pid_output_yaw = 0;
  
  }
  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
    start = 0;                                                                     //Set the start variable to 0 to disable the motors.
  }

}
