/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;


/**
 * Created by Owner on 8/31/2015.
 */
public class IMUtest extends OpMode {

  AdafruitIMU boschBNO055;

  //The following arrays contain both the Euler angles reported by the IMU (indices = 0) AND the
  // Tait-Bryan angles calculated from the 4 components of the quaternion vector (indices = 1)
  volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

  long systemTime;//Relevant values of System.nanoTime

  /************************************************************************************************
   * The following method was introduced in the 3 August 2015 FTC SDK beta release and it runs
   * before "start" runs.
   */
  @Override
  public void init() {
    systemTime = System.nanoTime();
    try {
      boschBNO055 = new AdafruitIMU(hardwareMap, "imu"

              //The following was required when the definition of the "I2cDevice" class was incomplete.
              //, "cdim", 5

              , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
              //addressing
              , (byte)AdafruitIMU.OPERATION_MODE_IMU);
    } catch (RobotCoreException e){
      Log.i("FtcRobotController", "Exception: " + e.getMessage());
    }
    Log.i("FtcRobotController", "IMU Init method finished in: "
            + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
    //ADDRESS_B is the "standard" I2C bus address for the Bosch BNO055 (IMU data sheet, p. 90).
    //BUT DAVID PIERCE, MENTOR OF TEAM 8886, HAS EXAMINED THE SCHEMATIC FOR THE ADAFRUIT BOARD ON
    //WHICH THE IMU CHIP IS MOUNTED. SINCE THE SCHEMATIC SHOWS THAT THE COM3 PIN IS PULLED LOW,
    //ADDRESS_A IS THE IMU'S OPERATIVE I2C BUS ADDRESS
    //IMU is an appropriate operational mode for FTC competitions. (See the IMU datasheet, Table
    // 3-3, p.20 and Table 3-5, p.21.)
  }

  /************************************************************************************************
   * Code to run when the op mode is first enabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
   */
  @Override
  public void start() {
        /*
      	* Use the hardwareMap to get the dc motors, servos and other sensors by name. Note
      	* that the names of the devices must match the names used when you
      	* configured your robot and created the configuration file. The hardware map
      	* for this OpMode is not initialized until the OpModeManager's "startActiveOpMode" method
      	* runs.
    		*/
    systemTime = System.nanoTime();
    boschBNO055.startIMU();//Set up the IMU as needed for a continual stream of I2C reads.
    Log.i("FtcRobotController", "IMU Start method finished in: "
            + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
  }

  /***********************************************************************************************
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   * NOTE: BECAUSE THIS "loop" METHOD IS PART OF THE OVERALL OpMode/EventLoop/ReadWriteRunnable
   * MECHANISM, ALL THAT THIS METHOD WILL BE USED FOR, IN AUTONOMOUS MODE, IS TO:
   * 1. READ SENSORS AND ENCODERS AND STORE THEIR VALUES IN SHARED VARIABLES
   * 2. WRITE MOTOR POWER AND CONTROL VALUES STORED IN SHARED VARIABLES BY "WORKER" THREADS, AND
   * 3. SEND TELELMETRY DATA TO THE DRIVER STATION
   * THIS "loop" METHOD IS THE ONLY ONE THAT "TOUCHES" ANY SENSOR OR MOTOR HARDWARE.
   */
  @Override
  public void loop() {
    //Log.i("FtcRobotController", "Loop method starting at: " +
    //      -(systemTime - (systemTime = System.nanoTime())) + " since last loop start.");

    // write the values computed by the "worker" threads to the motors (if any)

    //Read the encoder values that the "worker" threads will use in their computations
    boschBNO055.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
		/*
		 * Send whatever telemetry data you want back to driver station.
		 */
    //telemetry.addData("Text", "*** Robot Data***");
    telemetry.addData("Headings(yaw): ",
            String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
    telemetry.addData("Pitches: ",
            String.format("Euler= %4.5f, Quaternion calculated= %4.5f", pitchAngle[0], pitchAngle[1]));
    telemetry.addData("Max I2C read interval: ",
            String.format("%4.4f ms. Average interval: %4.4f ms.", boschBNO055.maxReadInterval
                    , boschBNO055.avgReadInterval));
  }

  /*
  * Code to run when the op mode is first disabled goes here
  * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
  */
  @Override
  public void stop() {
    //When the FTC Driver Station's "Start with Timer" button commands autonomous mode to start,
    //then stop after 30 seconds, stop the motors immediately!
    //Following this method, the underlying FTC system will call a "stop" routine of its own
    systemTime = System.nanoTime();
    Log.i("FtcRobotController", "IMU Stop method finished in: "
            + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
  }
}