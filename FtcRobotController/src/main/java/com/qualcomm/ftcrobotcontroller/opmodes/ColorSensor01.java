/* Copyright (c) 2015 Qualcomm Technologies Inc

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

import com.qualcomm.hardware.HiTechnicNxtLightSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Linear Tele Op Mode
 * <p>
 * Enables control of the robot via the gamepad.
 * NOTE: This op mode will not work with the NXT Motor Controllers. Use an Nxt op mode instead.
 */

public class ColorSensor01 extends LinearOpMode {

  final static double LIGHT_THRESHOLD = 0.5;

  ColorSensor colorSensor;
  DeviceInterfaceModule cdim;
  TouchSensor t;
  LightSensor ls1;
  LightSensor ls2;

  @Override
  public void runOpMode() throws InterruptedException {
    hardwareMap.logDevices();

    cdim = hardwareMap.deviceInterfaceModule.get("dim");
    colorSensor = hardwareMap.colorSensor.get("mr");

    ls1 = hardwareMap.lightSensor.get("ls1");
    ls2 = hardwareMap.lightSensor.get("ls2");

    // turn on LED of light sensor.
    ls1.enableLed(true);
    ls2.enableLed(true);

    t = hardwareMap.touchSensor.get("t");

    waitForStart();

    int count = 0;
    float red_acc = 0 , blue_acc = 0 , red_final = 0 , blue_final = 0;
    while (opModeIsActive()) {

      count++;
      red_acc += colorSensor.red();
      blue_acc += colorSensor.blue();


      if (count == 10){
        red_final = red_acc;
        blue_final = blue_acc;
        red_acc = 0;
        blue_acc = 0;
        count = 0;
        telemetry.addData("Red", red_final);
        telemetry.addData("Blue", blue_final);
        telemetry.addData("LS1", ls1.getLightDetected());
        telemetry.addData("LS2", ls2.getLightDetected());
      }
    }
  }
}
