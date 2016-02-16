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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Linear Tele Op Mode
 * <p>
 * Enables control of the robot via the gamepad.
 * NOTE: This op mode will not work with the NXT Motor Controllers. Use an Nxt op mode instead.
 */

public class TT_SensorTest extends TobotHardware {

    final static double LIGHT_THRESHOLD = 0.5;

    //ColorSensor colorSensor;
    //DeviceInterfaceModule cdim;
    //OpticalDistanceSensor op;
    //UltrasonicSensor   ultra;
    //ColorSensor sensorRGB;
    //LightSensor ls1;
    //LightSensor ls2;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap.logDevices();

        tobot_init(State.STATE_TUNEUP);

        //cdim = hardwareMap.deviceInterfaceModule.get("dim");
        //colorSensor = hardwareMap.colorSensor.get("co");
        //colorSensor.enableLed(false);
        TT_ColorPicker cp = new TT_ColorPicker(coSensor);

        //ls1 = hardwareMap.lightSensor.get("ll");
        //ls2 = hardwareMap.lightSensor.get("lr");

        // turn on LED of light sensor.
        //ls1.enableLed(true);
        //ls2.enableLed(true);

        //op = hardwareMap.opticalDistanceSensor.get("op");
        //ultra = hardwareMap.ultrasonicSensor.get("ultra");
        //sensorRGB = hardwareMap.colorSensor.get("rgb");
        //sensorRGB.enableLed(true);
        //coSensor.enableLed(true);
        waitForStart();

        int detectwhite = 0;
        int count = 0;
        speedScale = (float) 0.5;
        double red_acc = 0, blue_acc = 0, red_final = 0, blue_final = 0;
        while (opModeIsActive()) {

            float left = -gamepad1.left_stick_y;
            float right = -gamepad1.right_stick_y;
            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);
            rightPower = (float) ((float) scaleInput(right * speedScale));
            leftPower = (float) ((float) scaleInput(left * speedScale));

            if (gamepad1.a) {
                // if the A button is pushed on gamepad1, decrease the speed
                // of the chassis
                if (speedScale > 0.1)
                    speedScale -= 0.01;
            } else if (gamepad1.y) {
                // if the Y button is pushed on gamepad1, increase the speed
                // of the chassis
                if (speedScale < 1)
                    speedScale += 0.01;
            }

            // write the values to the motors
            motorFR.setPower(rightPower);
            motorBR.setPower(rightPower);
            motorFL.setPower(leftPower);
            motorBL.setPower(leftPower);

            count++;
            red_acc += coSensor.red();
            blue_acc += coSensor.blue();

            if (count == 10) {
                red_final = red_acc;
                blue_final = blue_acc;
                red_acc = 0;
                blue_acc = 0;
                count = 0;
            }
            if (detectWhite()) {
                detectwhite = 1;
            } else {
                detectwhite = 0;
            }
            if (gamepad1.dpad_up) { // try out auto-blue
                auto_part1(false);
                sleep(1000);
                StraightIn(-0.3, 12);
                sleep(1000);
                TurnRightD(0.5, 90, true);
                sleep(5000);
            } else if (gamepad1.dpad_down) { // try out auto-red
                auto_part1(true);
                sleep(1000);
                StraightIn(-0.3, 12);
                sleep(1000);
                TurnLeftD(0.5, 90, true);
                sleep(5000);
            } else if (gamepad1.dpad_right) {
                TurnRightD(0.5, 90, true);
            } else if (gamepad1.dpad_left) {
                TurnLeftD(0.5, 90, true);
            }
            telemetry.addData("1. Red  cumu. / cur = ", red_final + String.format("/ %d", coSensor.red()));
            telemetry.addData("2. Blue cumu. / cur = ", blue_final + String.format("/ %d", coSensor.blue()));
            telemetry.addData("3. TT Color Picker  = ", String.format("%s", cp.getColor().toString()));
            telemetry.addData("4. Low color R/G/B  = ", String.format("%d / %d / %d", coSensor2.red(), coSensor2.green(), coSensor2.blue()));
            telemetry.addData("5. Ada C/B/R/G/Sum  = ", String.format("%d/%d/%d/%d/%d",coAda.alpha(),coAda.blue(),coAda.red(),coAda.green(),
                    (coAda.alpha()+coAda.blue()+coAda.red()+coAda.green())));
            telemetry.addData("6. White detected   = ", detectwhite);
            telemetry.addData("7. ODS / Ultra      = ", String.format("%.4f / %.4f", opSensor.getLightDetected(),ultra.getUltrasonicLevel()));
            telemetry.addData("8. Heading go / cur", String.format("%d / %d", heading, gyro.getHeading()));

            waitForNextHardwareCycle();
        }
    }
}
