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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Linear Tele Op Mode
 * <p>
 * Enables control of the robot via the gamepad.
 * NOTE: This op mode will not work with the NXT Motor Controllers. Use an Nxt op mode instead.
 */
@TeleOp(name="Sensor-Test-2016", group="TT-LN-Op")
public class TT_2016_SensorTest extends TT_2016_Hardware {

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
        TT_ColorPicker cp2 = new TT_ColorPicker(coSensor2);
        boolean connected = false;
        //ls1 = hardwareMap.lightSensor.get("ll");
        //ls2 = hardwareMap.lightSensor.get("lr");

        // turn on LED of light sensor.
        //ls1.enableLed(true);
        //ls2.enableLeud(true);

        //op = hardwareMap.opticalDistanceSensor.get("op");
        //ultra = hardwareMap.ultrasonicSensor.get("ultra");
        //sensorRGB = hardwareMap.colorSensor.get("rgb");
        //sensorRGB.enableLed(true);
        //coSensor.enableLed(true);
        cdim.setDigitalChannelState(LED_CHANNEL, true); // enable Ada color sensor led for white line detection

        waitForStart();

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

            connected = false; // navx_device.isConnected();
            //telemetry.addData("1 navX-Device", connected ?
            //        "Connected" : "Disconnected" );

            if ( connected ) {
                yaw = navx_device.getYaw();
            }

            if (gamepad1.a) {
                // if the A button is pushed on gamepad1, decrease the speed
                // of the chassis
                if (speedScale > 0.2) {
                    speedScale -= 0.001;
                    sleep(5);
                }
            } else if (gamepad1.y) {
                // if the Y button is pushed on gamepad1, increase the speed
                // of the chassis
                if (speedScale < 1) {
                    speedScale += 0.001;
                    sleep(5);
                }
            }
            if (false) {
                if (gamepad1.dpad_down) {
                    if (light_sensor_sv_pos < 1)
                        set_light_sensor(light_sensor_sv_pos + 0.005);
                    sleep(5);
                }
                if (gamepad1.dpad_up) {
                    if (light_sensor_sv_pos > 0)
                        set_light_sensor(light_sensor_sv_pos - 0.005);
                    sleep(5);
                }
            }
            if (gamepad1.dpad_right) {
                if (left_beacon_sv_pos<1)
                    set_left_beacon(left_beacon_sv_pos+0.005);
                sleep(5);
            }
            if (gamepad1.dpad_left) {
                if (left_beacon_sv_pos>0)
                    set_left_beacon(left_beacon_sv_pos-0.005);
                sleep(5);
            }
            if (gamepad1.x) {
                if (right_beacon_sv_pos<1)
                    set_right_beacon(right_beacon_sv_pos+0.005);
                sleep(5);
            }
            if (gamepad1.b) {
                if (right_beacon_sv_pos>0)
                    set_right_beacon(right_beacon_sv_pos-0.005);
                sleep(5);
            }
            if (gamepad1.right_trigger > 0.1) { //

                sleep(300);
            }
            if (gamepad1.right_bumper) { //
                TurnRightD(0.9,90,true);
            }

            if (gamepad1.left_trigger > 0.1) { //

                sleep(300);
            }
            if (gamepad1.left_bumper) { //
                TurnLeftD(0.9,90,true);
            }

            if (gamepad2.dpad_left) {
                goBeacon(true);
            }
            if (gamepad2.dpad_right) {
                goBeacon(false);
            }

            // write the values to the motors
            motorR.setPower(rightPower);
            motorL.setPower(leftPower);

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
            if (gamepad1.dpad_up && gamepad1.start) {
                StraightIn(1 , 72);
            }
            else if (gamepad1.dpad_up) { // try out auto-blue
                DbgLog.msg("MY_DEBUG - Beginning of Auto_Part 1 red out!");
                auto_part1(true,false);
            } else if (gamepad1.dpad_down) { // try out auto-red
                DbgLog.msg("MY_DEBUG - Beginning of Auto_Part 1 red in!");
                auto_part1(true, true);
            } else if (gamepad1.dpad_right) {
                TurnRightD(0.5, 90, true);
            } else if (gamepad1.dpad_left) {
                TurnLeftD(0.5, 90, true);
            }
            if (gamepad2.dpad_left) {
                if (gate_sv_pos>0.005)
                    set_gate(gate_sv_pos-0.005);
                sleep(5);
            }
            if (gamepad2.dpad_right) {
                if (gate_sv_pos<0.995)
                    set_gate(gate_sv_pos+0.005);
                sleep(5);
            }
            if (gamepad2.dpad_down) {
                if (pusher_sv_pos>0.005)
                    set_pusher(pusher_sv_pos-0.005);
                sleep(5);
            }
            if (gamepad2.dpad_up) {
                if (pusher_sv_pos<0.995)
                    set_pusher(pusher_sv_pos+0.005);
                sleep(5);
            }



            //touch = (tSensor.isPressed()?1:0);

            show_telemetry();
            if (false) {
                // tel9emetry.addData("1. Red  cumu. / cur = ", red_final + String.format("/ %d", coSensor.red()));
                // telemetry.addData("2. Blue cumu. / cur = ", blue_final + String.format("/ %d", coSensor.blue()));
                telemetry.addData("1. TT Color Picker 1/2 = ", String.format("%s / %s", cp.getColor().toString(), cp2.getColor().toString()));
                telemetry.addData("2. color-1 R/G/B    = ", String.format("%d / %d / %d", coSensor.red(), coSensor.green(), coSensor.blue()));
                telemetry.addData("3. color-2 R/G/B    = ", String.format("%d / %d / %d", coSensor2.red(), coSensor2.green(), coSensor2.blue()));
                telemetry.addData("4. sv ls/l_b/r_b  = ", String.format("%.2f / %.2f / %.2f", light_sensor_sv_pos, left_beacon_sv_pos, right_beacon_sv_pos));
                telemetry.addData("5. gate/ pusher  = ", String.format("%.2f / %.2f", gate_sv_pos, pusher_sv_pos));
                telemetry.addData("6. Ada C/B/R/G/Sum  = ", String.format("%d/%d/%d/%d/%d", coAda.alpha(), coAda.blue(), coAda.red(), coAda.green(),
                        (coAda.alpha() + coAda.blue() + coAda.red() + coAda.green())));
                telemetry.addData("7. White detected   = ", String.format("%d",detectwhite));
                //telemetry.addData("7. ODS / Ultra / Touch = ", String.format("%.4f / %.4f / %d",
                //opSensor.getLightDetected(),ultra.getUltrasonicLevel(),touch));
                //telemetry.addData("8. Heading goal / gyro / navx", String.format("%d / %d / %4.2f",
                //        heading, gyro.getHeading(),yaw));
                telemetry.update();
            }
        }
        cdim.setDigitalChannelState(LED_CHANNEL, false);
        if (connected)
            navx_device.close();
    }
}
