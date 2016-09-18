/* Copyright (c) 2014 Qualcomm Technologies Inc

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
import com.qualcomm.robotcore.util.Range;

/**
 * TobotHardware
 * <p/>
 * Define all hardware (e.g. motors, servos, sensors) used by Tobot
 */
public class TT_TuneUp extends TobotHardware {

    @Override
    public void runOpMode() throws InterruptedException {

        tobot_init(State.STATE_TUNEUP);

        waitForStart();

        while (opModeIsActive()) {
            // disable chassis wheels now to give gamepad1 sticks to tape control
            float left = 0;  // -gamepad1.left_stick_y;
            float right = 0; // -gamepad1.right_stick_y;

            elbow_pos = elbow.getCurrentPosition();
            tape_slider_pos = tape_slider.getCurrentPosition();
            tape_rotator_pos = tape_rotator.getPosition();
            tape_rotator_dir = -gamepad1.left_stick_y;
            tape_slider_dir = -gamepad1.right_stick_y;
            shoulder_dir = -gamepad2.left_stick_x;
            elbow_dir = -gamepad2.right_stick_y;

            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.

            // Use speedScale to control the speed
            rightPower = (float) ((float) scaleInput(right * speedScale));
            leftPower = (float) ((float) scaleInput(left * speedScale));

            // write the values to the motors
            motorFR.setPower(rightPower);
            motorBR.setPower(rightPower);
            motorFL.setPower(leftPower);
            motorBL.setPower(leftPower);
            motorSW.setPower(SW_power);
            elbow.setPower(cur_arm_power);

            if (state == State.STATE_TUNEUP) {
                if (gamepad1.dpad_down) { // backward 2-rotation
                    // StraightR(-0.8, 2);
                    front_sv_pos -= (SERVO_SCALE);
                    if (front_sv_pos < 0) {
                        front_sv_pos = 0.01;
                    }
                    front_sv.setPosition(front_sv_pos);
                }
                if (gamepad1.dpad_up) { //forward 2 rotation
                    // StraightR(0.8, 2);
                    auto_part1(false, true);
                  front_sv_pos += (SERVO_SCALE);
                    if (front_sv_pos > 1) {
                        front_sv_pos = 0.99;
                    }
                    front_sv.setPosition(front_sv_pos);
                }
                if (gamepad1.dpad_left) { //left spot turn 90 Degrees
                    DbgLog.msg(String.format("MY_DEBUG - Beginning of left turn 90D cur heading = %d!", gyro.getHeading()));
                    TurnLeftD(1, 90, true);
                    DbgLog.msg(String.format("MY_DEBUG - End of left turn 90D tar/cur heading = %d/%d!", heading, gyro.getHeading()));
                    sleep(1000);
                    DbgLog.msg(String.format("Gyro current heading = %d, power L/R = %.2f/%.2f",
                            gyro.getHeading(), leftPower, rightPower));
                }
                if (gamepad1.dpad_right) { //right spot turn 90 Degrees
                    DbgLog.msg(String.format("MY_DEBUG - Beginning of right turn 90D cur heading = %d!", gyro.getHeading()));
                    TurnRightD(1, 90, true);
                    DbgLog.msg(String.format("MY_DEBUG - End of right turn 90D tar/cur heading = %d/%d!", heading, gyro.getHeading()));
                    sleep(1000);
                    DbgLog.msg(String.format("Gyro current heading = %d, power L/R = %.2f/%.2f",
                            gyro.getHeading(), leftPower, rightPower));
                }
            }
            if (gamepad1.b && gamepad1.x) { // stop sweeper
                SW_power = (float) 0;
                motorSW.setPower(SW_power); // stop right away
                sleep(400); // make sure other botton reset
            } else if (gamepad1.b) { // sweeper backward
                SW_power = (float) 1.0;
            } else if (gamepad1.x) { // sweeper forward
                SW_power = (float) -1.0;
            }
            // update the speed of the chassis, or stop tape slider
            if (gamepad1.a && gamepad1.y) {
                tape_slider_dir = 0;
                tape_count = 0;
            } else if (gamepad1.a) {
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

            if (gamepad1.right_trigger > 0.1) { // right climber down: mid then low
                if (Math.abs(climberR_pos - RIGHT_CLIMBER_MID) < 0.05) {
                    set_right_climber(RIGHT_CLIMBER_LOW);
                } else {
                    set_right_climber(RIGHT_CLIMBER_MID);
                }
                sleep(500);
            }
            if (gamepad1.right_bumper) { // right climber up
                set_right_climber(RIGHT_CLIMBER_UP);
            }
            if (gamepad1.left_trigger > 0.1) { // left climber down
                if (Math.abs(climberL_pos - LEFT_CLIMBER_MID) < 0.05) {
                    set_left_climber(LEFT_CLIMBER_LOW);
                } else {
                    set_left_climber(LEFT_CLIMBER_MID);
                }
                sleep(500);
            }
            if (gamepad1.left_bumper) { // left climber up
                set_left_climber(LEFT_CLIMBER_UP);
            }
            //control direction of tape slider and tape rotator
            // up to stop/slide out
            // down to stop/slide in
            // left to stop/rotate up
            // right to stop/rotate down
            if (gamepad1.back) { // stop tape
                tape_slider_dir = 0;
                tape_rotator_dir = 0;
                tape_slider.setPower(0); // slider power off right away
            }

            if (slider_counter > 0)
                slider_counter--;
            else {
                arm_slider.setPosition(SLIDER_STOP);
            }

            if (tape_count > 0)
                tape_count--;
            else {
                // tape_rotator_dir = 0;
            }
            if (gamepad2.x && gamepad2.b){
                climber_mission(true);
            }
            else if (gamepad2.x || gamepad2.b) { // control continuous serve requires wait
                slider_counter = 10; // 20ms * 10 = 0.2 sec
                if (gamepad2.x) {
                    slider_pos = SLIDER_SHORTEN;
                    slider_curr_count -= slider_counter;
                }
                if (gamepad2.b) {
                    slider_pos = SLIDER_LENGHTEN;
                    slider_curr_count += slider_counter;
                }
                arm_slider.setPosition(slider_pos);
            }

            if (gamepad2.y) {
                arm_power += 0.005;
                if (arm_power > 1) {
                    arm_power = 1;
                }
            }
            if (gamepad2.a) {
                arm_power -= 0.005;
                if (arm_power < 0) {
                    arm_power = 0.05;
                }
            }

            // control to the elbow motor power
            if (elbow_count > 0) {
                elbow_count--;
            } else if (elbow_dir < -THRESHOLD) { // arm down 20% of power
                cur_arm_power = -arm_power * 0.2;
                elbow_count = 10;
            } else if (elbow_dir > THRESHOLD) { // arm up
                cur_arm_power = arm_power;
                elbow_count = 10;
            } else {
                cur_arm_power = 0;
            }
            if (tape_slider_dir < -THRESHOLD) { // tape slider in 100% power
                tape_slider.setPower(-1);
            } else if (tape_slider_dir > THRESHOLD) { // tape slider out 100% power
                tape_slider.setPower(1);
            } else {
                tape_slider.setPower(0);
            }
            if (tape_rotator_dir < -THRESHOLD) { // tape down 20% of power
                inc_tape_rotator(-SERVO_SCALE);
            } else if (tape_rotator_dir > THRESHOLD) { // arm up 30% of power
                inc_tape_rotator(SERVO_SCALE);
            }
            if (shoulder_dir > THRESHOLD) {
                shoulder_pos += (SERVO_SCALE/6.0);
                if (shoulder_pos > 1) {
                    shoulder_pos = 0.99;
                }
            } else if (shoulder_dir < THRESHOLD * -1) {
                shoulder_pos -= (SERVO_SCALE/6.0);
                if (shoulder_pos < 0) {
                    shoulder_pos = 0.01;
                }
            }
            shoulder.setPosition(shoulder_pos);

            // manual adjust wrist position
            if ((gamepad2.left_trigger > 0.1) && gamepad2.start) { // wristLR -
                wristLR_pos -= SERVO_SCALE;
                if (wristLR_pos < 0.01) wristLR_pos = 0.01;
            } else if (gamepad2.left_bumper && gamepad2.start) {
                wristLR_pos += SERVO_SCALE;
                if (wristLR_pos > 0.99) wristLR_pos = 0.99;
            }
            else if (gamepad2.left_trigger > 0.1) { // wrist servo down
                wristUD_pos -= SERVO_SCALE;
                if (wristUD_pos < 0.01) wristUD_pos = 0.01;
            }
            if (gamepad2.left_bumper) { // wrist servo up
                wristUD_pos += SERVO_SCALE;
                if (wristUD_pos > 0.99) wristUD_pos = 0.99;
            }
            wristUD.setPosition(wristUD_pos);
            wristLR.setPosition(wristLR_pos);

            // manual adjust gate position
            if (gamepad2.right_trigger > 0.1) { // gate servo down
                gate_pos -= SERVO_SCALE;
                if (gate_pos < 0.01) gate_pos = 0.01;
            }
            if (gamepad2.right_bumper) { // gate servo up
                gate_pos += SERVO_SCALE;
                if (gate_pos > 0.99) gate_pos = 0.99;
            }
            gate.setPosition(gate_pos);

            // manual adjust leveler
            if (gamepad2.dpad_left) {
                if (gamepad2.start) {
                    leveler_left();
                } else {
                    leveler_pos += SERVO_SCALE;
                    if (leveler_pos > 0.99) leveler_pos = 0.99;
                }
            } else if (gamepad2.dpad_right) {
                if (gamepad2.start) {
                    leveler_right();
                } else {
                    leveler_pos -= SERVO_SCALE;
                    if (leveler_pos < 0.01) leveler_pos = 0.01;
                }
            }
            leveler.setPosition(leveler_pos);

            // manual adjust light sensor position
            if (gamepad2.dpad_up) {
                light_sensor_sv_pos += SERVO_SCALE;
                if (light_sensor_sv_pos > 0.99) light_sensor_sv_pos = 0.99;
            } else if (gamepad2.dpad_down) {
                light_sensor_sv_pos -= SERVO_SCALE;
                if (light_sensor_sv_pos < 0.01) light_sensor_sv_pos = 0.01;
            }
            light_sensor_sv.setPosition(light_sensor_sv_pos);

            show_telemetry();
            waitOneFullHardwareCycle();
        }
    }

    public void show_telemetry() {
        telemetry.addData("0. Program/Arm State: ", state.toString() + "/" + arm_state.toString());
        telemetry.addData("1. shoulder:", "pos= " + String.format("%.4f, dir=%.2f)", shoulder_pos, shoulder_dir));
        telemetry.addData("2. elbow:", "pwr= " + String.format("%.2f, pos= %d, dir=%.2f", cur_arm_power, elbow_pos, elbow_dir));
        telemetry.addData("3. wrist LR/UD", "pos= " + String.format("%.2f / %.2f", wristLR_pos, wristUD_pos));
        telemetry.addData("4. arm_slider", "pos (dir): " + String.format("%.2f (%.2f)", slider_pos, slider_dir));
        telemetry.addData("5. tape_rotator/gate", "pos= " + String.format("%.2f / %.2f", tape_rotator_pos, gate_pos));
        telemetry.addData("6. drive power: L=", String.format("%.2f", leftPower) + "/R=" + String.format("%.2f", rightPower));
        telemetry.addData("7. leveler = ", String.format("%.2f", leveler_pos) + String.format(", light sensor sv = %.2f", light_sensor_sv_pos));
        telemetry.addData("8. right cur/tg enc:", motorFR.getCurrentPosition() + "/" + rightCnt);
        show_heading();
        //telemetry.addData("9. ods:", String.format("%.2f, imu: disable", opSensor.getLightDetected())+
        //        String.format("fr_sv_pos = %.2f",front_sv_pos));
    }

}