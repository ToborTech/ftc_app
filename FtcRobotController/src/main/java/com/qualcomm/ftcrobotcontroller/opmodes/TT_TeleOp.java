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

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.util.Range;

/**
 * TobotHardware
 * <p/>
 * Define all hardware (e.g. motors, servos, sensors) used by Tobot
 */
public class TT_TeleOp extends TobotHardware {
    @Override
    public void runOpMode() throws InterruptedException {

        tobot_init(State.STATE_TELEOP);

        waitForStart();

        while (opModeIsActive()) {

            float left = -gamepad1.left_stick_y;
            float right = -gamepad1.right_stick_y;

            elbow_pos = elbow.getCurrentPosition();
            tape_slider_pos = tape_slider.getCurrentPosition();
            tape_rotator_pos = tape_rotator.getCurrentPosition();
            shoulder_dir = -gamepad2.right_stick_x;
            elbow_dir = -gamepad2.right_stick_y;
            tape_slider_dir = -gamepad2.left_stick_y;
            // tape_rotator_dir = -gamepad2.left_stick_x;

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
                sleep(300);
            }
            if (gamepad1.left_bumper) { // left climber up
                set_left_climber(LEFT_CLIMBER_UP);
            }
            if (gamepad1.dpad_down) {
                // front guard down
                front_sv_down();
            }
            if (gamepad1.dpad_up) {
                // front guard up
                front_sv_up();
            }
            if (slider_counter > 0)
                slider_counter--;
            else {
                arm_slider.setPosition(SLIDER_STOP);
            }

            if (tape_count > 0)
                tape_count--;
            else {
                tape_rotator_dir = 0;
            }

            if (gamepad2.x || gamepad2.b) { // control continuous serve requires wait
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
                cur_arm_power = -arm_power * 0.3;
                elbow_count = 10;
            } else if (elbow_dir > THRESHOLD) { // arm up
                cur_arm_power = arm_power;
                elbow_count = 10;
            } else {
                cur_arm_power = 0;
            }
            if (tape_slider_dir < -THRESHOLD * 2.0) { // tape slider in 100% power
                tape_slider.setPower(-1);
            } else if (tape_slider_dir > THRESHOLD * 2.0) { // tape slider out 100% power
                tape_slider.setPower(1);
            } else {
                tape_slider.setPower(0);
            }

            if (shoulder_dir > THRESHOLD) {
                shoulder_pos += (SERVO_SCALE);
                if (shoulder_pos > 1) {
                    shoulder_pos = 0.99;
                }
            } else if (shoulder_dir < THRESHOLD * -1) {
                shoulder_pos -= (SERVO_SCALE);
                if (shoulder_pos < 0) {
                    shoulder_pos = 0.01;
                }
            }
            shoulder.setPosition(shoulder_pos);

            if ((gamepad2.right_trigger>0.1) && gamepad2.start) {
                calibre_elbow();
            } else if (gamepad2.right_trigger > 0.1) {
                gate_pos = GATE_OPEN;
            }

            if (gamepad2.right_bumper && gamepad2.start) { // change arm state
                // arm_init -> arm_down_back -> arm_up_back -> arm_up_front ->arm_init
                if (arm_state==ArmState.ARM_INIT) {
                    arm_state = ArmState.ARM_DOWN_BACK;
                } else if (arm_state==ArmState.ARM_DOWN_BACK) {
                    arm_state = ArmState.ARM_UP_BACK;
                } else if (arm_state==ArmState.ARM_UP_BACK) {
                    arm_state = ArmState.ARM_UP_FRONT;
                } else if (arm_state==ArmState.ARM_UP_FRONT) {
                    arm_state = ArmState.ARM_INIT;
                }
                gamepad2.reset();
                sleep(200);
            } else if (gamepad2.right_bumper) {
                gate_pos = GATE_CLOSED;
            }

            gate.setPosition(gate_pos);

            if ((gamepad2.left_trigger > 0.1) && (gamepad2.start)) {
                set_wrist_pos(WRIST_COLLECT);
            } else if (gamepad2.left_trigger > 0.1) { // tape down
                    tape_rotator_dir = -1;
                    tape_count = 0;
            }

            if (gamepad2.left_bumper && gamepad2.start) {
                set_wrist_pos(WRIST_UP);
            } else if (gamepad2.left_bumper ){
                    tape_rotator_dir = 1;
                    tape_count = 0;
            }

            if (tape_rotator_dir < -0.1) { // tape down 20% of power
                tape_rotator.setPower(-0.15); sleep(2);
            } else if (tape_rotator_dir > 0.1) { // arm up 30% of power
                tape_rotator.setPower(0.25); sleep(2);
            } else {
                tape_rotator.setPower(0);
            }

            if (gamepad2.dpad_up) {
                gamepad2.reset();
                if (arm_state == ArmState.ARM_COLLECT) {
                    arm_collect_mode_to_up_back();
                } else if (arm_state == ArmState.ARM_DOWN_BACK) {
                    arm_up();
                } else if (arm_state == ArmState.ARM_UP_BACK) {
                    arm_front(true);
                    sleep(1000);
                } else if (arm_state == ArmState.ARM_INIT) {
                    release_arm();
                } else if (arm_state == ArmState.ARM_UP_FRONT) {
                    arm_down();
                } else if (arm_state == ArmState.ARM_DOWN_FRONT) {
                    arm_up();
                }
            } else if (gamepad2.dpad_left) {
                gamepad2.reset();
                if (arm_state == ArmState.ARM_UP_FRONT || arm_state == ArmState.ARM_DOWN_FRONT) {
                    go_red_mid_zone();
                } else if (arm_state == ArmState.ARM_UP_BACK) {
                    arm_down();
                }
            } else if (gamepad2.dpad_right) {
                gamepad2.reset();
                if (arm_state == ArmState.ARM_UP_FRONT || arm_state == ArmState.ARM_DOWN_FRONT) {
                    go_blue_mid_zone();
                } else if (arm_state == ArmState.ARM_UP_BACK) {
                    arm_down();
                }
            } else if (gamepad2.dpad_down) {
                gamepad2.reset();
                if (arm_state == ArmState.ARM_UP_BACK || arm_state == ArmState.ARM_DOWN_BACK ||
                        arm_state == ArmState.ARM_INIT) {
                    arm_collection_mode();
                } else if (arm_state == ArmState.ARM_UP_FRONT) {
                    arm_back();
                    sleep(1000);
                } else if (arm_state == ArmState.ARM_SCORE_MID_RED || arm_state == ArmState.ARM_SCORE_MID_BLUE) {
                    arm_back_from_goal();
                }
            }

        show_telemetry();
        waitOneFullHardwareCycle();
        }
    }
}
