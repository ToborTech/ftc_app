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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TobotHardware_Op;

/**
 * TobotHardware
 * <p/>
 * Define all hardware (e.g. motors, servos, sensors) used by Tobot
 */
@TeleOp(name="TeleOp-Test", group="TT-OpModes")
@Disabled
public class
TT_TeleOp_Op extends TobotHardware_Op {
    @Override
    public void loop() {
        // getIMUGyroAngles();
        if (debug) DbgLog.msg(String.format("TOBOT-loop-0"));
        Boolean hanging_there = false;

        if (ch_action== Action.RTURN) { // right turn state
            TurnRightD(leftPower, 0, true);
        } else if (ch_action== Action.LTURN) {
            TurnLeftD(rightPower, 0, true);
        } else if (ch_action== Action.STRAIGHT) {
            StraightR(rightPower,0);
        } else { // main loop for detection all button
            if (true) {
                ; // skip
            } else if (test_count == 0) {
                    StraightIn(1, 90);
                    test_count++;
                    return;
                } else if (test_count == 100) {
                    TurnRightD(1, 45, true); // red zone
                    test_count++;
                    return;
                } else if (test_count == 200) {
                    StraightIn(1, 20);
                    test_count++;
                    return;
                } else if (test_count == 300) {
                    TurnLeftD(1, 90, true); // facing beacon
                    test_count++;
                    return;
                } else if (test_count<10000) {
                    test_count++;
                }

            float left = -gamepad1.left_stick_y;
            float right = -gamepad1.right_stick_y;
            if (debug) DbgLog.msg(String.format("TOBOT-loop-1"));
            elbow_pos = elbow.getCurrentPosition();
            tape_slider_pos = tape_slider.getCurrentPosition();
            tape_rotator_pos = tape_rotator.getPosition();
            shoulder_dir = -gamepad2.right_stick_x;
            elbow_dir = gamepad2.right_stick_y;
            tape_slider_dir = -gamepad2.left_stick_y;
            // tape_rotator_dir = -gamepad2.left_stick_x;
            if (debug) DbgLog.msg(String.format("TOBOT-loop-2"));
            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.

            // Use speedScale to control the speed
            rightPower = (float) ((float) scaleInput(right * speedScale));
            leftPower = (float) ((float) scaleInput(left * speedScale));

            if (debug) DbgLog.msg(String.format("TOBOT-loop-3"));

            if (gamepad1.b && gamepad1.x) { // stop sweeper
                SW_power = (float) 0;
                motorSW.setPower(SW_power); // stop right away
                //sleep(400); // make sure other botton reset
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
                TurnRightD(1, 90, true);
                //sleep(500);
            }
            if (gamepad1.right_bumper) { // right climber up
                set_right_climber(RIGHT_CLIMBER_UP);
            }

            if (gamepad1.left_trigger > 0.1) { // left climber down
                TurnLeftD(1, 90, true);
                //sleep(300);
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
                cur_arm_power = -arm_power;
                elbow_count = 10;
            } else if (elbow_dir > THRESHOLD) { // arm up
                cur_arm_power = arm_power;
                elbow_count = 10;
            } else {
                cur_arm_power = 0;
            }
            if (tape_slider_dir < -THRESHOLD * 2.0) { // tape slider in 100% power
                if (gamepad2.start)
                    hanging_there = true;
                else if (gamepad2.back)
                    hanging_there = false;
                tape_slider.setPower(-1);
            } else if (tape_slider_dir > THRESHOLD * 2.0) { // tape slider out 100% power
                tape_slider.setPower(1);
                hanging_there = false;
            } else if (hanging_there) { // minimum power to ensure Tobot does not slide down
                tape_slider.setPower(-0.2);
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
            if (debug) DbgLog.msg(String.format("TOBOT-loop-4"));
            if ((gamepad2.right_trigger > 0.1) && gamepad2.start) {
                // calibre_elbow();
                inc_wristLR_pos(-0.01);
            } else if (gamepad2.right_trigger > 0.1) {
                gate_pos = GATE_OPEN;
            }

            if (gamepad2.right_bumper && gamepad2.start) {
                inc_wristLR_pos(0.01);
                // change arm state
                // arm_init -> arm_down_back -> arm_up_back -> arm_up_front ->arm_init
                //if (arm_state==ArmState.ARM_INIT) {
                //    arm_state = ArmState.ARM_DOWN_BACK;
                //} else if (arm_state==ArmState.ARM_DOWN_BACK) {
                //    arm_state = ArmState.ARM_UP_BACK;
                //} else if (arm_state==ArmState.ARM_UP_BACK) {
                //    arm_state = ArmState.ARM_UP_FRONT;
                //} else if (arm_state==ArmState.ARM_UP_FRONT) {
                //    arm_state = ArmState.ARM_INIT;
                //}
                //gamepad2.reset();
                ////sleep(200);
            } else if (gamepad2.right_bumper) {
                gate_pos = GATE_CLOSED;
            }

            gate.setPosition(gate_pos);

            if ((gamepad2.left_trigger > 0.1) && (gamepad2.start)) {
                inc_wristUD_pos(-0.01);
            } else if (gamepad2.left_trigger > 0.1) { // tape down
                tape_rotator_dir = -1;
                tape_count = 0;
            }

            if (gamepad2.left_bumper && gamepad2.start) {
                inc_wristUD_pos(0.01);
            } else if (gamepad2.left_bumper) {
                tape_rotator_dir = 1;
                tape_count = 0;
            }

            if (tape_rotator_dir < -0.1) { // tape down
                inc_tape_rotator(-SERVO_SCALE / 4.0);
            } else if (tape_rotator_dir > 0.1) { // arm up
                inc_tape_rotator(SERVO_SCALE / 4.0);
            }

            if (gamepad2.dpad_up) {
                stop_chassis();
                gamepad2.reset();
                if (arm_state == ArmState.ARM_DOWN_BACK) {
                    arm_up();
                } else if (arm_state == ArmState.ARM_INIT) {
                    arm_up();
                } else if (arm_state == ArmState.ARM_UP_FRONT) {
                    arm_front();
                } else if (arm_state == ArmState.ARM_SCORE_MID_RED || arm_state == ArmState.ARM_SCORE_MID_BLUE ||
                        arm_state == ArmState.ARM_FRONT_DUMP || arm_state == ArmState.ARM_SCORE_HIGH_RED ||
                        arm_state == ArmState.ARM_SCORE_HIGH_BLUE) {
                    arm_back_from_goal();
                }
            } else if (gamepad2.dpad_left) {
                // gamepad2.reset();
                stop_chassis();
                if (arm_state == ArmState.ARM_UP_FRONT || arm_state == ArmState.ARM_DOWN_FRONT) {
                    if (gamepad2.start)
                        go_red_high_zone();
                    else
                        go_red_mid_zone();
                } else if (arm_state == ArmState.ARM_UP_BACK) {
                    arm_down();
                }
            } else if (gamepad2.dpad_right) {
                // gamepad2.reset();
                stop_chassis();
                if (arm_state == ArmState.ARM_UP_FRONT || arm_state == ArmState.ARM_DOWN_FRONT) {
                    if (gamepad2.start)
                        go_blue_high_zone();
                    else
                        go_blue_mid_zone();
                } else if (arm_state == ArmState.ARM_UP_BACK) {
                    arm_down();
                }
            } else if (gamepad2.dpad_down) {
                gamepad2.reset();
                stop_chassis();
                if (arm_state == ArmState.ARM_UP_FRONT) {
                    arm_back();
                    ////sleep(1000);
                } else if (arm_state == ArmState.ARM_SCORE_MID_RED || arm_state == ArmState.ARM_SCORE_MID_BLUE ||
                        arm_state == ArmState.ARM_FRONT_DUMP || arm_state == ArmState.ARM_SCORE_HIGH_RED ||
                        arm_state == ArmState.ARM_SCORE_HIGH_BLUE) {
                    arm_front_from_goal();
                } else if (arm_state == ArmState.ARM_DOWN_FRONT) {
                    arm_up();
                }
            }
        } // Action.INIT
        // write the values to the motors
        show_telemetry();
        if (debug) DbgLog.msg(String.format("TOBOT-loop-5"));
        motorFR.setPower(rightPower);
        motorBR.setPower(rightPower);
        motorFL.setPower(leftPower);
        motorBL.setPower(leftPower);
        motorSW.setPower(SW_power);
        elbow.setPower(cur_arm_power);
        if (debug) DbgLog.msg(String.format("TOBOT-loop-6"));
    }

    @Override
    public void stop() {
        test_count = 0;
        stop_tobot();
    }
}
