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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * TobotHardware
 * <p/>
 * Define all hardware (e.g. motors, servos, sensors) used by Tobot
 */
@TeleOp(name="TeleOp-2016", group="TT-LN-OP")
public class TT_2016_TeleOp extends TT_2016_Hardware {
    @Override
    public void runOpMode() throws InterruptedException {

        tobot_init(State.STATE_TELEOP);
        Boolean hanging_there = false;
        int test_count = 0;


        waitForStart();

        // StraightIn(0.4, 10);
        cdim.setDigitalChannelState(LED_CHANNEL, true);

        while (opModeIsActive()) {
            if (true) {
                ; // skip turn testing
            } else if (test_count == 0) {
                // StraightR(1.0, 1);
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 100) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 200) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 300) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 400) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 500) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 600) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 700) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;

            } else if (test_count == 1000) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1100) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1200) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1300) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1400) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1500) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1600) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1700) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else {
                test_count++;
            }

            float left = -gamepad1.left_stick_y;
            float right = -gamepad1.right_stick_y;

            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.

            // Use speedScale to control the speed
            rightPower = (float) ((float) scaleInput(right * speedScale));
            leftPower = (float) ((float) scaleInput(left * speedScale));

            // write the values to the motors
            motorR.setPower(rightPower);
            motorL.setPower(leftPower);
            sweeper.setPower(SW_power);
            shooter.setPower(SH_power);

            if (gamepad1.b && gamepad1.x) { // stop sweeper
                SW_power = (float) 0;
                sweeper.setPower(SW_power); // stop right away
                sleep(400); // make sure other botton reset
            } else if (gamepad1.b) { // sweeper backward
                SW_power = (float) 1.0;
                SH_power = (float) 0;
            } else if (gamepad1.x) { // sweeper forward
                SW_power = (float) -1.0;
                SH_power = (float) 0;
            }
            if (gamepad2.b) { // shooter on
                SW_power = (float) 0;
                if (SH_power<0.1)
                    SH_power = (float) 0.5;
                else
                    SH_power = (float) 1.0;
                sleep(400);
            } else if (gamepad2.x) { // shooter off
                SW_power = (float) 0;
                SH_power = (float) 0;
            }
            // update the speed of the chassis, or stop tape slider
            if (gamepad1.a) {
                // if the A button is pushed on gamepad1, decrease the speed
                // of the chassis
                if (speedScale > 0.2) {
                    speedScale -= 0.005;
                    sleep(5);
                }
            } else if (gamepad1.y) {
                // if the Y button is pushed on gamepad1, increase the speed
                // of the chassis
                if (speedScale < 1) {
                    speedScale += 0.005;
                    sleep(5);
                }
            }

            if (gamepad1.right_trigger > 0.1) { // *reconfigure later*

                sleep(300);
            }
            if (gamepad1.right_bumper) { //

            }

            if (gamepad1.left_trigger > 0.1) { // left climber down

                sleep(300);
            }
            if (gamepad1.left_bumper) { // left climber up

            }
            if (gamepad1.dpad_down) {

            }
            if (gamepad1.dpad_up) {

            }


            if (gamepad2.x || gamepad2.b) {

            }

            if (gamepad2.y) {

            }
            if (gamepad2.a) {

            }
            if (gamepad2.right_trigger > 0.1) {
                set_gate(GATE_CLOSED);
                sleep(5);
            }
            if (gamepad2.right_bumper) {
                set_gate(GATE_OPEN);
                //sleep(2000);
                //set_gate(GATE_CLOSED);
            }
            if (gamepad2.left_bumper) {
                set_pusher(PUSHER_UP);
                sleep(5);
            }
            if (gamepad2.left_trigger > 0.1) {
                set_pusher(PUSHER_EXTRA);
                sleep(1000);
                set_pusher(PUSHER_UP);
            }


                if (gamepad2.dpad_up) {
                    stop_chassis();
                    gamepad2.reset();
                } else if (gamepad2.dpad_left) {
                    if (Math.abs(left_beacon_sv_pos-LEFT_BEACON_INIT) < 0.1) {
                        set_left_beacon(LEFT_BEACON_PRESS);
                    }
                    else {
                        set_left_beacon(LEFT_BEACON_INIT);
                    }
                    sleep(500);
                } else if (gamepad2.dpad_right) {
                   if (Math.abs(right_beacon_sv_pos-RIGHT_BEACON_INIT) < 0.1) {
                        set_right_beacon(RIGHT_BEACON_PRESS);
                    }
                    else {
                       set_right_beacon(RIGHT_BEACON_INIT);
                   }
                    sleep(500);
                } else if (gamepad2.dpad_down) {
                    gamepad2.reset();
                    stop_chassis();
                }
                if (detectWhite()) {
                    detectwhite = 1;
                } else {
                    detectwhite = 0;
                }

                show_telemetry();
            }
            cdim.setDigitalChannelState(LED_CHANNEL, false);
        }
    }
