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

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Linear Tele Op Mode
 * <p/>
 * Enables control of the robot via the gamepad.
 * NOTE: This op mode will not work with the NXT Motor Controllers. Use an Nxt op mode instead.
 */
public class LinearAutoRedOut extends TobotHardware {
    // CONSTANT VALUES.
    // CONSTANT VALUES.

    /**
     * Constructor
     */


    @Override
    public void runOpMode() throws InterruptedException {

        tobot_init(State.STATE_AUTO);

        waitForStart();


        StraightR(0.75, 8);

        //StraightR(-0.5, 2);
        TurnRightD(0.5, 35, true);
        //StraightR(0.5, 1);
        goUntilWhite(-0.15);
        TurnLeftD(0.5,90, true);

        // Follow line until optical distance sensor detect 0.2 value to the wall (about 6cm)
        followLineTillOp(0.03, true);
        hit_left_button();
        sleep(1000);
        hit_right_button();
        sleep(1000);
        leveler_down();

        // Detect Beacon color and hit the right side
        if (colorPicker.getColor()==TT_ColorPicker.BLUE) {
            hit_left_button();
        } else if (colorPicker.getColor()==TT_ColorPicker.RED) {
            hit_right_button();
        } else { // unknown, better not do anything than giving the credit to the opponent
            // doing nothing. May print out the message for debugging
        }

        // dump two climbers, please Kevin!!
        sleep(5000);

        //  StraightR(0.5,0.1);
        //  TurnRightD(0.5,90,true);
        //  StraightR(0.6,3.33);
        stop_tobot();

        telemetry.addData("shoulder", "pos(dir): " + String.format("%.2f (%.2f)", shoulder_pos, shoulder_dir));
        telemetry.addData("elbow", "pwr(pos): " + String.format("%.2f (%d)", arm_power, elbow_pos));
        telemetry.addData("wrist", "pos(dir): " + String.format("%.2f (%.2f)", wrist_pos, wrist_dir));
        telemetry.addData("gate", "pos(dir): " + String.format("%.2f (%.2f)", gate_pos, gate_dir));
        telemetry.addData("arm_slider", "pos(dir): " + String.format("%.2f (%.2f)", slider_pos, slider_dir));

        waitOneFullHardwareCycle();

    }

}
