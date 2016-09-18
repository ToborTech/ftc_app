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

/**
 * Linear Tele Op Mode
 * <p/>
 * Enables control of the robot via the gamepad.
 * NOTE: This op mode will not work with the NXT Motor Controllers. Use an Nxt op mode instead.
 */
public class TT_AutoRedIn extends TobotHardware {
    // CONSTANT VALUES.
    // CONSTANT VALUES.

    /**
     * Constructor
     */


    @Override
    public void runOpMode() throws InterruptedException {

        tobot_init(State.STATE_AUTO);

        waitForStart();

        auto_part1(true, true);

        auto_part2(true);

        //  StraightR(0.5,0.1);
        //  TurnRightD(0.5,90,true);
        //  StraightR(0.6,3.33);
        stop_tobot();
        //telemetry.addData("1. Red   = ", red_detected);
        //telemetry.addData("2. Blue  = ", blue_detected);
        //telemetry.addData("3. LL/LR = ", String.format("%.2f/%.2f", LL.getLightDetected(), LR.getLightDetected()));
        telemetry.addData("3. Heading go / cur", String.format("%d / %d", heading, gyro.getHeading()));
        telemetry.addData("4. ODS   = ", opSensor.getLightDetected());
        telemetry.addData("5. shoulder", "pos(dir): " + String.format("%.2f (%.2f)", shoulder_pos, shoulder_dir));
        telemetry.addData("6. elbow", "pwr(pos): " + String.format("%.2f (%d)", arm_power, elbow_pos));
        telemetry.addData("7. wrist", "pos LR/UD: " + String.format("%.2f / %.2f", wristLR_pos, wristUD_pos ));
        telemetry.addData("8. gate", "pos: " + String.format("%.2f", gate_pos));
        telemetry.addData("9. arm_slider", "pos(dir): " + String.format("%.2f (%.2f)", slider_pos, slider_dir));

    }



}
