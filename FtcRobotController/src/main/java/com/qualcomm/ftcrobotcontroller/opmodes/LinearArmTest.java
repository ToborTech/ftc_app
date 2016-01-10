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
import com.qualcomm.robotcore.util.Range;

/**
 * Linear Tele Op Mode
 * <p>
 * Enables control of the robot via the gamepad.
 * NOTE: This op mode will not work with the NXT Motor Controllers. Use an Nxt op mode instead.
 */
public class LinearArmTest extends TobotHardware {

  @Override
  public void runOpMode() throws InterruptedException {

    tobot_init(State.STATE_TELEOP);

    waitForStart();

    while (opModeIsActive()) {
      shoulder_dir = -gamepad2.left_stick_x;
      elbow_dir = -gamepad2.right_stick_y;

      if (shoulder_dir > THRESHOLD) {
        shoulder_pos += (SERVO_SCALE/3.0);
        if (shoulder_pos > 1) {
          shoulder_pos = 0.99;
        }
      } else if (shoulder_dir < THRESHOLD * -1) {
        shoulder_pos -= (SERVO_SCALE/3.0);
        if (shoulder_pos < 0) {
          shoulder_pos = 0.01;
        }
      }
      shoulder.setPosition(shoulder_pos);

      if (elbow_dir < -THRESHOLD) { // arm down 20% of power
        elbow.setPower(-arm_power * 0.2);
      }
      else if (elbow_dir > THRESHOLD) { // arm up
        elbow.setPower(arm_power);
      } else {
        elbow.setPower(0);
      }

      // release arm
        if (gamepad2.right_bumper) {
          if (arm_state==ArmState.ARM_INIT)
            //release_arm();
          climber_mission(true);
          else if (arm_state==ArmState.ARM_COLLECT)
            arm_collect_mode_to_up_back();
        }

      if (gamepad2.right_trigger > 0.1) {
        if (arm_state==ArmState.ARM_UP_BACK)
          arm_collection_mode();

      }

      if (gamepad2.left_bumper) {
        open_gate();
      }
      if (gamepad2.x) {
        arm_slider_in_for_n_sec(0.1);
      }
      if (gamepad2.b) {
        arm_slider_out_for_n_sec(0.1);
      }
      if (gamepad2.left_trigger > 0.1) {
        close_gate();
      }
      if (gamepad2.y) {
        if (arm_state==ArmState.ARM_UP_BACK)
           arm_front();
        else if (arm_state==ArmState.ARM_DOWN_FRONT)
          go_red_mid_zone();
        else if (arm_state==ArmState.ARM_DOWN_BACK)
          arm_collection_mode();
      }
      if (gamepad2.a) {
         if (arm_state==ArmState.ARM_UP_FRONT)
            arm_back();
         else if (arm_state==ArmState.ARM_SCORE_MID_RED)
           arm_back_from_goal();
      }

      show_telemetry();


      //telemetry.addData("shoulder", "pos(dir): " + String.format("%.2f (%.2f)", shoulder_pos, shoulder_dir));
      //telemetry.addData("elbow", "pwr(pos): " + String.format("%.2f (%d)", arm_power, elbow_pos));
      //telemetry.addData("wrist", "pos(dir): " + String.format("%.2f (%.2f)", wrist_pos, wrist_dir));
      //telemetry.addData("gate", "pos(dir): " + String.format("%.2f (%.2f)", gate_pos, gate_dir));
      //telemetry.addData("arm_slider", "pos(dir): " + String.format("%.2f (%.2f)", slider_pos, slider_dir));

      waitOneFullHardwareCycle();
    }
  }
}

