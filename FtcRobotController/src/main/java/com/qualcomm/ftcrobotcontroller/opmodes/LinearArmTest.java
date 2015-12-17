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
public class LinearArmTest extends LinearOpMode {
  // CONSTANT VALUES.
  // CONSTANT VALUES.
  final static double ARM_MIN_RANGE  = 0.20;
  final static double ARM_MAX_RANGE  = 0.90;
  final static double CLAW_MIN_RANGE  = 0.20;
  final static double CLAW_MAX_RANGE  = 0.7;
  final static double THRESHOLD = 0.01;
  final static double SERVO_SCALE = 0.001;
  final static double GATE_CLOSED = 0.01;
  final static double GATE_OPEN = 0.8;
  final static double WRIST_UP = 0.99;
  final static double WRIST_COLLECT = 0.82;
  final static double SHOULDER_START = 0.5;
  final static double SHOULDER_TAPE_OUT = 0.46; // position to let tape out
  final static double SHOULDER_SCORE = 0.795;     // position to outside score position
  final static double SLIDER_LENGHTEN = 0.0;
  final static double SLIDER_SHORTEN = 1.0;
  final static double SLIDER_STOP = 0.5;
  

  // amount to change the arm servo position.
  double armDelta = 0.1;
  int slider_counter = 0;

  // position of servos
  double shoulder_pos;
  double wrist_pos;
  double gate_pos;
  double slider_pos;
  // amount to change the claw servo position by
  double arm_r;
  double arm_power;
  int elbow_pos;
  double shoulder_dir;
  double wrist_dir;
  double gate_dir;
  double slider_dir;
  double elbow_dir;


  DcMotor elbow;
  Servo shoulder;
  Servo wrist;
  Servo gate;
  Servo arm_slider;
  /**
   * Constructor
   */

  public void init_robot() {
    v_warning_generated = false;
    v_warning_message = "Can't map; ";
    try {
      elbow = hardwareMap.dcMotor.get("elbow");
    }
    catch (Exception p_exeception)
    {
      m_warning_message ("elbow");
      DbgLog.msg(p_exeception.getLocalizedMessage());
      elbow = null;
    }

    //elbow.setDirection(DcMotor.Direction.REVERSE);
    try {
      shoulder = hardwareMap.servo.get("shoulder");
    }
    catch (Exception p_exeception)
    {
      m_warning_message ("shoulder");
      DbgLog.msg (p_exeception.getLocalizedMessage ());
      shoulder = null;
    }
    try {
      gate = hardwareMap.servo.get("gate");
    }
    catch (Exception p_exeception)
    {
      m_warning_message ("gate");
      DbgLog.msg (p_exeception.getLocalizedMessage ());
      gate = null;
    }
    try {
      wrist = hardwareMap.servo.get("wrist");
    }
    catch (Exception p_exeception)
    {
      m_warning_message ("wrist");
      DbgLog.msg (p_exeception.getLocalizedMessage ());
      wrist = null;
    }
    try {
      arm_slider = hardwareMap.servo.get("arm_slider");
    }
    catch (Exception p_exeception)
    {
      m_warning_message ("arm_slider");
      DbgLog.msg (p_exeception.getLocalizedMessage ());
      arm_slider = null;
    }
    // assign the starting position of the wrist and claw
    shoulder_pos = SHOULDER_START;
    shoulder.setPosition(shoulder_pos);
    wrist_pos = WRIST_UP;
    wrist.setPosition(wrist_pos);
    gate_pos = GATE_CLOSED;
    gate.setPosition(gate_pos);
    slider_pos = SLIDER_STOP;
    arm_slider.setPosition(slider_pos);
    arm_r = 0.25; // initial arm power
    arm_power = arm_r;
    shoulder_dir = 0;
    elbow_pos = elbow.getCurrentPosition();
    elbow_dir = 0;
    wrist_dir = 0;
    gate_dir = 0;
    slider_dir = 0;
    slider_counter = 0;
  }

  @Override
  public void runOpMode() throws InterruptedException {

    init_robot();

    waitForStart();

    while (opModeIsActive()) {
      // throttle:  left_stick_y ranges from -1 to 1, where -1 is full up,  and 1 is full down
      // direction: left_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
      float throttle  = -gamepad2.left_stick_y;
      float direction =  gamepad2.left_stick_x;
      float right = throttle - direction;
      float left  = throttle + direction;

      // clip the right/left values so that the values never exceed +/- 1

      // release arm
      if (gamepad2.right_bumper) {
        release_arm();
      }
      if (gamepad2.right_trigger > 0.1) {
        arm_collection_mode();
      }
      if (gamepad2.left_bumper) {
        open_gate();
      }
      if (gamepad2.left_trigger > 0.1) {
        close_gate();
      }

      if (gamepad2.dpad_up) {
        slider_out();
      }
      if (gamepad2.dpad_down) {
        slider_in();
      }




      telemetry.addData("shoulder", "pos(dir): " + String.format("%.2f (%.2f)", shoulder_pos, shoulder_dir));
      telemetry.addData("elbow", "pwr(pos): " + String.format("%.2f (%d)", arm_power,elbow_pos));
      telemetry.addData("wrist",  "pos(dir): " + String.format("%.2f (%.2f)", wrist_pos, wrist_dir));
      telemetry.addData("gate", "pos(dir): " + String.format("%.2f (%.2f)", gate_pos, gate_dir));
      telemetry.addData("arm_slider",  "pos(dir): " + String.format("%.2f (%.2f)", slider_pos, slider_dir));

      waitOneFullHardwareCycle();
    }
  }

  void release_arm() {

  }
  void arm_collection_mode() {

  }
  void open_gate() {

  }
  void close_gate() {

  }
  void slider_out() {

  }
  void slider_in() {

  }
  void m_warning_message (String p_exception_message)

  {
    if (v_warning_generated)
    {
      v_warning_message += ", ";
    }
    v_warning_generated = true;
    v_warning_message += p_exception_message;

  }
  private boolean v_warning_generated = false;
  private String v_warning_message;
}
