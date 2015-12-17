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

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class ArmTest extends OpMode {

	// CONSTANT VALUES.
	final static double ARM_MIN_RANGE  = 0.20;
	final static double ARM_MAX_RANGE  = 0.90;
	final static double CLAW_MIN_RANGE  = 0.20;
	final static double CLAW_MAX_RANGE  = 0.7;
	final static double THRESHOLD = 0.01;
	final static double SERVO_SCALE = 0.001;
	final static double GATE_CLOSED = 0.01;
	final static double GATE_OPEN = 0.8;
	final static double WRIST_UP = 0.87;
	final static double WRIST_COLLECT = 0.59;
	final static double SHOULDER_START = 0.5;
	final static double SHOULDER_TAPE_OUT = 0.46; // position to let tape out
	final static double SHOULDER_SCORE = 0.795;     // position to outside score position
	final static double SLIDER_LENGHTEN = 0.0;
	final static double SLIDER_SHORTEN = 1.0;
	final static double SLIDER_STOP = 0.5;
	final static double TAPE_ROTATE = 0.25;
	final static double TAPE_SLIDER = 0.75;
	final static double LIGHT_SENSOR_UP = 0.4;
	final static double LIGHT_SENSOR_DOWN = 0.9;
	final static double LEVELER_RIGHT = 0.0;
	final static double LEVELER_UP = 0.4;
	final static double LEVELER_LEFT = 0.9;



	// amount to change the arm servo position.
	double armDelta = 0.1;
 	int slider_counter = 0;

	// position of servos
	double shoulder_pos;
	double wrist_pos;
	double gate_pos;
	double slider_pos;
	double leveler_pos;
	double light_sensor_sv_pos;
	// amount to change the claw servo position by
	double arm_r;
	double arm_power;
	int elbow_pos;
	int tape_rotator_pos;
	int tape_slider_pos;
	int tape_count;
	double shoulder_dir;
	double wrist_dir;
	double gate_dir;
	double slider_dir;
	double elbow_dir;
	double tape_slider_dir;
	double tape_rotator_dir;
	int slider_curr_count;
	//double tape_rotation_pos;
	//double tape_slider_pos;
	DcMotor tape_rotator;
	DcMotor tape_slider;
	DcMotor elbow;
	Servo shoulder;
	Servo wrist;
	Servo gate;
	Servo arm_slider;
	Servo leveler;
	Servo light_sensor_sv;
	/**
	 * Constructor
	 */
	public ArmTest() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		arm_init();
	}

	public void arm_init() {
		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		v_warning_generated = false;
		v_warning_message = "Can't map; ";
		try {
			tape_slider = hardwareMap.dcMotor.get("tape_slider");
		}
		catch (Exception p_exeception)
		{
			m_warning_message ("tape_slider");
			DbgLog.msg (p_exeception.getLocalizedMessage ());
			tape_slider = null;
		}
		tape_slider.setDirection(DcMotor.Direction.REVERSE);

		try {
			tape_rotator = hardwareMap.dcMotor.get("tape_rotator");
		}
		catch (Exception p_exeception)
		{
			m_warning_message ("tape_rotator");
			DbgLog.msg (p_exeception.getLocalizedMessage ());
			tape_rotator = null;
		}

		try {
			elbow = hardwareMap.dcMotor.get("elbow");
		}
		catch (Exception p_exeception)
		{
			m_warning_message ("elbow");
			DbgLog.msg (p_exeception.getLocalizedMessage ());
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
		try {
			leveler = hardwareMap.servo.get("leveler");
		}
		catch (Exception p_exeception)
		{
			m_warning_message ("leveler");
			DbgLog.msg (p_exeception.getLocalizedMessage ());
			leveler = null;
		}
		try {
			light_sensor_sv = hardwareMap.servo.get("light_sensor_sv");
		}
		catch (Exception p_exeception)
		{
			m_warning_message ("light_sensor_sv");
			DbgLog.msg (p_exeception.getLocalizedMessage ());
			light_sensor_sv = null;
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
		leveler_pos = LEVELER_RIGHT;
		leveler.setPosition(leveler_pos);
		light_sensor_sv_pos = LIGHT_SENSOR_DOWN;
		light_sensor_sv.setPosition(light_sensor_sv_pos);
		arm_r = 0.25; // initial arm power
		arm_power = arm_r;
		shoulder_dir = 0;
		elbow_pos = elbow.getCurrentPosition();
		tape_rotator_pos = tape_rotator.getCurrentPosition();
		tape_slider_pos = tape_slider.getCurrentPosition();
		elbow_dir = 0;
		tape_slider_dir = 0;
		tape_rotator_dir = 0;
		tape_count = 0;
		wrist_dir = 0;
		gate_dir = 0;
		slider_dir = 0;
		slider_curr_count = 0;
		slider_counter = 0;
		//tape_rotator = 0;
		//tape_slider	= 0;
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {
		arm_loop();
	}
	public void arm_loop() {

		/*
		 * Gamepad 2
		 * 
		 * Gamepad 2 controls
		 *     shoulder via the right_stick_x
		 *     elbow via left_stick_y
		 *     wrist via left_trigger/left_bumper
		 *     gate via right_trigger/right_bumper
		 */

		elbow_pos = elbow.getCurrentPosition();
		tape_slider_pos = tape_slider.getCurrentPosition();
		tape_rotator_pos = tape_rotator.getCurrentPosition();
		shoulder_dir = -gamepad2.left_stick_x;
		elbow_dir = -gamepad2.right_stick_y;
		// gate_dir = gamepad2.right_stick_x;
		if (slider_counter>0)
			slider_counter --;
		else
			arm_slider.setPosition(SLIDER_STOP);

		if (tape_count>0)
			tape_count--;
		else
		    tape_rotator_dir = 0;

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
		if (gamepad2.right_trigger > 0.1) {
			gate_pos = GATE_OPEN;
		}
		if (gamepad2.right_bumper) {
			gate_pos = GATE_CLOSED;
		}
		// manual adjust gate position
		//if (gamepad2.dpad_right) { // gate servo down
		//	gate_pos -= SERVO_SCALE;
		//	if (gate_pos<0.1) gate_pos = 0.1;
		//}
		//if (gamepad2.dpad_left) { // gate servo up
		//	gate_pos += SERVO_SCALE;
		//	if (gate_pos>0.9) gate_pos = 0.9;
		//}
		gate.setPosition(gate_pos);

		if (gamepad2.left_trigger > 0.1) {
			wrist_pos = WRIST_COLLECT;
		}
		if (gamepad2.left_bumper) {
			wrist_pos = WRIST_UP;
		}
		// manual adjust wrist position
		if (true) {
			if (gamepad2.back) { // wrist servo down
				wrist_pos -= SERVO_SCALE;
				if (wrist_pos < 0.01) wrist_pos = 0.01;
			}
			if (gamepad2.start) { // wrist servo up
				wrist_pos += SERVO_SCALE;
				if (wrist_pos > 0.99) wrist_pos = 0.99;
			}
		}
		wrist.setPosition(wrist_pos);

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
		//control direction of tape slider and tape rotator
		// up to stop/slide out
		// down to stop/slide in
		// left to stop/rotate up
		// right to stop/rotate down
		if (gamepad2.dpad_up) {
			tape_slider_dir = 1;
		}
		else if (gamepad2.dpad_down) {
			tape_slider_dir = -1;
		}

		if (gamepad2.dpad_left) {
			tape_rotator_dir = 1;
			tape_count = 2;
		}
		else if (gamepad2.dpad_right) {
			tape_rotator_dir = -1;
			tape_count = 1;
		}

		if (gamepad2.back) { // stop tape
			tape_slider_dir = 0;
			tape_rotator_dir = 0;
		}

		// control to the elbow motor power
		if (elbow_dir < -THRESHOLD) { // arm down 20% of power
			elbow.setPower(-arm_power * 0.2);
		}
		else if (elbow_dir > THRESHOLD) { // arm up
			elbow.setPower(arm_power);
		} else {
			elbow.setPower(0);
		}
		if (tape_slider_dir < -THRESHOLD) { // tape slider in 100% power
			tape_slider.setPower(-1);
		}
		else if (tape_slider_dir > THRESHOLD) { // tape slider out 100% power
			tape_slider.setPower(1);
		} else {
			tape_slider.setPower(0);
		}
		if (tape_rotator_dir < -THRESHOLD) { // tape down 20% of power
			tape_rotator.setPower(-0.2);
		}
		else if (tape_rotator_dir > THRESHOLD) { // arm up 30% of power
			tape_rotator.setPower(0.3);
		} else {
			tape_rotator.setPower(0);
		}


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

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("shoulder", "pos(dir): " + String.format("%.4f (%.2f)", shoulder_pos, shoulder_dir));
        telemetry.addData("elbow", "pwr(pos): " + String.format("%.2f (%d)", arm_power,elbow_pos));
        telemetry.addData("wrist",  "pos(dir): " + String.format("%.2f (%.2f)", wrist_pos, wrist_dir));
        telemetry.addData("gate", "pos(dir): " + String.format("%.2f (%.2f)", gate_pos, gate_dir));
		telemetry.addData("arm_slider",  "pos(count): " + String.format("%.2f (%d)", slider_pos, slider_curr_count));
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

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
