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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * TobotHardware
 * <p>
 * Define all hardware (e.g. motors, servos, sensors) used by Tobot
 */
public class TobotHardware extends LinearOpMode {

	// CONSTANT VALUES.
	final static double ARM_MIN_RANGE  = 0.20;
	final static double ARM_MAX_RANGE  = 0.90;
	final static double CLAW_MIN_RANGE  = 0.20;
	final static double CLAW_MAX_RANGE  = 0.7;
	final static double THRESHOLD = 0.01;
	final static double SERVO_SCALE = 0.001;
	final static double GATE_CLOSED = 0.05;
	final static double GATE_OPEN = 0.91;
	final static double WRIST_UP = 0.98;
	final static double WRIST_COLLECT = 0.77;
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
	final static double LEVELER_RIGHT = 0.9;
	final static double LEVELER_DOWN = 0.5;
	final static double LEVELER_LEFT = 0.0;
	final static int    ONE_ROTATION = 1120; // for AndyMark motor encoder one rotation
	final static double  RROBOT = 11;  // number of wheel turns to get chassis 360-degree turn
	int numOpLoops = 1;

	//
	// following variables are used by Arm/slider
	//

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

	// variables for sensors
	ColorSensor coSensor;
	DeviceInterfaceModule cdim;
	TouchSensor tSensor;
	OpticalDistanceSensor opSensor;
	LightSensor LL, LR ;
	// IBNO055IMU imu;
	TT_Nav nav;
	TT_ColorPicker colorPicker;


	// following variables are used by Chassis
	State state;
	public enum State {
		STATE_TELEOP,	// state to test teleop
		STATE_AUTO,		// state to test auto routines
		STATE_TUNEUP    // state to manually tune up servo positions and arm positions
	}

	float speedScale = (float) 0.7; // controlling the speed of the chassis in teleOp state
	float leftPower = 0;
	float rightPower = 0;
	float SW_power = 0;
	double initAutoOpTime = 0;
	float currRaw = 0;
	DcMotor motorFR;
	DcMotor motorFL;
	DcMotor motorBR;
	DcMotor motorBL;
	DcMotor motorSW;
	int motorRightCurrentEncoder = 0;
	int motorLeftCurrentEncoder = 0;
	int motorRightTargetEncoder = 0;
	int motorLeftTargetEncoder = 0;
	int leftCnt = 0; // left motor target counter
	int rightCnt = 0; // right motor target counter

	public void tobot_init(State st) throws InterruptedException {
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
			m_warning_message("tape_slider");
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

		// initialize chassis variables
		motorFR = hardwareMap.dcMotor.get("motorFR");
		motorFL = hardwareMap.dcMotor.get("motorFL");
		motorBR = hardwareMap.dcMotor.get("motorBR");
		motorBL = hardwareMap.dcMotor.get("motorBL");
		motorSW = hardwareMap.dcMotor.get("motorSW");
		motorBL.setDirection(DcMotor.Direction.REVERSE);
		motorFL.setDirection(DcMotor.Direction.REVERSE);
		motorBL.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		motorBR.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		motorFL.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		motorFR.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		motorSW.setPower(0);
		motorFR.setPower(0);
		motorBR.setPower(0);
		motorFL.setPower(0);
		motorBL.setPower(0);

		state = st;

		reset_motors();
		stop_tobot();

		if (state==State.STATE_AUTO) {
			set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
		} else { // State.STATE_TELEOP
			set_drive_modes(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		}

		// initialize sensores
		cdim = hardwareMap.deviceInterfaceModule.get("dim");
		coSensor = hardwareMap.colorSensor.get("co");

		tSensor = hardwareMap.touchSensor.get("to");
		opSensor = hardwareMap.opticalDistanceSensor.get("op");

		LL = hardwareMap.lightSensor.get("ll");
		LR = hardwareMap.lightSensor.get("lr");

		//Instantiate ToborTech Nav object
		nav = new TT_Nav( motorFR, motorFL, opSensor, true , LL, LR); // Not using Follow line
		colorPicker = new TT_ColorPicker(coSensor);
	}

	@Override
	public void runOpMode() throws InterruptedException {

		tobot_init(State.STATE_TELEOP);

		waitForStart();

		while (opModeIsActive()) {

			if (gamepad1.back) { // change the state TeleOp->TuneUp->Auto
				if (state==State.STATE_AUTO)
					state = State.STATE_TELEOP;
				else if (state==State.STATE_TELEOP)
					state = State.STATE_TUNEUP;
				else
					state = State.STATE_AUTO;
				gamepad2.reset();
			}
			float right = gamepad1.left_stick_y;
			float left = gamepad1.right_stick_y;

			elbow_pos = elbow.getCurrentPosition();
			tape_slider_pos = tape_slider.getCurrentPosition();
			tape_rotator_pos = tape_rotator.getCurrentPosition();
			shoulder_dir = -gamepad2.left_stick_x;
			elbow_dir = -gamepad2.right_stick_y;
			// gate_dir = gamepad2.right_stick_x;
			// clip the right/left values so that the values never exceed +/- 1
			right = Range.clip(right, -1, 1);
			left = Range.clip(left, -1, 1);

			// scale the joystick value to make it easier to control
			// the robot more precisely at slower speeds.

			// Use speedScale to control the speed
			rightPower = (float) ((float) scaleInput(left * speedScale *-1));
			leftPower = (float) ((float) scaleInput(right * speedScale *-1));

			// write the values to the motors
			motorFR.setPower(rightPower);
			motorBR.setPower(rightPower);
			motorFL.setPower(leftPower);
			motorBL.setPower(leftPower);
			motorSW.setPower(SW_power);
			if (gamepad1.dpad_down) { // backward 2-rotation
				StraightR(-0.8, 2);
			}
			if (gamepad1.dpad_up) { //forward 2 rotation
				StraightR(0.8, 2);
			}
			if (gamepad1.dpad_left) { //left spot turn 90 Degrees
				TurnLeftD(0.95, 90, true);
			}
			if (gamepad1.dpad_right) { //right spot turn 90 Degrees
				TurnRightD(0.95, 90, true);
			}
			if (gamepad1.right_trigger>0.1) { // Sweeper Forward
				SW_power = (float)-1.0;
			}
			if (gamepad1.right_bumper) { // Sweeper Backward
				SW_power = (float)1.0;
			}
			if (gamepad1.x) { // stop sweeper
				SW_power = (float) 0;
			}
			// update the speed of the chassis
			if (gamepad1.a) {
				// if the A button is pushed on gamepad1, decrease the speed
				// of the chassis
				if (speedScale > 0.1)
					speedScale -= 0.01;
			}
			if (gamepad1.y) {
				// if the Y button is pushed on gamepad1, increase the speed
				// of the chassis
				if (speedScale < 1)
					speedScale += 0.01;
			}

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
			if (state==State.STATE_TUNEUP) {
				// manual adjust wrist position
				if (gamepad2.left_trigger>0.1) { // wrist servo down
					wrist_pos -= SERVO_SCALE;
					if (wrist_pos < 0.01) wrist_pos = 0.01;
				}
				if (gamepad2.left_bumper) { // wrist servo up
					wrist_pos += SERVO_SCALE;
					if (wrist_pos > 0.99) wrist_pos = 0.99;
				}
				wrist.setPosition(wrist_pos);
				// manual adjust gate position
				if (gamepad2.right_trigger>0.1) { // gate servo down
					gate_pos -= SERVO_SCALE;
					if (gate_pos<0.01) gate_pos = 0.01;
				}
				if (gamepad2.right_bumper) { // gate servo up
					gate_pos += SERVO_SCALE;
					if (gate_pos>0.99) gate_pos = 0.99;
				}
				gate.setPosition(gate_pos);
			} else if (state==State.STATE_TELEOP) {
				if (gamepad2.right_trigger > 0.1) {
					gate_pos = GATE_OPEN;
				}
				if (gamepad2.right_bumper) {
					gate_pos = GATE_CLOSED;
				}
				gate.setPosition(gate_pos);

				if (gamepad2.left_trigger > 0.1) {
					wrist_pos = WRIST_COLLECT;
				}
				if (gamepad2.left_bumper) {
					wrist_pos = WRIST_UP;
				}
				wrist.setPosition(wrist_pos);
			} else { // Auto state, test the arm routines
				// release arm
				if (gamepad2.right_bumper) {
					//release_arm();
				}
				if (gamepad2.right_trigger > 0.1) {
					//arm_collection_mode();
				}
				if (gamepad2.left_bumper) {
					//open_gate();
				}
				if (gamepad2.left_trigger > 0.1) {
					//close_gate();
				}

				if (gamepad2.dpad_up) {
					//slider_out();
				}
				if (gamepad2.dpad_down) {
					//slider_in();
				}
			}
			show_telemetry();
			waitOneFullHardwareCycle();
		}
	}

	public void show_telemetry() {
		telemetry.addData("0. State: ", state.toString());
		telemetry.addData("1. shoulder:", "pos= " + String.format("%.2f, dir=%.2f)", shoulder_pos, shoulder_dir));
		telemetry.addData("2. elbow:", "pwr= " + String.format("%.2f, pos= %d", arm_power,elbow_pos));
		telemetry.addData("3. wrist/gate",  "pos= " + String.format("%.2f / %.2f", wrist_pos, gate_pos));
		telemetry.addData("4. arm_slider",  "pos (dir): " + String.format("%.2f (%.2f)", slider_pos, slider_dir));
		telemetry.addData("5. tape_rotator",  "pos= " + String.format("%2d", tape_rotator_pos));
		telemetry.addData("6. drive power: L=", String.format("%.2f", leftPower) + "/R=" + String.format("%.2f", rightPower) + "(mode=" + motorFR.getMode().toString() + ")");
		//telemetry.addData("7. left  cur/tg enc:", motorBL.getCurrentPosition() + "/" + motorBL.getTargetPosition());
		//telemetry.addData("8. right cur/tg enc:", motorFR.getCurrentPosition() + "/" + motorFR.getTargetPosition());
		telemetry.addData("7. left  cur/tg enc:", motorBL.getCurrentPosition() + "/" + leftCnt);
		telemetry.addData("8. right cur/tg enc:", motorFR.getCurrentPosition() + "/" + rightCnt);
		telemetry.addData("9. ods:", String.format("%.2f",opSensor.getLightDetected()));
	}

	public void StraightR(double power, double n_rotations) throws InterruptedException {
		reset_chassis();
		set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
		int leftEncode = motorBL.getCurrentPosition();
		int rightEncode = motorFR.getCurrentPosition();
		initAutoOpTime = this.time;
		//motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		//motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		leftCnt = (int) (ONE_ROTATION * n_rotations);
		rightCnt = (int) (ONE_ROTATION * n_rotations);
		leftPower = rightPower = (float) power;
		if (power < 0) { // move backward
			leftCnt = leftEncode - leftCnt;
			rightCnt = rightEncode - rightCnt;
		} else {
			leftCnt += leftEncode;
			rightCnt += rightEncode;
		}
		run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
	}

	public void run_until_encoder(int leftCnt, double leftPower, int rightCnt, double rightPower) throws InterruptedException {
		//motorFR.setTargetPosition(rightCnt);
		//motorBL.setTargetPosition(leftCnt);
		//motorBL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
		//motorFR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
		//waitOneFullHardwareCycle();
		motorFR.setPower(rightPower);
		motorFL.setPower(leftPower);
		motorBR.setPower(rightPower);
		motorBL.setPower(leftPower);
		waitOneFullHardwareCycle();
		//while (motorFR.isBusy() || motorBL.isBusy()) {
		while (!have_drive_encoders_reached(leftCnt,rightCnt)) {
			motorFR.setPower(rightPower);
			motorFL.setPower(leftPower);
			motorBR.setPower(rightPower);
			motorBL.setPower(leftPower);
			show_telemetry();
			waitOneFullHardwareCycle();
		}
		stop_chassis();
		if (state==State.STATE_AUTO) {
			set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
		} else {
			set_drive_modes(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		}
		// waitOneFullHardwareCycle();
	}

	public void TurnLeftD(double power, int degree, boolean spotTurn) throws InterruptedException {
		initAutoOpTime = getRuntime();
		reset_chassis();
		set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
		//motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		//motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		int leftEncode = motorBL.getCurrentPosition();
		int rightEncode = motorFR.getCurrentPosition();
		if (spotTurn) { // use both motors for spot turn
			leftCnt = (int) (-ONE_ROTATION * RROBOT * degree / 720.0);
			rightCnt = (int) (ONE_ROTATION * RROBOT * degree / 720.0);
			leftPower = (float) -power;
		} else { // swing turn. only use right motor
			leftCnt = 0;
			rightCnt = (int) (ONE_ROTATION * RROBOT * degree / 360.0);
			leftPower = (float) 0;
		}
		leftCnt += leftEncode;
		rightCnt += rightEncode;
		rightPower = (float) power;

		run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
	}

	public void TurnRightD(double power, int degree, boolean spotTurn) throws InterruptedException {
		initAutoOpTime = getRuntime();
		reset_chassis();
		set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
		//motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		//motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		int leftEncode = motorBL.getCurrentPosition();
		int rightEncode = motorFR.getCurrentPosition();
		if (spotTurn) { // use both motors for spot turn
			leftCnt = (int) (ONE_ROTATION * RROBOT * degree / 720.0);
			rightCnt = (int) (-ONE_ROTATION * RROBOT * degree / 720.0);
			rightPower = (float) -power;
		} else { // swing turn. only use right motor
			leftCnt = 0;
			rightCnt = (int) (ONE_ROTATION * RROBOT * degree / 360.0);
			rightPower = (float) 0;
		}
		leftCnt += leftEncode;
		rightCnt += rightEncode;
		leftPower = (float) power;
		run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
	}

	void set_drive_modes(DcMotorController.RunMode mode) {
		motorBL.setMode(mode);
		motorBR.setMode(mode);
		motorFL.setMode(mode);
		motorFR.setMode(mode);
	}

	void stop_chassis() {
		motorBL.setPower(0);
		motorBR.setPower(0);
		motorFL.setPower(0);
		motorFR.setPower(0);
	}

	void stop_tobot() {
		stop_chassis();
		motorSW.setPower(0);
		elbow.setPower(0);
	}

	void reset_chassis() throws InterruptedException {
		motorBL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorBR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorFR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		while (motorFR.getCurrentPosition()!=0 && motorBL.getCurrentPosition()!=0) {
				// && motorBR.getCurrentPosition()!=0) && motorFL.getCurrentPosition()!=0) {
			waitOneFullHardwareCycle();
		}
		leftCnt = 0; rightCnt = 0;
	}
	void reset_motors() throws InterruptedException {
		reset_chassis();
		elbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		tape_rotator.setMode(DcMotorController.RunMode.RESET_ENCODERS);

		// tape_slider and motorSW are not using encoder for now
	}

	boolean has_left_drive_encoder_reached (double p_count) {
		if (leftPower<0) {
			//return (Math.abs(motorFL.getCurrentPosition()) < p_count);
			return (motorBL.getCurrentPosition() <= p_count);
		} else {
			//return (Math.abs(motorFL.getCurrentPosition()) > p_count);
			return (motorBL.getCurrentPosition() >= p_count);
		}
	} // has_left_drive_encoder_reached

	//--------------------------------------------------------------------------
	//
	// has_right_drive_encoder_reached
	//
	/**
	 * Indicate whether the right drive motor's encoder has reached a value.
	 */
	boolean has_right_drive_encoder_reached (double p_count) {
		if (rightPower<0) {
			return (motorFR.getCurrentPosition() < p_count);
		} else {
			return (motorFR.getCurrentPosition() > p_count);
		}

	} // has_right_drive_encoder_reached

	//--------------------------------------------------------------------------
	//
	// have_drive_encoders_reached
	//
	/**
	 * Indicate whether the drive motors' encoders have reached specified values.
	 */
	boolean have_drive_encoders_reached( double p_left_count, double p_right_count) {
		boolean l_return = false;
		if (has_left_drive_encoder_reached (p_left_count) ||
				has_right_drive_encoder_reached (p_right_count)) {
			l_return = true;
		}
		return l_return;
	} // have_encoders_reached

	double scaleInput(double dVal) {
		double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		if (index < 0) {
			index = -index;
		} else if (index > 16) {
			index = 16;
		}

		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}
		return dScale;
	}

	void leveler_right() {
		leveler_pos = LEVELER_RIGHT;
		leveler.setPosition(leveler_pos);
	}

	void leveler_left() {
		leveler_pos = LEVELER_LEFT;
		leveler.setPosition(leveler_pos);
	}
	void leveler_down(){
		leveler_pos = LEVELER_DOWN;
		leveler.setPosition(leveler_pos);
	}

	void hit_right_button() throws InterruptedException {
		leveler_right();
		TurnLeftD(0.3, 10, false);
		wait(100);
		TurnLeftD(-0.3, 10, false);
		wait(100);
	}

	void hit_left_button() throws InterruptedException {
		leveler_left();
		TurnRightD(0.3, 10, false);
		wait(100);
		TurnRightD(-0.3, 10, false);
		wait(100);
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
