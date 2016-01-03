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
 * <p/>
 * Define all hardware (e.g. motors, servos, sensors) used by Tobot
 */
public class TobotHardware extends LinearOpMode {

    // CONSTANT VALUES.
    final static double ARM_MIN_RANGE = 0.20;
    final static double ARM_MAX_RANGE = 0.90;
    final static double CLAW_MIN_RANGE = 0.20;
    final static double CLAW_MAX_RANGE = 0.7;
    final static double THRESHOLD = 0.01;
    final static double SERVO_SCALE = 0.001;
    final static double GATE_CLOSED = 0.05;
    final static double GATE_OPEN = 0.91;
    final static double WRIST_UP = 0.56;
    final static double WRIST_MID = 0.4;
    final static double WRIST_CLIMBER = 0.15;
    final static double WRIST_COLLECT = 0.11;
    final static double SHOULDER_START = 0.5115;
    final static double SHOULDER_TAPE_OUT = 0.46; // position to let tape out
    final static double SHOULDER_SCORE = 0.8;     // position to outside score position
    final static double SHOULDER_RED_MID_SCORE = 0.86; //position for scoring mid red zone basket
    final static double SHOULDER_RED_HIGH_SCORE = 0.86; //position for scoring high red zone basket
    final static double SLIDER_LENGHTEN = 0.0;
    final static double SLIDER_SHORTEN = 1.0;
    final static double SLIDER_STOP = 0.5;
    final static double TAPE_ROTATE = 0.25;
    final static double TAPE_SLIDER = 0.75;
    final static double LIGHT_SENSOR_UP = 0.46;
    final static double LIGHT_SENSOR_DOWN = 0.84;
    final static double LEVELER_RIGHT = 1;
    final static double LEVELER_DOWN = 0.45;
    final static double LEVELER_LEFT = 0.0;
    final static double RIGHT_CLIMBER_UP = 0.75;
    final static double RIGHT_CLIMBER_MID = 0.5;
    final static double RIGHT_CLIMBER_LOW = 0.2;
    final static double LEFT_CLIMBER_UP = 0.2;
    final static double LEFT_CLIMBER_MID = 0.65;
    final static double LEFT_CLIMBER_LOW = 0.8;
    final static double WHITE_MIN = 0.5;
    final static double WHITE_MAX = 0.75;

    final static int ONE_ROTATION = 1120; // for AndyMark motor encoder one rotation
    final static double RROBOT = 11;  // number of wheel turns to get chassis 360-degree turn
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
    double climberL_pos;
    double climberR_pos;
    // amount to change the claw servo position by
    double cur_arm_power;
    double arm_power;
    int elbow_pos;
    int elbow_count;
    int tape_rotator_pos;
    int tape_slider_pos;
    int tape_count;
    double shoulder_dir;
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
    Servo climberL;
    Servo climberR;

    // variables for sensors
    ColorSensor coSensor;
    DeviceInterfaceModule cdim;
    TouchSensor tSensor;
    OpticalDistanceSensor opSensor;
    LightSensor LL, LR;
    // IBNO055IMU imu;
    TT_Nav nav;
    TT_ColorPicker colorPicker;


    // following variables are used by Chassis
    State state;
    ArmState arm_state;

    public enum State {
        STATE_TELEOP,    // state to test teleop
        STATE_AUTO,        // state to test auto routines
        STATE_TUNEUP    // state to manually tune up servo positions and arm positions
    }

    public enum ArmState {
        ARM_INIT,       // arm at initial position
        ARM_UP_BACK,    // arm at up back position
        ARM_UP_FRONT,   // arm at up front position
        ARM_COLLECT,    // arm at collection position
        ARM_DOWN_BACK,  // arm at back down position
        ARM_DOWN_FRONT, // arm at front down position
        ARM_SCORE_HIGH_RED,
        ARM_SCORE_MID_RED,
        ARM_SCORE_LOW_RED,
        ARM_SCORE_HIGH_BLUE,
        ARM_SCORE_MID_BLUE,
        ARM_SCORE_LOW_BLUE
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
        } catch (Exception p_exeception) {
            m_warning_message("tape_slider");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            tape_slider = null;
        }
        tape_slider.setDirection(DcMotor.Direction.REVERSE);

        try {
            tape_rotator = hardwareMap.dcMotor.get("tape_rotator");
        } catch (Exception p_exeception) {
            m_warning_message("tape_rotator");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            tape_rotator = null;
        }

        try {
            elbow = hardwareMap.dcMotor.get("elbow");
        } catch (Exception p_exeception) {
            m_warning_message("elbow");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            elbow = null;
        }
        //elbow.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        //elbow.setDirection(DcMotor.Direction.REVERSE);
        try {
            shoulder = hardwareMap.servo.get("shoulder");
        } catch (Exception p_exeception) {
            m_warning_message("shoulder");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            shoulder = null;
        }
        try {
            gate = hardwareMap.servo.get("gate");
        } catch (Exception p_exeception) {
            m_warning_message("gate");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            gate = null;
        }
        try {
            wrist = hardwareMap.servo.get("wrist");
        } catch (Exception p_exeception) {
            m_warning_message("wrist");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            wrist = null;
        }
        try {
            arm_slider = hardwareMap.servo.get("arm_slider");
        } catch (Exception p_exeception) {
            m_warning_message("arm_slider");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            arm_slider = null;
        }
        try {
            leveler = hardwareMap.servo.get("leveler");
        } catch (Exception p_exeception) {
            m_warning_message("leveler");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            leveler = null;
        }
        try {
            climberR = hardwareMap.servo.get("climberR");
        } catch (Exception p_exeception) {
            m_warning_message("climberR");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            climberR = null;
        }
        try {
            climberL = hardwareMap.servo.get("climberL");
        } catch (Exception p_exeception) {
            m_warning_message("climberL");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            climberL = null;
        }
        try {
            light_sensor_sv = hardwareMap.servo.get("light_sensor_sv");
        } catch (Exception p_exeception) {
            m_warning_message("light_sensor_sv");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            light_sensor_sv = null;
        }
        set_shoulder_pos(SHOULDER_START);
        wrist_pos = WRIST_UP;
        wrist.setPosition(wrist_pos);
        gate_pos = GATE_CLOSED;
        gate.setPosition(gate_pos);
        slider_pos = SLIDER_STOP;
        arm_slider.setPosition(slider_pos);
        leveler_pos = LEVELER_DOWN;
        leveler.setPosition(leveler_pos);
        light_sensor_sv_pos = LIGHT_SENSOR_DOWN;
        light_sensor_sv.setPosition(light_sensor_sv_pos);
        set_right_climber(RIGHT_CLIMBER_UP);
        set_left_climber(LEFT_CLIMBER_UP);
        arm_power = 0.25;
        cur_arm_power = 0;
        shoulder_dir = 0;
        elbow_pos = elbow.getCurrentPosition();
        tape_rotator_pos = tape_rotator.getCurrentPosition();
        tape_slider_pos = tape_slider.getCurrentPosition();
        elbow_dir = 0;
        elbow_count = 0;
        tape_slider_dir = 0;
        tape_rotator_dir = 0;
        tape_count = 0;
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
        arm_state = ArmState.ARM_INIT;

        reset_motors();
        stop_tobot();
        //if (state == State.STATE_TELEOP) {
        //    set_light_sensor(LIGHT_SENSOR_UP);
        //}
        if (state == State.STATE_AUTO || state == State.STATE_TELEOP) {
            set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        } else { // State.STATE_TUNE
            set_drive_modes(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }

        // initialize sensores
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        coSensor = hardwareMap.colorSensor.get("co");

        //tSensor = hardwareMap.touchSensor.get("to");
        opSensor = hardwareMap.opticalDistanceSensor.get("op");

        LL = hardwareMap.lightSensor.get("ll");
        LR = hardwareMap.lightSensor.get("lr");

        //Instantiate ToborTech Nav object
        nav = new TT_Nav(motorFR, motorBR, motorFL, motorBL, opSensor, true, LL, LR); // Not using Follow line
        colorPicker = new TT_ColorPicker(coSensor);
    } // end of tobot_init

    @Override
    public void runOpMode() throws InterruptedException {

        tobot_init(State.STATE_TELEOP);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.back) { // change the state TeleOp->TuneUp->Auto
                if (state == State.STATE_AUTO)
                    state = State.STATE_TELEOP;
                else if (state == State.STATE_TELEOP)
                    state = State.STATE_TUNEUP;
                else
                    state = State.STATE_AUTO;
                gamepad2.reset();
                sleep(500);
            }
            float left = -gamepad1.left_stick_y;
            float right = -gamepad1.right_stick_y;

            elbow_pos = elbow.getCurrentPosition();
            tape_slider_pos = tape_slider.getCurrentPosition();
            tape_rotator_pos = tape_rotator.getCurrentPosition();
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
                    StraightR(-0.8, 2);
                }
                if (gamepad1.dpad_up) { //forward 2 rotation
                    StraightR(0.8, 2);
                }
                if (gamepad1.dpad_left) { //left spot turn 90 Degrees
                    TurnLeftD(0.8, 90, true);
                }
                if (gamepad1.dpad_right) { //right spot turn 90 Degrees
                    TurnRightD(0.8, 90, true);
                }
            }
            if (gamepad1.b && gamepad1.x) { // stop sweeper
                SW_power = (float) 0;
                motorSW.setPower(SW_power); // stop right away
                sleep(400); // make sure other botton reset
            }
            else if (gamepad1.b) { // sweeper backward
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
            }
            else if (gamepad1.y) {
                // if the Y button is pushed on gamepad1, increase the speed
                // of the chassis
                if (speedScale < 1)
                    speedScale += 0.01;
            }

            if (gamepad1.right_trigger>0.1) { // right climber down: mid then low
                if (Math.abs(climberR_pos-RIGHT_CLIMBER_MID)<0.05) {
                    set_right_climber(RIGHT_CLIMBER_LOW);
                } else {
                    set_right_climber(RIGHT_CLIMBER_MID);
                }
                sleep(500);
            }
            if (gamepad1.right_bumper) { // right climber up
                set_right_climber(RIGHT_CLIMBER_UP);
            }
            if (gamepad1.left_trigger>0.1) { // left climber down
                if (Math.abs(climberL_pos-LEFT_CLIMBER_MID)<0.05) {
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
            } else if (gamepad1.dpad_up) {
                tape_slider_dir = 1;
            } else if (gamepad1.dpad_down) {
                tape_slider_dir = -1;
            }

            if (gamepad1.dpad_left) {
                tape_rotator_dir = 1;
                tape_count = 2;
            } else if (gamepad1.dpad_right) {
                tape_rotator_dir = -1;
                tape_count = 1;
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
                tape_rotator.setPower(-0.2);
            } else if (tape_rotator_dir > THRESHOLD) { // arm up 30% of power
                tape_rotator.setPower(0.3);
            } else {
                tape_rotator.setPower(0);
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
            if (state == State.STATE_TUNEUP) {
                // manual adjust wrist position
                if (gamepad2.left_trigger > 0.1) { // wrist servo down
                    wrist_pos -= SERVO_SCALE;
                    if (wrist_pos < 0.01) wrist_pos = 0.01;
                }
                if (gamepad2.left_bumper) { // wrist servo up
                    wrist_pos += SERVO_SCALE;
                    if (wrist_pos > 0.99) wrist_pos = 0.99;
                }
                wrist.setPosition(wrist_pos);
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
            } else if (state == State.STATE_TELEOP) {
                if (gamepad2.right_trigger > 0.1) {
                    gate_pos = GATE_OPEN;
                }
                if (gamepad2.right_bumper) {
                    gate_pos = GATE_CLOSED;
                }
                gate.setPosition(gate_pos);

                if (gamepad2.left_trigger > 0.1) {
                    set_wrist_pos(WRIST_COLLECT);
                }
                if (gamepad2.left_bumper) {
                    set_wrist_pos(WRIST_UP);
                }
                if (gamepad2.dpad_up) {
                    gamepad2.reset();
                    if (arm_state == ArmState.ARM_COLLECT) {
                        arm_collect_mode_to_up_back();
                    } else if (arm_state == ArmState.ARM_UP_BACK) {
                        arm_front();
                        sleep(1000);
                    } else if (arm_state == ArmState.ARM_INIT) {
                        release_arm();
                    } else if (arm_state == ArmState.ARM_UP_FRONT) {
                        arm_down();
                    } else if (arm_state == ArmState.ARM_DOWN_FRONT) {
                        arm_up();
                    }
                }
                else if (gamepad2.dpad_left) {
                    gamepad2.reset();
                    if (arm_state == ArmState.ARM_UP_FRONT || arm_state == ArmState.ARM_DOWN_FRONT)
                        go_red_mid_zone();
                }
                else if (gamepad2.dpad_down) {
                    gamepad2.reset();
                    if (arm_state == ArmState.ARM_UP_BACK) {
                        arm_collection_mode();
                    } else if (arm_state == ArmState.ARM_UP_FRONT) {
                        arm_back();
                        sleep(1000);
                    } else if (arm_state==ArmState.ARM_SCORE_MID_RED) {
                        arm_back_from_goal();
                    }
                }
            } else { // Auto state, test the arm routines

            }
            show_telemetry();
            waitOneFullHardwareCycle();
        }
    }

    public void show_telemetry() {
        telemetry.addData("0. State: ", state.toString() + "(driver mode =" + motorFR.getMode().toString() + ")");
        telemetry.addData("1. shoulder:", "pos= " + String.format("%.4f, dir=%.2f)", shoulder_pos, shoulder_dir));
        telemetry.addData("2. elbow:", "pwr= " + String.format("%.2f, pos= %d, dir=%.2f", cur_arm_power, elbow_pos, elbow_dir));
        telemetry.addData("3. wrist/gate", "pos= " + String.format("%.2f / %.2f", wrist_pos, gate_pos));
        telemetry.addData("4. arm_slider", "pos (dir): " + String.format("%.2f (%.2f)", slider_pos, slider_dir));
        telemetry.addData("5. tape_rotator", "pos= " + String.format("%2d", tape_rotator_pos));
        telemetry.addData("6. drive power: L=", String.format("%.2f", leftPower) + "/R=" + String.format("%.2f", rightPower));
        //telemetry.addData("7. left  cur/tg enc:", motorBL.getCurrentPosition() + "/" + motorBL.getTargetPosition());
        //telemetry.addData("8. right cur/tg enc:", motorFR.getCurrentPosition() + "/" + motorFR.getTargetPosition());
        telemetry.addData("7. left  cur/tg enc:", motorBL.getCurrentPosition() + "/" + leftCnt);
        telemetry.addData("8. right cur/tg enc:", motorFR.getCurrentPosition() + "/" + rightCnt);
        telemetry.addData("9. ods:", String.format("%.2f", opSensor.getLightDetected()));
    }


    public void set_elbow_pos(int pos, double power) throws InterruptedException {
        double init_time = getRuntime();
        if (power < 0)
            power = -power; // power always use positive, and re-adgust based on current position
        if (power > 1) power = 1;
        int cur_pos = elbow.getCurrentPosition();
        elbow_pos = pos;
        if (cur_pos < pos) { // elbow up
            elbow.setPower(power);
            while (elbow.getCurrentPosition() < elbow_pos && (getRuntime()-init_time)<5) { // time out 5 sec
                elbow.setPower(power);
                waitForNextHardwareCycle();
            }
        } else { // elbow should go down
            elbow.setPower(-power);
            while (elbow.getCurrentPosition() > elbow_pos && (getRuntime()-init_time)<5) {
                elbow.setPower(-power);
                waitForNextHardwareCycle();
            }
        }
        elbow.setPower(0);
    }

    public void arm_slider_in_for_n_sec(double n) throws InterruptedException {
        arm_slider.setPosition(SLIDER_SHORTEN);
        sleep((long) (n * 1000.0));
        arm_slider.setPosition(SLIDER_STOP);
    }

    public void arm_slider_out_for_n_sec(double n) throws InterruptedException {
        arm_slider.setPosition(SLIDER_LENGHTEN);
        sleep((long) (n * 1000.0));
        arm_slider.setPosition(SLIDER_STOP);
    }

    void arm_down() throws InterruptedException {
        set_elbow_pos(1370, 0.25);
        arm_slider_in_for_n_sec(0.5);
        set_wrist_pos(WRIST_COLLECT);
        set_elbow_pos(650, 0.25);
        set_wrist_pos(WRIST_MID);
        elbow.setPower(0);
        if (arm_state == ArmState.ARM_UP_FRONT)
            arm_state = ArmState.ARM_DOWN_FRONT;
        else
            arm_state = ArmState.ARM_DOWN_BACK;
    }

    void set_shoulder_pos(double pos) {
        shoulder_pos = pos;
        shoulder.setPosition(shoulder_pos);
    }

    void set_wrist_pos(double pos) {
        wrist_pos = pos;
        wrist.setPosition(wrist_pos);
    }

    void arm_front() throws InterruptedException {
        set_shoulder_pos(SHOULDER_SCORE);
        set_light_sensor(LIGHT_SENSOR_UP); // also make sure light sensor up in case need to go climbing mt
        wait_arm_pos(7);
        arm_state = ArmState.ARM_UP_FRONT;
    }

    void arm_back() throws InterruptedException {
        set_shoulder_pos(SHOULDER_START);
        wait_arm_pos(7);
        arm_state = ArmState.ARM_UP_BACK;
    }

    void wait_arm_pos(double max_sec) throws InterruptedException {
        double init_time = getRuntime();
        // the following loop will timeout in 10 sec
        while (Math.abs(shoulder.getPosition() - shoulder_pos) > 0.01 && ((getRuntime() - init_time) < max_sec)) {
            waitForNextHardwareCycle();
        }
    }

    void climber_mission() throws InterruptedException {
        release_arm();
        arm_front();
        wrist.setPosition(WRIST_CLIMBER);
        sleep(3000);
        open_gate();
        sleep(2000);
        close_gate();
        arm_back();
    }

    void go_red_mid_zone() throws InterruptedException {
        set_elbow_pos(685, 0.3);
        set_shoulder_pos(SHOULDER_RED_MID_SCORE);
        wrist.setPosition(WRIST_UP);
        arm_slider_out_for_n_sec(5.0);
        arm_state = ArmState.ARM_SCORE_MID_RED;
    }

    void arm_back_from_goal() throws InterruptedException {
        wrist.setPosition(WRIST_UP);
        arm_slider_in_for_n_sec(5.0);
        set_shoulder_pos(SHOULDER_SCORE);
        // set_elbow_pos(1300, 0.4);
        arm_state = ArmState.ARM_DOWN_FRONT;
        // arm_back();
        // arm_state = ArmState.ARM_UP_BACK;
    }

    void arm_up() throws InterruptedException {
        set_elbow_pos(1335, 0.25);
        if (arm_state==ArmState.ARM_DOWN_FRONT)
            arm_state = ArmState.ARM_UP_FRONT;
        else {
            arm_state = ArmState.ARM_UP_BACK;
        }
    }

    void arm_up_back() throws InterruptedException { // shared by init and collection modes
        wrist.setPosition(WRIST_UP);
        sleep(500);
        set_elbow_pos(400, 0.5);
        wrist.setPosition(WRIST_MID);
        set_elbow_pos(990, 0.5);
        wrist.setPosition(WRIST_COLLECT);
        arm_slider_out_for_n_sec(0.7);
        arm_up();
    }

    void release_arm() throws InterruptedException {
        set_elbow_pos(100, 0.5);
        arm_slider_in_for_n_sec(1.0);
        arm_up_back();
    }

    void arm_collection_mode() throws InterruptedException {
        if (arm_state == ArmState.ARM_UP_FRONT) {
            arm_back();
        }
        if (arm_state == ArmState.ARM_UP_BACK) {
            arm_down();
        }
        set_light_sensor(LIGHT_SENSOR_DOWN);
        set_elbow_pos(200, 0.25);
        arm_slider_out_for_n_sec(2);
        wrist.setPosition(WRIST_COLLECT);
        set_elbow_pos(90, 0.25);
        arm_slider_out_for_n_sec(2.5);
        set_elbow_pos(20, 0.25);
        arm_state = ArmState.ARM_COLLECT;
    }

    void arm_collect_mode_to_up_back() throws InterruptedException {
        arm_slider_in_for_n_sec(3);
        set_elbow_pos(200, 0.3);
        wrist.setPosition(WRIST_MID);
        arm_slider_in_for_n_sec(2.2);
        arm_up_back();
    }

    void open_gate() {
        gate_pos = GATE_OPEN;
        gate.setPosition(gate_pos);
    }

    void close_gate() {
        gate_pos = GATE_CLOSED;
        gate.setPosition(gate_pos);
    }

    public void StraightR(double power, double n_rotations) throws InterruptedException {
        reset_chassis();
        // set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
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

    public void driveTT(double lp, double rp) {
        motorFR.setPower(rp);
        motorFL.setPower(lp);
        motorBR.setPower(rp);
        motorBL.setPower(lp);
    }

    public void run_until_encoder(int leftCnt, double leftPower, int rightCnt, double rightPower) throws InterruptedException {
        //motorFR.setTargetPosition(rightCnt);
        //motorBL.setTargetPosition(leftCnt);
        //motorBL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //motorFR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //waitOneFullHardwareCycle();
        driveTT(leftPower, rightPower);
        waitOneFullHardwareCycle();
        //while (motorFR.isBusy() || motorBL.isBusy()) {
        while (!have_drive_encoders_reached(leftCnt, rightCnt)) {
            driveTT(leftPower, rightPower);
            show_telemetry();
            waitOneFullHardwareCycle();
        }
        stop_chassis();
        if (state == State.STATE_AUTO) {
            set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        } else {
            set_drive_modes(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }
        // waitOneFullHardwareCycle();
    }

    public void TurnLeftD(double power, int degree, boolean spotTurn) throws InterruptedException {
        initAutoOpTime = getRuntime();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
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
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
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
        while (motorFR.getCurrentPosition() != 0 && motorBL.getCurrentPosition() != 0) {
            // && motorBR.getCurrentPosition()!=0) && motorFL.getCurrentPosition()!=0) {
            waitOneFullHardwareCycle();
        }
        leftCnt = 0;
        rightCnt = 0;
        motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    void reset_motors() throws InterruptedException {
        reset_chassis();
        elbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        tape_rotator.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        while (elbow.getCurrentPosition() != 0 && tape_rotator.getCurrentPosition() != 0) {
            waitOneFullHardwareCycle();
        }
        elbow.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        tape_rotator.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    boolean has_left_drive_encoder_reached(double p_count) {
        if (leftPower < 0) {
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
    boolean has_right_drive_encoder_reached(double p_count) {
        if (rightPower < 0) {
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
    boolean have_drive_encoders_reached(double p_left_count, double p_right_count) {
        boolean l_return = false;
        if (has_left_drive_encoder_reached(p_left_count) ||
                has_right_drive_encoder_reached(p_right_count)) {
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

    void leveler_right() throws InterruptedException {
        leveler_pos = LEVELER_RIGHT;
        leveler.setPosition(leveler_pos);
        sleep(500);
    }

    void leveler_left() throws InterruptedException {
        leveler_pos = LEVELER_LEFT;
        leveler.setPosition(leveler_pos);
        sleep(500);
    }

    void leveler_down() throws InterruptedException {
        leveler_pos = LEVELER_DOWN;
        leveler.setPosition(leveler_pos);
        sleep(100);
    }
    void set_right_climber(double pos) throws InterruptedException {
        climberR_pos = pos;
        climberR.setPosition(climberR_pos);
        sleep(100);
    }

    void set_left_climber(double pos) throws InterruptedException {
        climberL_pos = pos;
        climberL.setPosition(climberL_pos);
        sleep(100);
    }

    void hit_right_button() throws InterruptedException {
        leveler_right();
        TurnLeftD(0.3, 15, true);
        sleep(300);
        TurnLeftD(-0.3, 15, true);
        sleep(300);
    }

    void hit_left_button() throws InterruptedException {
        leveler_left();
        TurnRightD(0.3, 15, true);
        sleep(300);
        TurnRightD(-0.3, 15, true);
        sleep(300);
    }

    public void followLineTillOp(double op_stop_val, boolean leftFirst) throws InterruptedException {
        double op_val = 0;
        while ((op_val = opSensor.getLightDetected()) < op_stop_val) {
            //follow the line , using getDirection and drive methods
            int direction2go;
            direction2go = nav.getFollowLineDirection(leftFirst);
            nav.drive(direction2go, 0.3); sleep(100);
            telemetry.addData("1. ods:", String.format("%.2f", op_val));
            telemetry.addData("2. ll/lr:", String.format("%.2f/%.2f", LL.getLightDetected(), LR.getLightDetected()));
        }
        nav.drive(nav.BRAKE, 0); // Make sure robot is stopped
    }

    public void set_light_sensor(double pos) {
        light_sensor_sv_pos = pos;
        light_sensor_sv.setPosition(light_sensor_sv_pos);
    }

    public void goUntilWhite(double power) throws InterruptedException {
        while (!detectWhite()) {
            driveTT(power, power);
            waitForNextHardwareCycle();
        }
        stop_chassis();
    }

    public boolean detectWhite() {
        if (LL.getLightDetected() <= WHITE_MIN || LL.getLightDetected() >= WHITE_MAX) {
            return false;
        }
        return true;
    }

    void m_warning_message(String p_exception_message) {
        if (v_warning_generated) {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    }

    private boolean v_warning_generated = false;
    private String v_warning_message;
}
