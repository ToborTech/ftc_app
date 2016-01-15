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
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
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
    final static double THRESHOLD = 0.1;
    final static double SERVO_SCALE = 0.001;
    final static double GATE_CLOSED = 0.05;
    final static double GATE_OPEN = 0.91;
    final static double WRIST_UP = 0.54;
    final static double WRIST_MID = 0.4;
    final static double WRIST_CLIMBER = 0.15;
    final static double WRIST_COLLECT = 0.11;
    final static double SHOULDER_START = 0.5082 ;
    final static double SHOULDER_TAPE_OUT = 0.46; // position to let tape out
    final static double SHOULDER_SCORE = 0.806;     // position to outside score position
    final static double SHOULDER_RED_MID_SCORE = 0.86; //position for scoring mid red zone basket
    final static double SHOULDER_BLUE_MID_SCORE = 0.7240; //position for scoring mid blue zone basket
    final static double SHOULDER_RED_HIGH_SCORE = 0.86; //position for scoring high red zone basket
    final static int ELBOW_LOW_POINT = 327;
    final static int ELBOW_MID_POINT = 600;
    final static int ELBOW_UP_POINT = 1335;
    final static double SLIDER_LENGHTEN = 0.0;
    final static double SLIDER_SHORTEN = 1.0;
    final static double SLIDER_STOP = 0.5;
    final static double TAPE_ROTATE = 0.25;
    final static double TAPE_SLIDER = 0.75;
    final static double LIGHT_SENSOR_UP = 0.2;
    final static double LIGHT_SENSOR_DOWN = 0.8;
    final static double LEVELER_RIGHT = 0.92;
    final static double LEVELER_DOWN = 0.52;
    final static double LEVELER_LEFT = 0.08;
    final static double FRONT_SV_DOWN = 0.99;
    final static double FRONT_SV_UP = 0.43;
    final static double RIGHT_CLIMBER_UP = 0.75;
    final static double RIGHT_CLIMBER_MID = 0.5;
    final static double RIGHT_CLIMBER_LOW = 0.2;
    final static double LEFT_CLIMBER_UP = 0.2;
    final static double LEFT_CLIMBER_MID = 0.65;
    final static double LEFT_CLIMBER_LOW = 0.8;
    final static double WHITE_MIN = 0.45;
    final static double WHITE_MAX = 0.85;
    final static int ONE_ROTATION = 1120; // for AndyMark motor encoder one rotation
    // final static double RROBOT = 11;  // number of wheel turns to get chassis 360-degree
    final static double RROBOT = 15.5;  // number of wheel turns to get chassis 360-degree turn
    final static double INCHES_PER_ROTATION = 9.8; // inches per chassis motor rotation based on 16/24 gear ratio
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
    double front_sv_pos;
    double light_sensor_sv_pos;
    double climberL_pos;
    double climberR_pos;
    // amount to change the claw servo position by
    double cur_arm_power;
    double arm_power;
    int elbow_pos;
    int elbow_pos_offset; // this is the elbow position where the start position should be.
                          // When re-start the robot, it should be 0. But if the elbow gear skips,
                          // it may change the value. Important to caliber it at up-front or up-back states
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
    Servo front_sv;
    boolean blue_detected = false;
    boolean red_detected = false;

    // variables for sensors
    ColorSensor coSensor;
    DeviceInterfaceModule cdim;
    TouchSensor tSensor;
    UltrasonicSensor ultra;
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

    float speedScale = (float) 0.9; // controlling the speed of the chassis in teleOp state
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
        elbow_pos = elbow.getCurrentPosition();
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
            wrist = hardwareMap.servo.get("wrist");
        } catch (Exception p_exeception) {
            m_warning_message("wrist");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            wrist = null;
        }
        if (elbow_pos>800) { // assuming at up-front position
            shoulder_pos = SHOULDER_SCORE;
            set_wrist_pos(WRIST_COLLECT);
        } else { // assuming init position
            shoulder_pos = SHOULDER_START;
            set_wrist_pos(WRIST_UP);
        }
        set_shoulder_pos(shoulder_pos); // make sure shoulder does not move initially

        try {
            gate = hardwareMap.servo.get("gate");
        } catch (Exception p_exeception) {
            m_warning_message("gate");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            gate = null;
        }

        try {
            arm_slider = hardwareMap.servo.get("arm_slider");
        } catch (Exception p_exeception) {
            m_warning_message("arm_slider");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            arm_slider = null;
        }
        slider_pos = SLIDER_STOP;
        arm_slider.setPosition(slider_pos);

        try {
            leveler = hardwareMap.servo.get("leveler");
        } catch (Exception p_exeception) {
            m_warning_message("leveler");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            leveler = null;
        }

        try {
            front_sv = hardwareMap.servo.get("front_sv");
        } catch (Exception p_exeception) {
            m_warning_message("front_sv");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            front_sv = null;
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

        set_left_climber(LEFT_CLIMBER_UP);
        set_right_climber(RIGHT_CLIMBER_UP);
        front_sv_down();
        leveler_down();
        gate_pos = GATE_CLOSED;
        gate.setPosition(gate_pos);

        light_sensor_sv_pos = LIGHT_SENSOR_DOWN;
        light_sensor_sv.setPosition(light_sensor_sv_pos);

        arm_power = 0.2;
        cur_arm_power = 0;
        shoulder_dir = 0;

        elbow_pos_offset = 0;
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

        if (state == State.STATE_TELEOP) {
            arm_state = ArmState.ARM_INIT;
            if (shoulder_pos>SHOULDER_START+0.2) { // arm must in up-front position after auto
                arm_state = ArmState.ARM_UP_FRONT;
                set_wrist_pos(WRIST_COLLECT);
            } else if (elbow_pos>ELBOW_MID_POINT) {
                arm_state = ArmState.ARM_UP_BACK;
                set_wrist_pos(WRIST_COLLECT);
            } else if (elbow_pos>ELBOW_LOW_POINT-150){
                arm_state = ArmState.ARM_DOWN_BACK;
                set_wrist_pos(WRIST_MID);
            } else { // ARM_INIT
                set_wrist_pos(WRIST_UP);
            }
        } else { // tune up or Auto
            arm_state = ArmState.ARM_INIT;
            set_wrist_pos(WRIST_UP);
        }
        if(arm_state == ArmState.ARM_UP_FRONT) {
            set_shoulder_pos(SHOULDER_SCORE);
        } else {
            set_shoulder_pos(SHOULDER_START);
        }

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
        ultra = hardwareMap.ultrasonicSensor.get("ultra");

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
            show_telemetry();
            waitOneFullHardwareCycle();
        }
    }

    public void show_telemetry() {
        telemetry.addData("0. Program/Arm State: ", state.toString() + "/" + arm_state.toString());
        telemetry.addData("1. shoulder:", "pos= " + String.format("%.4f, dir=%.2f)", shoulder_pos, shoulder_dir));
        telemetry.addData("2. elbow:", "pwr= " + String.format("%.2f, pos= %d, offset=%d", cur_arm_power, elbow_pos, elbow_pos_offset));
        telemetry.addData("3. wrist/gate", "pos= " + String.format("%.2f / %.2f", wrist_pos, gate_pos));
        telemetry.addData("4. arm_slider", "pos (dir): " + String.format("%.2f (%.2f)", slider_pos, slider_dir));
        telemetry.addData("5. tape_rotator", "pos= " + String.format("%2d", tape_rotator_pos));
        telemetry.addData("6. drive power: L=", String.format("%.2f", leftPower) + "/R=" + String.format("%.2f", rightPower));
        telemetry.addData("7. left  cur/tg enc:", motorBL.getCurrentPosition() + "/" + leftCnt);
        telemetry.addData("8. right cur/tg enc:", motorFR.getCurrentPosition() + "/" + rightCnt);
        // telemetry.addData("9. ods/ultra:", String.format("%.4f/%.2f", opSensor.getLightDetected(),ultra.getUltrasonicLevel()));
    }


    public void calibre_elbow() {
        // elbow calibration can only be done when arm at following states:
        //     up_front, up_back, down_back and init
        int cur_pos = elbow.getCurrentPosition();
        if (arm_state==ArmState.ARM_INIT) {
            elbow_pos_offset = cur_pos;
        } else if (arm_state==ArmState.ARM_UP_FRONT || arm_state==ArmState.ARM_UP_BACK) {
            elbow_pos_offset = cur_pos - ELBOW_UP_POINT;
        } else if (arm_state==ArmState.ARM_DOWN_BACK) {
            elbow_pos_offset = cur_pos - ELBOW_LOW_POINT;
        }
    }

    public void set_elbow_pos(int pos, double power) throws InterruptedException {
        double init_time = getRuntime();
        if (power < 0)
            power = -power; // power always use positive, and re-adgust based on current position
        if (power > 1) power = 1;
        int cur_pos = elbow.getCurrentPosition();
        elbow_pos = pos+elbow_pos_offset;
        if (cur_pos < elbow_pos) { // elbow up
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
        if (arm_state == ArmState.ARM_UP_BACK) {
            set_elbow_pos(990, 0.25);
            arm_slider_in_for_n_sec(0.7);
        }
        set_wrist_pos(WRIST_COLLECT); sleep(500);
        if (arm_state == ArmState.ARM_UP_FRONT) {
            set_elbow_pos(ELBOW_LOW_POINT, 0.2);
            arm_state = ArmState.ARM_DOWN_FRONT;
        } else {
            set_elbow_pos(570, 0.2);
            arm_state = ArmState.ARM_DOWN_BACK;
        }
        set_wrist_pos(WRIST_MID);
        if (arm_state == ArmState.ARM_DOWN_BACK) {
            set_elbow_pos(355, 0.2);
        }

        elbow.setPower(0);
    }

    void set_shoulder_pos(double pos) {
        shoulder_pos = pos;
        shoulder.setPosition(shoulder_pos);
    }

    void set_wrist_pos(double pos) {
        wrist_pos = pos;
        wrist.setPosition(wrist_pos);
    }

    void arm_front(boolean guard_up) throws InterruptedException {
        set_shoulder_pos(SHOULDER_SCORE);
        set_light_sensor(LIGHT_SENSOR_UP); // also make sure light sensor up in case need to go climbing mt
        if(guard_up){
            front_sv_up();
        }
        wait_arm_pos(4);
        arm_state = ArmState.ARM_UP_FRONT;
    }

    void arm_back() throws InterruptedException {
        set_shoulder_pos(SHOULDER_START);
        wait_arm_pos(4);
        arm_state = ArmState.ARM_UP_BACK;
    }

    void wait_arm_pos(double max_sec) throws InterruptedException {
        double init_time = getRuntime();
        // the following loop will timeout in 10 sec
        while (Math.abs(shoulder.getPosition() - shoulder_pos) > 0.02 && ((getRuntime() - init_time) < max_sec)) {
            waitForNextHardwareCycle();
        }
    }

    void climber_mission(boolean should_dump) throws InterruptedException {
        release_arm();
        arm_front(false);
        wrist.setPosition(WRIST_CLIMBER);
        sleep(3000);
        if (should_dump) {
            open_gate();
            sleep(2000);
            close_gate();
            StraightIn(-0.5,6);
            arm_back(); sleep(5000);
            arm_down();
        }
    }

    void go_red_mid_zone() throws InterruptedException {
        if (arm_state==ArmState.ARM_UP_FRONT) {
            set_elbow_pos(580, 0.3);
        } else { // ARM_DOWN_FRONT
            set_elbow_pos(575, 0.5);
        }
        set_shoulder_pos(SHOULDER_RED_MID_SCORE);
        wrist.setPosition(WRIST_UP);
        arm_slider_out_for_n_sec(5.0);
        arm_state = ArmState.ARM_SCORE_MID_RED;
    }
    void go_blue_mid_zone() throws InterruptedException {
        if (arm_state==ArmState.ARM_UP_FRONT) {
            set_elbow_pos(560, 0.3);
        } else { // ARM_DOWN_FRONT
            set_elbow_pos(555, 0.5);
        }
        set_shoulder_pos(SHOULDER_BLUE_MID_SCORE);
        wrist.setPosition(WRIST_UP);
        arm_slider_out_for_n_sec(3.0);
        arm_state = ArmState.ARM_SCORE_MID_BLUE;
    }

    void arm_back_from_goal() throws InterruptedException {
        wrist.setPosition(WRIST_UP);
        // elbow up a little bit for easy traction
        elbow.setPower(0.2); sleep(100); elbow.setPower(0.0);
        if (arm_state==ArmState.ARM_SCORE_MID_BLUE) {
            arm_slider_in_for_n_sec(3.0);
        } else {
            arm_slider_in_for_n_sec(5.0);
        }
        set_shoulder_pos(SHOULDER_SCORE);
        // set_elbow_pos(1300, 0.4);
        arm_state = ArmState.ARM_DOWN_FRONT;
        // arm_back();
        // arm_state = ArmState.ARM_UP_BACK;
    }

    void arm_up() throws InterruptedException {
        if (arm_state==ArmState.ARM_DOWN_BACK) {
            set_elbow_pos(990, 0.4);
            wrist.setPosition(WRIST_COLLECT);
            arm_slider_out_for_n_sec(0.7);
        }
        set_elbow_pos(ELBOW_UP_POINT, 0.25);

        if (arm_state==ArmState.ARM_DOWN_FRONT)
            arm_state = ArmState.ARM_UP_FRONT;
        else { // either from ARM_DOWN_BACK or ARM_COLLECT
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
        } else if (arm_state == ArmState.ARM_UP_BACK) {
            arm_down();
        }
        set_light_sensor(LIGHT_SENSOR_DOWN);
        front_sv_down();

        if (arm_state == ArmState.ARM_INIT) {
            set_elbow_pos(100, 0.25);
            arm_slider_out_for_n_sec(1.5);
        } else {
            set_elbow_pos(180, 0.25);
            arm_slider_out_for_n_sec(2);
        }
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

        sleep(300);
    }

    public void StraightIn(double power, double in) throws InterruptedException {
        double numberR = in/INCHES_PER_ROTATION;
        StraightR(power, numberR);
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

        sleep(500);
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

        sleep(500);
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
        arm_slider.setPosition(SLIDER_STOP);
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
        if (has_left_drive_encoder_reached(p_left_count) &&
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
    void front_sv_up() throws InterruptedException {
        front_sv_pos = FRONT_SV_UP;
        front_sv.setPosition(front_sv_pos);
        sleep(500);
    }
    void front_sv_down() throws InterruptedException {
        front_sv_pos = FRONT_SV_DOWN;
        front_sv.setPosition(front_sv_pos);
        sleep(500);
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
        bump_beacon();
    }

    void bump_beacon() throws InterruptedException {
        driveTT(0.2, 0.2); sleep(1000); driveTT(0, 0);
        // StraightIn(-0.5, 6.0);
        if (false) {
            StraightIn(0.3, 1.0);
            sleep(200);
            StraightIn(-0.3, 1.0);
            sleep(200);
        }
    }

    void hit_left_button() throws InterruptedException {
        leveler_left();
        bump_beacon();
    }

    public void followLineTillOp(double op_stop_val, boolean leftFirst, double max_sec) throws InterruptedException {
        double op_val = 0;
        double init_time = getRuntime();
        while ((op_val = opSensor.getLightDetected()) < op_stop_val && ((getRuntime() - init_time) < max_sec)) {
            //follow the line , using getDirection and drive methods
            int direction2go;
            direction2go = nav.getFollowLineDirection(leftFirst);
            nav.drive(direction2go, 0.2);
            if (direction2go!=TT_Nav.FORWARD) {
                sleep(40);
            } else { // forward, make the move more
                sleep(100);
            }
            telemetry.addData("1. ods:", String.format("%.2f", op_val));
            telemetry.addData("2. ll/lr:", String.format("%.2f/%.2f", LL.getLightDetected(), LR.getLightDetected()));
        }
        nav.drive(nav.BRAKE, 0); // Make sure robot is stopped
    }
    public void forwardTillOp(double op_stop_val, double power, double max_sec) throws InterruptedException {
        double op_val = opSensor.getLightDetected();
        double init_time = getRuntime();
        while (op_val < op_stop_val && ((getRuntime() - init_time) < max_sec)) {
            nav.drive(TT_Nav.FORWARD, 0.2);
            op_val = opSensor.getLightDetected();
            waitForNextHardwareCycle();
        }
        nav.drive(nav.BRAKE, 0); // Make sure robot is stopped
    }

    public void forwardTillUltra(double us_stop_val, double power, double max_sec) throws InterruptedException {
        double us_val = ultra.getUltrasonicLevel();
        double init_time = getRuntime();
        while ((us_val<0.1 || us_val > us_stop_val) && ((getRuntime() - init_time) < max_sec)) {
            nav.drive(TT_Nav.FORWARD, 0.2);
            us_val = ultra.getUltrasonicLevel();
            waitForNextHardwareCycle();
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

    public void auto_part2(boolean is_red) throws InterruptedException {
        if (true) {
            goUntilWhite(-0.4);
            StraightIn(0.5, 1.5);
            sleep(500);
        }

        blue_detected = false;
        red_detected = false;
        if (true) {
            if (is_red) {
                TurnLeftD(0.5, 90, true);
            } else { // must be blue zone
                TurnRightD(0.5, 82, true);
            }
        }
        if (true) {
            // Follow line until optical distance sensor detect 0.2 value to the wall (about 6cm)
            // followLineTillOp(0.03, true, 5);
            // forwardTillOp(0.021, 0.35, 1.2);
            forwardTillUltra(15, 0.3, 5);
            // StraightIn(0.3, 1.0);
            //hit_left_button();
            TT_ColorPicker.Color cur_co = TT_ColorPicker.Color.UNKNOWN;
            double initTime = getRuntime();
            // sense color up to 2 secs
            do {
                cur_co = colorPicker.getColor();
                waitForNextHardwareCycle();
            } while (cur_co==TT_ColorPicker.Color.UNKNOWN && getRuntime()-initTime<2);
            // Detect Beacon color and hit the right side
            boolean should_dump = false;
            if (cur_co == TT_ColorPicker.Color.BLUE) {
                blue_detected = true;
                should_dump = true;
                if (is_red) {
                    hit_left_button();
                } else {
                    hit_right_button();
                }
            } else if (cur_co == TT_ColorPicker.Color.RED) {
                red_detected = true;
                should_dump = true;
                if (is_red) {
                    hit_right_button();
                } else {
                    hit_left_button();
                }
            } else { // unknown, better not do anything than giving the credit to the opponent
                // doing nothing. May print out the message for debugging
            }
            // dump two climbers
            climber_mission(should_dump);

            // climber_mission(true);
        }
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
