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
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    final static double GATE_CLOSED = 0.99;
    final static double GATE_OPEN = 0.3;
    final static double GATE_DUMP = 0.3;
    final static double WRIST_UD_INIT = 0.20;
    final static double WRIST_UD_UP = 0.68;
    final static double WRIST_UD_RED_MID = 0.6;
    final static double WRIST_UD_RED_HIGH = 0.73;
    final static double WRIST_UD_BLUE_HIGH = 0.8;
    final static double WRIST_UD_DUMP = 0.53;
    final static double WRIST_LR_INIT = 0.72;
    final static double WRIST_LR_DOWN = 0.11;
    final static double WRIST_LR_BLUE_MID = 0.29;
    final static double WRIST_LR_BLUE_HIGH = 0.26;
    final static double WRIST_LR_RED_HIGH = 0.001;
    final static double WRIST_LR_DUMP = 0.2;

    final static double SHOULDER_START = 0.4885;
    final static double SHOULDER_SCORE = 0.806;     // position to outside score position
    final static double SHOULDER_RED_MID_SCORE = 0.52;  //position for scoring mid red zone basket
    final static double SHOULDER_BLUE_MID_SCORE = 0.42; //position for scoring mid blue zone basket
    final static double SHOULDER_RED_HIGH_SCORE = 0.52; //position for scoring high red zone basket
    final static double SHOULDER_BLUE_HIGH_SCORE = 0.4480; //position for scoring high red zone basket
    final static int ELBOW_LOW_POINT = 327;
    final static int ELBOW_MID_POINT = 600;
    final static int ELBOW_UP_POINT = 1260;
    final static double SLIDER_LENGHTEN = 0.0;
    final static double SLIDER_SHORTEN = 1.0;
    final static double SLIDER_STOP = 0.5;
    final static double TAPE_ROTATE_INIT = 0.5;
    final static double TAPE_SLIDER = 0.75;
    final static double LIGHT_SENSOR_UP = 0.03;
    final static double LIGHT_SENSOR_DOWN = 0.5;
    final static double LEVELER_RIGHT = 0.36;
    final static double LEVELER_INIT = 0.14;
    final static double LEVELER_LEFT = 0.62;
    final static double FRONT_SV_DOWN = 0.99;
    final static double FRONT_SV_UP = 0.43;
    final static double RIGHT_CLIMBER_UP = 0.71;
    final static double RIGHT_CLIMBER_MID = 0.5;
    final static double RIGHT_CLIMBER_LOW = 0.2;
    final static double LEFT_CLIMBER_UP = 0.25;
    final static double LEFT_CLIMBER_MID = 0.65;
    final static double LEFT_CLIMBER_LOW = 0.8;
    final static double WHITE_MAX = 0.79;
    final static double WHITE_MIN = 0.55;
    final static double WHITE_OP = 0.08; // optical distance sensor white color number
    final static int WHITE_ADA = 250;
    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 5;

    final static int ONE_ROTATION = 1120; // for AndyMark motor encoder one rotation
    // final static double RROBOT = 11;  // number of wheel turns to get chassis 360-degree
    final static double RROBOT = 25.63;  // number of wheel turns to get chassis 360-degree turn
    final static double INCHES_PER_ROTATION = 6.67; // inches per chassis motor rotation based on 16/24 gear ratio
    final static double GYRO_ROTATION_RATIO_L = 0.92; // 0.83; // Ratio of Gyro Sensor Left turn to prevent overshooting the turn.
    final static double GYRO_ROTATION_RATIO_R = 0.88; // 0.84; // Ratio of Gyro Sensor Right turn to prevent overshooting the turn.
    int numOpLoops = 1;

    //
    // following variables are used by Arm/slider
    //

    double armDelta = 0.1;
    int slider_counter = 0;

    // position of servos
    double shoulder_pos;
    double wristLR_pos;
    double wristUD_pos;
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
    int tape_slider_pos;
    int tape_count;
    double tape_rotator_pos = 0.5;
    double shoulder_dir;
    double slider_dir;
    double elbow_dir;
    double tape_slider_dir;
    double tape_rotator_dir;
    int slider_curr_count;
    //double tape_rotation_pos;
    //double tape_slider_pos;
    DcMotor tape_slider;
    DcMotor elbow;
    Servo tape_rotator;
    Servo shoulder;
    Servo wristLR;
    Servo wristUD;
    Servo gate;
    Servo arm_slider;
    Servo leveler;
    //Servo light_sensor_sv;
    // Servo climberL;
    // Servo climberR;
    Servo front_sv;
    boolean blue_detected = false;
    boolean red_detected = false;

    // variables for sensors
      /* This is the port on the Core Device Interace Module */
  /* in which the navX-Micro is connected.  Modify this  */
  /* depending upon which I2C port you are using.        */
    private final int NAVX_DIM_I2C_PORT = 4;

    AHRS navx_device;
    double yaw;
    //ColorSensor coSensor;
    //ColorSensor coSensor2;
    //ColorSensor coAda;
    DeviceInterfaceModule cdim;
    TouchSensor tSensor;
    //UltrasonicSensor ultra;
    OpticalDistanceSensor opSensor;
    GyroSensor gyro;
    int heading = 360;
    double imu_heading = 0;
    int touch = 0;
    // LightSensor LL, LR;

    // IBNO055IMU imu;
    TT_Nav_old nav;
    TT_ColorPicker colorPicker;

    // following variables are used by Chassis
    State state;
    ArmState arm_state;
    Boolean use_navx = true;
    Boolean use_gyro = false;
    Boolean use_encoder = true;

    public enum State {
        STATE_TELEOP,    // state to test teleop
        STATE_AUTO,        // state to test auto routines
        STATE_TUNEUP    // state to manually tune up servo positions and arm positions
    }

    public enum ArmState {
        ARM_INIT,       // arm at initial position
        ARM_UP_BACK,    // arm at up back position
        ARM_UP_FRONT,   // arm at up front position
        ARM_FRONT_DUMP, // arm at climber dump position
        ARM_DOWN_BACK,  // arm at back down position
        ARM_DOWN_FRONT, // arm at front down position
        ARM_SCORE_HIGH_RED,
        ARM_SCORE_MID_RED,
        ARM_SCORE_LOW_RED,
        ARM_SCORE_HIGH_BLUE,
        ARM_SCORE_MID_BLUE,
        ARM_SCORE_LOW_BLUE
    }

    float speedScale = (float) 0.8; // controlling the speed of the chassis in teleOp state
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

    public Servo init_servo(String name) {
        Servo new_sv;
        try {
            new_sv = hardwareMap.servo.get(name);
        } catch (Exception p_exeception) {
            m_warning_message(name);
            DbgLog.msg(p_exeception.getLocalizedMessage());
            new_sv = null;
        }
        DbgLog.msg(String.format("TOBOT init_servo() - %s", name));
        // commenting out following lines as the new release since March 2016,
        // Servo cannot get position right after initialization
        //double pos = new_sv.getPosition();
        //new_sv.setPosition(pos);
        return new_sv;
    }

    public void tobot_init(State st) throws InterruptedException {
        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

        DbgLog.msg(String.format("TOBOT-INIT Begin - 8-27"));
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

        gate = init_servo("gate");

        tape_rotator = init_servo("tape_rotator");
        set_tape_rotator(SLIDER_STOP);

        try {
            elbow = hardwareMap.dcMotor.get("elbow");
        } catch (Exception p_exeception) {
            m_warning_message("elbow");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            elbow = null;
        }

        elbow_pos = elbow.getCurrentPosition();
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setDirection(DcMotor.Direction.REVERSE);

        shoulder = init_servo("shoulder");
        shoulder_pos = SHOULDER_START;
        set_shoulder_pos(shoulder_pos); // make sure shoulder does not move initially

        wristLR = init_servo("wristLR");
        wristUD = init_servo("wristUD");


        arm_slider = init_servo("arm_slider");
        slider_pos = SLIDER_STOP;
        arm_slider.setPosition(slider_pos);

        leveler = init_servo("leveler");
        front_sv = init_servo("front_sv");
        // climberR = init_servo("climberR");
        // climberL = init_servo("climberL");
        //light_sensor_sv = init_servo("light_sensor_sv");

        //DbgLog.msg(String.format("TOBOT-INIT  light_sensor_sv -"));

        set_left_climber(LEFT_CLIMBER_UP);
        set_right_climber(RIGHT_CLIMBER_UP);
        front_sv_down();
        leveler_down();
        gate_pos = GATE_CLOSED;
        gate.setPosition(gate_pos);

        light_sensor_sv_pos = LIGHT_SENSOR_DOWN;
        //light_sensor_sv.setPosition(light_sensor_sv_pos);


        long systemTime = System.nanoTime();


        arm_power = 0.2;
        cur_arm_power = 0;
        shoulder_dir = 0;

        elbow_pos_offset = 0;
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
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSW.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);

        state = st;

        if (state == State.STATE_TELEOP) {
            arm_state = ArmState.ARM_INIT;
            if (elbow_pos > ELBOW_MID_POINT) { // robot must be at ARM_FRONT_DUMP state
                arm_state = ArmState.ARM_FRONT_DUMP;
                //set_wristLR_pos(WRIST_LR_DUMP);
                //set_wristUD_pos(WRIST_UD_DUMP);
            } else { // ARM_INIT
                set_wristUD_pos(WRIST_UD_INIT);
                set_wristLR_pos(WRIST_LR_INIT);
            }
        } else { // tune up or Auto
            arm_state = ArmState.ARM_INIT;
            set_wristUD_pos(WRIST_UD_INIT);
            set_wristLR_pos(WRIST_LR_INIT);
        }
        if (arm_state == ArmState.ARM_FRONT_DUMP) {
            set_shoulder_pos(SHOULDER_START);
        } else {
            set_shoulder_pos(SHOULDER_START);
        }

        if (state == State.STATE_AUTO || state == State.STATE_TELEOP) {
            set_drive_modes(DcMotor.RunMode.RUN_USING_ENCODER);
        } else { // State.STATE_TUNE
            set_drive_modes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        // initialize sensores
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        //coSensor = hardwareMap.colorSensor.get("co");
        //coSensor.setI2cAddress(I2cAddr.create8bit(0x60));

        //coSensor2 = hardwareMap.colorSensor.get("co2");
        //coSensor2.setI2cAddress(I2cAddr.create8bit(0x3c));
        //coSensor2.enableLed(true);

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        // get a reference to our ColorSensor object.
        //coAda = hardwareMap.colorSensor.get("color");

        // bEnabled represents the state of the LED.
        boolean bEnabled = true;

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        cdim.setDigitalChannelState(LED_CHANNEL, bEnabled);

        tSensor = hardwareMap.touchSensor.get("to");
        opSensor = hardwareMap.opticalDistanceSensor.get("op");
        //ultra = hardwareMap.ultrasonicSensor.get("ultra");

        //LL = hardwareMap.lightSensor.get("ll");
        //LR = hardwareMap.lightSensor.get("lr");

        gyro = hardwareMap.gyroSensor.get("gyro");
        // calibrate the gyro.
        gyro.calibrate();

        //Instantiate ToborTech Nav object
        nav = new TT_Nav_old(motorFR, motorBR, motorFL, motorBL, opSensor, false); // Not using Follow line
        // colorPicker = new TT_ColorPicker(coSensor2);
        if (state == State.STATE_TELEOP && arm_state == ArmState.ARM_FRONT_DUMP) {
            //sleep(500);
            set_wristLR_pos(WRIST_LR_DUMP);
            set_wristUD_pos(WRIST_UD_DUMP);
        }

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);

        double init_time = getRuntime();
        boolean navx_ok = false;
        while (!navx_ok && (getRuntime()-init_time<3)) { // wait for three sec to get connected
            navx_ok = navx_device.isConnected();
        }
        if (navx_ok) {
            boolean navx_cal = true;
            while (navx_cal && (getRuntime()-init_time<5)) { // wait for 2 sec to get calibration
                navx_cal = navx_device.isCalibrating();
            }
            if (navx_cal)
                navx_ok = false;
        }
        if (!navx_ok) {
            DbgLog.msg(String.format("TOBOT-INIT: NaxX IMU is not connected!"));
        } else {
            navx_device.zeroYaw();
        }

        hardwareMap.logDevices();
        DbgLog.msg(String.format("TOBOT-INIT  end() -"));
    } // end of tobot_init

    @Override
    public void runOpMode() throws InterruptedException {

        tobot_init(State.STATE_TELEOP);

        waitForStart();

        while (opModeIsActive()) {
            //show_telemetry();
            waitOneFullHardwareCycle();
        }
    }

    public void show_telemetry() {
        int cur_heading = mapHeading(gyro.getHeading());
        telemetry.addData("0. Program/Arm State: ", state.toString() + "/" + arm_state.toString());
        //telemetry.addData("1. shoulder:", "pos= " + String.format("%.4f, dir=%.2f)", shoulder_pos, shoulder_dir));
        //telemetry.addData("2. elbow:", "pwr= " + String.format("%.2f, pos= %d, offset=%d", cur_arm_power, elbow_pos, elbow_pos_offset));
        //telemetry.addData("3. wrist LR/UD", "pos= " + String.format("%.2f / %.2f", wristLR_pos, wristUD_pos));
        //telemetry.addData("4. arm_slider", "pos (dir): " + String.format("%.2f (%.2f)", slider_pos, slider_dir));
        //telemetry.addData("5. tape_rotator/gate", "pos= " + String.format("%.2f / %.2f", tape_rotator_pos, gate_pos));
        //telemetry.addData("4. Color1 R/G/B  = ", String.format("%d / %d / %d", coSensor.red(), coSensor.green(), coSensor.blue()));
        //telemetry.addData("5. Color2 R/G/B  = ", String.format("%d / %d / %d", coSensor2.red(), coSensor2.green(), coSensor2.blue()));
        telemetry.addData("6. drive power: L=", String.format("%.2f", leftPower) + "/R=" + String.format("%.2f", rightPower));
        //telemetry.addData("7. left  cur/tg enc:", motorBL.getCurrentPosition() + "/" + leftCnt);
        //telemetry.addData("8. right cur/tg enc:", motorBR.getCurrentPosition() + "/" + rightCnt);
        show_heading();
        telemetry.update();
        // Dbg.msg(String.format("Gyro heading tar/curr = %d/%d, power L/R = %.2f/%.2f",
	    //                   heading, cur_heading, leftPower, rightPower));
    }

    public void show_heading() {
        touch = (tSensor.isPressed()?1:0);
        telemetry.addData("9. head/gyro/ods/touch:", String.format("%d/%d/%.4f/%d",
                heading, gyro.getHeading(), opSensor.getLightDetected(),touch));
        //telemetry.addData("9. head/gyro/ods/ultra/touch:", String.format("%d/%d/%.4f/%.2f/%d",
        //        heading, gyro.getHeading(), opSensor.getLightDetected(), ultra.getUltrasonicLevel(),touch));
    }

    public void calibre_elbow() {
        // elbow calibration can only be done when arm at following states:
        //     up_front, up_back, down_back and init
        int cur_pos = elbow.getCurrentPosition();
        if (arm_state == ArmState.ARM_INIT) {
            elbow_pos_offset = cur_pos;
        } else if (arm_state == ArmState.ARM_UP_FRONT || arm_state == ArmState.ARM_UP_BACK) {
            elbow_pos_offset = cur_pos - ELBOW_UP_POINT;
        } else if (arm_state == ArmState.ARM_DOWN_BACK) {
            elbow_pos_offset = cur_pos - ELBOW_LOW_POINT;
        }
    }

    public void set_elbow_pos(int pos, double power) throws InterruptedException {
        double init_time = getRuntime();
        if (power < 0)
            power = -power; // power always use positive, and re-adgust based on current position
        if (power > 1) power = 1;
        int cur_pos = elbow.getCurrentPosition();
        elbow_pos = pos + elbow_pos_offset;
        if (cur_pos < elbow_pos) { // elbow up
            elbow.setPower(power);
            while (elbow.getCurrentPosition() < elbow_pos && (getRuntime() - init_time) < 2) { // time out 3 sec
                elbow.setPower(power);
                waitForNextHardwareCycle();
            }
        } else { // elbow should go down
            elbow.setPower(-power);
            while (elbow.getCurrentPosition() > elbow_pos && (getRuntime() - init_time) < 3) {
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

    public void arm_slider_in_till_touch(double nsec) throws InterruptedException {
        arm_slider.setPosition(SLIDER_SHORTEN);
        initAutoOpTime = getRuntime();
        while (!tSensor.isPressed() && (getRuntime()-initAutoOpTime<nsec)) {
            waitOneFullHardwareCycle();
        }
        arm_slider.setPosition(SLIDER_STOP);
    }

    public void arm_slider_out_for_n_sec(double n) throws InterruptedException {
        arm_slider.setPosition(SLIDER_LENGHTEN);
        sleep((long) (n * 1000.0));
        arm_slider.setPosition(SLIDER_STOP);
    }

    void arm_down() throws InterruptedException {
        set_wristLR_pos(WRIST_LR_INIT);
        sleep(100);
        set_wristUD_pos(WRIST_UD_INIT);
        sleep(50);
        arm_slider_out_for_n_sec(3);
        set_elbow_pos(ELBOW_LOW_POINT, 0.2);
        arm_state = ArmState.ARM_DOWN_FRONT;
        elbow.setPower(0);
    }

    void arm_front() throws InterruptedException {
        arm_slider.setPosition(SLIDER_LENGHTEN);
        set_wristLR_pos_slow(WRIST_LR_DOWN, 0.05);
        arm_slider_out_for_n_sec(2.2);
        set_elbow_pos(1800, 0.3);
        set_elbow_pos(2000, 0.1);
        set_elbow_pos(2100, 0.07); // slow down to avoid losing cubes
        arm_slider.setPosition(SLIDER_STOP);
        arm_state = ArmState.ARM_DOWN_FRONT;
        elbow.setPower(0);
        //light_sensor_sv.setPosition(LIGHT_SENSOR_UP);
    }

    void arm_back() throws InterruptedException {
        set_shoulder_pos(SHOULDER_START);
        set_wristLR_pos(WRIST_LR_INIT);
        if (arm_state != ArmState.ARM_UP_FRONT) {
            sleep(500);
        }
        set_wristUD_pos(WRIST_UD_INIT);
        if (!tSensor.isPressed()) {
            arm_slider_in_till_touch(4);
        }
        set_elbow_pos(20, 0.4);
        // set_elbow_pos(0, 0.1);
        arm_state = ArmState.ARM_DOWN_BACK;
    }

    void wait_arm_pos(double max_sec) throws InterruptedException {
        double init_time = getRuntime();
        // the following loop will timeout in 10 sec
        while (Math.abs(shoulder.getPosition() - shoulder_pos) > 0.02 && ((getRuntime() - init_time) < max_sec)) {
            waitForNextHardwareCycle();
        }
    }

    void wrist_shake() throws InterruptedException {
        set_elbow_pos(elbow_pos + 100, 0.3);
        set_elbow_pos(elbow_pos - 100, 0.3);
    }

    void climber_mission(boolean should_dump) throws InterruptedException {
        //arm_up();
        //arm_slider_out_for_n_sec(5);

        if (should_dump) {
            set_wristLR_pos(WRIST_LR_DUMP);
            set_wristUD_pos(WRIST_UD_DUMP);
            sleep(1000);
            //driveTT(-0.2, -0.2); sleep(500); driveTT(0, 0);
            dump_gate();
            elbow.setPower(0);
            sleep(1000);
            arm_state = ArmState.ARM_FRONT_DUMP;
            arm_back_from_goal();
            arm_back();
        } else {
            arm_state = ArmState.ARM_FRONT_DUMP;
        }
    }

    void go_red_high_zone() throws InterruptedException {
        if (arm_state == ArmState.ARM_UP_FRONT) {
            arm_front();
        }
        arm_slider_out_for_n_sec(1);
        set_shoulder_pos(SHOULDER_RED_MID_SCORE);
        arm_slider_out_for_n_sec(2);
        wristUD.setPosition(WRIST_UD_RED_HIGH);
        wristLR.setPosition(WRIST_LR_RED_HIGH);
        arm_state = ArmState.ARM_SCORE_HIGH_RED;
    }

    void go_red_mid_zone() throws InterruptedException {
        if (arm_state == ArmState.ARM_UP_FRONT) {
            arm_front();
        }
        arm_slider_out_for_n_sec(1);
        set_shoulder_pos(SHOULDER_RED_MID_SCORE);
        arm_slider_out_for_n_sec(1.5);
        wristUD.setPosition(WRIST_UD_RED_MID);
        arm_state = ArmState.ARM_SCORE_MID_RED;
    }

    void go_blue_high_zone() throws InterruptedException {
        if (arm_state == ArmState.ARM_UP_FRONT) {
            arm_front();
        }
        arm_slider_out_for_n_sec(1);
        set_shoulder_pos(SHOULDER_BLUE_HIGH_SCORE);
        arm_slider_out_for_n_sec(1.5);
        wristLR.setPosition(WRIST_LR_BLUE_HIGH);
        wristUD.setPosition(WRIST_UD_BLUE_HIGH);
        arm_state = ArmState.ARM_SCORE_HIGH_BLUE;
    }

    void go_blue_mid_zone() throws InterruptedException {
        if (arm_state == ArmState.ARM_UP_FRONT) {
            arm_front();
        }
        arm_slider_out_for_n_sec(0.5);
        set_shoulder_pos(SHOULDER_BLUE_MID_SCORE);
        //arm_slider_out_for_n_sec(1);
        wristLR.setPosition(WRIST_LR_BLUE_MID);
        arm_state = ArmState.ARM_SCORE_MID_BLUE;
    }

    void arm_back_from_goal() throws InterruptedException {
        close_gate();
        set_shoulder_pos(SHOULDER_START);
        if (arm_state == ArmState.ARM_SCORE_MID_BLUE) {
            arm_slider_in_for_n_sec(0.5);
        } else if (arm_state == ArmState.ARM_FRONT_DUMP) {
            arm_up();
            arm_slider_in_till_touch(5);
        } else { // high goals +  mid red
            arm_slider_in_for_n_sec(2.5);
        }
        if (arm_state != ArmState.ARM_FRONT_DUMP) {
            arm_state = ArmState.ARM_DOWN_FRONT;
            arm_up();
        }
        arm_state = ArmState.ARM_UP_FRONT;
    }

    void arm_front_from_goal() throws InterruptedException {
        set_shoulder_pos(SHOULDER_START);
        if (arm_state == ArmState.ARM_SCORE_MID_BLUE) {
            arm_slider_in_for_n_sec(0.5);
        } else if (arm_state == ArmState.ARM_SCORE_MID_RED) {
            arm_slider_in_for_n_sec(2.4);
        } else  if (arm_state == ArmState.ARM_SCORE_HIGH_BLUE) {
            arm_slider_in_for_n_sec(2.4);
        }  if (arm_state == ArmState.ARM_SCORE_HIGH_RED) {
            arm_slider_in_for_n_sec(2.9);
        }
        set_shoulder_pos(SHOULDER_START);
        arm_state = ArmState.ARM_DOWN_FRONT;
    }

    void arm_up() throws InterruptedException {
        close_gate();
        if (arm_state == ArmState.ARM_DOWN_FRONT) {
            set_elbow_pos(ELBOW_UP_POINT, 0.25);
            inc_wristUD_pos(0.1);
            set_wristLR_pos(WRIST_LR_INIT);
            arm_slider_in_till_touch(4.5);
            inc_wristUD_pos(-0.1);
        } else if (arm_state == ArmState.ARM_UP_FRONT || arm_state == ArmState.ARM_FRONT_DUMP) {
            inc_wristLR_pos(0.1);
            set_wristUD_pos(WRIST_UD_UP);
            set_wristLR_pos(WRIST_LR_INIT);
            set_elbow_pos(ELBOW_UP_POINT, 0.25);
        } else { // from init or back state
            set_elbow_pos(ELBOW_UP_POINT / 3, 0.25);
            inc_wristLR_pos(0.1);
            set_wristUD_pos(WRIST_UD_UP);
            set_elbow_pos(ELBOW_UP_POINT, 0.25);
        }
        arm_state = ArmState.ARM_UP_FRONT;
    }

    void set_shoulder_pos(double pos) {
        shoulder_pos = pos;
        shoulder.setPosition(shoulder_pos);
    }

    void set_wristUD_pos(double pos) {
        wristUD_pos = pos;
        wristUD.setPosition(wristUD_pos);
    }

    void set_wristLR_pos(double pos) {
        wristLR_pos = pos;
        wristLR.setPosition(wristLR_pos);
    }

    void set_wristLR_pos_slow(double pos, double tick) throws InterruptedException {
        wristLR_pos = pos;
        double cur_pos = wristLR.getPosition();
        if (cur_pos < pos) {
            while (wristLR.getPosition() < pos) {
                cur_pos += tick;
                wristLR.setPosition(cur_pos);
                sleep(100);
            }
        } else {
            while (wristLR.getPosition() > pos) {
                cur_pos -= tick;
                wristLR.setPosition(cur_pos);
                sleep(100);
            }
        }
        wristLR.setPosition(wristLR_pos);
    }

    void inc_wristLR_pos(double pos) {
        wristLR_pos += pos;
        if (wristLR_pos > 0.99) {
            wristLR_pos = 0.99;
        } else if (wristLR_pos < 0.01) {
            wristLR_pos = 0.01;
        }
        wristLR.setPosition(wristLR_pos);
    }

    void inc_wristUD_pos(double pos) {
        wristUD_pos += pos;
        if (wristUD_pos > 0.99) {
            wristUD_pos = 0.99;
        } else if (wristUD_pos < 0.01) {
            wristUD_pos = 0.01;
        }
        wristUD.setPosition(wristUD_pos);
    }

    void open_gate() {
        gate_pos = GATE_OPEN;
        gate.setPosition(gate_pos);
    }

    void dump_gate() {
        gate_pos = GATE_DUMP;
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
        int rightEncode = motorBR.getCurrentPosition();
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

    public int mapHeading(int n) {
        if (n < 45) return (n + 360);
        return n;
    }

    public void StraightIn(double power, double in) throws InterruptedException {
        if (use_encoder) {
            double numberR = in / INCHES_PER_ROTATION;
            StraightR(power, numberR);
        } else { // using timer
            double in_per_ms = 0.014 * power / 0.8;
            if (in_per_ms < 0) in_per_ms *= -1.0;
            long msec = (long) (in / in_per_ms);
            if (msec < 100) msec = 100;
            if (msec > 6000) msec = 6000; // limit to 6 sec
            driveTT(power, power);
            sleep(msec);
            driveTT(0, 0);
        }
    }

    public void driveTT(double lp, double rp) {
        //if (use_gyro == true && lp == rp) {
        if (use_navx) {
            int cur_heading = mapHeading(gyro.getHeading());
            if (cur_heading > imu_heading) { // cook to right,  slow down left motor
                if (lp > 0) lp *= 0.9;
                else rp *= 0.9;
            } else if (cur_heading < imu_heading) {
                if (lp > 0) rp *= 0.9;
                else lp *= 0.9;
            }
        }
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
        initAutoOpTime = getRuntime();
        //while (motorFR.isBusy() || motorBL.isBusy()) {
        while (!have_drive_encoders_reached(leftCnt, rightCnt) && (getRuntime() - initAutoOpTime < 5)) {
            driveTT(leftPower, rightPower);
            show_telemetry();
            waitOneFullHardwareCycle();
        }
        stop_chassis();
        if (state == State.STATE_AUTO) {
            set_drive_modes(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            set_drive_modes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        // waitOneFullHardwareCycle();
    }

    public void TurnLeftD(double power, int degree, boolean spotTurn) throws InterruptedException {
        double adjust_degree = GYRO_ROTATION_RATIO_L * (double) degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        initAutoOpTime = getRuntime();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = motorBL.getCurrentPosition();
        int rightEncode = motorBR.getCurrentPosition();
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
        if (use_navx) {
            current_pos = navx_device.getYaw();
            imu_heading = current_pos + adjust_degree ;
            if (imu_heading <= -180) {
                imu_heading += 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos <= 0)) {
                current_pos += 360;
            }
            while ((current_pos >= imu_heading) && ((getRuntime() - initAutoOpTime) < 5.0)) {
                current_pos = navx_device.getYaw();
                if (heading_cross_zero && (current_pos <= 0)) {
                    current_pos += 360;
                }
                driveTT(leftPower, rightPower);
            }
        }
        if (use_gyro) {
        // if (false) {
            initAutoOpTime = getRuntime();
            int cur_heading = gyro.getHeading();
            heading = gyro.getHeading() - (int) adjust_degree;
            Boolean cross_zero = false;
            if (heading < 0) {
                heading += 360;
                cur_heading += 360;
                cross_zero = true;
            }
            DbgLog.msg(String.format("LOP: Left Turn %d degree: Gyro tar/curr heading = %d/%d",
                    degree, heading, gyro.getHeading()));
            int prev_heading = -1;

            while (cur_heading > heading && (getRuntime() - initAutoOpTime < 4)) {
                driveTT(leftPower, rightPower);
                if (prev_heading!=cur_heading) {
                    DbgLog.msg(String.format("LOP: Gyro heading tar/curr = %d/%d, power L/R = %.2f/%.2f",
                            heading, cur_heading, leftPower, rightPower));
                }
                prev_heading = cur_heading;
                cur_heading = gyro.getHeading();
                if (cross_zero == true) {
                    if (cur_heading < adjust_degree) {
                        cur_heading += 360;
                    } else {
                        cross_zero = false;
                    }
                }

                // show_heading();
                waitForNextHardwareCycle();
            }
            driveTT(0, 0);
        } else {
            run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
        }

        // sleep(500);
    }

    public void TurnRightD(double power, int degree, boolean spotTurn) throws InterruptedException {
        double adjust_degree = GYRO_ROTATION_RATIO_R * (double) degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        initAutoOpTime = getRuntime();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = motorBL.getCurrentPosition();
        int rightEncode = motorBR.getCurrentPosition();
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

        if (use_navx) {
            current_pos = navx_device.getYaw();
            imu_heading = current_pos + adjust_degree ;
            if (imu_heading >= 180) {
                imu_heading -= 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos >= 0)) {
                current_pos -= 360;
            }
            while ((current_pos <= imu_heading) && ((getRuntime() - initAutoOpTime) < 5.0)) {
                current_pos = navx_device.getYaw();
                if (heading_cross_zero && (current_pos >= 0)) {
                    current_pos -= 360;
                }
                driveTT(leftPower, rightPower);
            }

        } else if (use_gyro) {
        // if (false) {
            initAutoOpTime = getRuntime();
            int cur_heading = gyro.getHeading();
            heading = cur_heading + (int)adjust_degree;
            int prev_heading = -1;
            int init_heading = cur_heading;
            DbgLog.msg(String.format("LOP: Right Turn %d degree: Gyro tar/curr heading = %d/%d",
                    degree, heading, gyro.getHeading()));

            while (cur_heading < heading && (getRuntime() - initAutoOpTime < 4)) {
                driveTT(leftPower, rightPower);
                if (prev_heading!=cur_heading) {
                    DbgLog.msg(String.format("LOP: Gyro heading tar/curr = %d/%d, power L/R = %.2f/%.2f",
                            heading, cur_heading, leftPower, rightPower));
                }
                prev_heading = cur_heading;
                cur_heading = gyro.getHeading();
                if (heading > 360 && init_heading > cur_heading) {
                    cur_heading += 360;
                }
                // show_heading();
                waitForNextHardwareCycle();

            }
            driveTT(0, 0);
        } else {
            if (use_encoder) {
                run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
            }
            else {
                long degree_in_ms = 33 * (long) degree;
                driveTT(leftPower, rightPower);
                sleep(degree_in_ms);
                driveTT(0, 0);
            }

        }
        // sleep(500);
    }

    void set_drive_modes(DcMotor.RunMode mode) {
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
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (motorBR.getCurrentPosition() != 0 && motorBL.getCurrentPosition() != 0) {
            // && motorBR.getCurrentPosition()!=0) && motorFL.getCurrentPosition()!=0) {
            waitOneFullHardwareCycle();
        }
        leftCnt = 0;
        rightCnt = 0;
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void reset_motors() throws InterruptedException {
        reset_chassis();
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (elbow.getCurrentPosition() != 0) {
            waitOneFullHardwareCycle();
        }
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            return (motorBR.getCurrentPosition() <= p_count);
        } else {
            return (motorBR.getCurrentPosition() >= p_count);
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
        if (has_left_drive_encoder_reached(p_left_count) && has_right_drive_encoder_reached(p_right_count)) {
            l_return = true;
        } else if (has_left_drive_encoder_reached(p_left_count)) { // shift target encoder value from right to left
            double diff = Math.abs(p_right_count - motorBR.getCurrentPosition())/2;
            if (leftPower<0) {
                leftCnt -= diff;
            } else {
                leftCnt += diff;
            }
            if (rightPower<0) {
                rightCnt += diff;
            } else {
                rightCnt -= diff;
            }
        } else if (has_right_drive_encoder_reached(p_right_count)) { // shift target encoder value from left to right
            double diff = Math.abs(p_left_count - motorBL.getCurrentPosition())/2;
            if (rightPower<0) {
                rightCnt -= diff;
            } else {
                rightCnt += diff;
            }
            if (leftPower<0) {
                leftCnt += diff;
            } else {
                leftCnt -= diff;
            }
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

    void set_tape_rotator(double pos) {
        tape_rotator_pos = pos;
        tape_rotator.setPosition(tape_rotator_pos);
    }

    void inc_tape_rotator(double inc) throws InterruptedException {
        if (inc < 0) {
            tape_rotator.setPosition(SLIDER_LENGHTEN);
        } else {
            tape_rotator.setPosition(SLIDER_SHORTEN);
        }
        sleep(30);
        tape_rotator.setPosition(SLIDER_STOP);
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
        leveler_pos = LEVELER_INIT;
        leveler.setPosition(leveler_pos);
        sleep(100);
    }

    void set_right_climber(double pos) throws InterruptedException {
        climberR_pos = pos;
        // climberR.setPosition(climberR_pos);
        sleep(100);
    }

    void set_left_climber(double pos) throws InterruptedException {
        climberL_pos = pos;
        // climberL.setPosition(climberL_pos);
        sleep(100);
    }

    void hit_right_button() throws InterruptedException {
        leveler_right();
        bump_beacon();
    }

    void bump_beacon() throws InterruptedException {
        driveTT(0.2, 0.2);
        sleep(1000);
        driveTT(0, 0);
        StraightIn(-0.5, 3);
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
            if (direction2go != TT_Nav_old.FORWARD) {
                sleep(40);
            } else { // forward, make the move more
                sleep(100);
            }
            telemetry.addData("1. ods:", String.format("%.2f", op_val));
            // telemetry.addData("2. ll/lr:", String.format("%.2f/%.2f", LL.getLightDetected(), LR.getLightDetected()));
        }
        nav.drive(nav.BRAKE, 0); // Make sure robot is stopped
    }

    public void forwardTillOp(double op_stop_val, double power, double max_sec) throws InterruptedException {
        double op_val = opSensor.getLightDetected();
        double init_time = getRuntime();
        while (op_val < op_stop_val && ((getRuntime() - init_time) < max_sec)) {
            nav.drive(TT_Nav_old.FORWARD, 0.2);
            op_val = opSensor.getLightDetected();
            waitForNextHardwareCycle();
        }
        nav.drive(nav.BRAKE, 0); // Make sure robot is stopped
    }

    public void forwardTillUltra(double us_stop_val, double power, double max_sec) throws InterruptedException {
        double us_val = 0; // ultra.getUltrasonicLevel();
        double init_time = getRuntime();
        while ((us_val < 0.1 || us_val > us_stop_val) && ((getRuntime() - init_time) < max_sec)) {
            nav.drive(TT_Nav_old.FORWARD, 0.2);
            // us_val = ultra.getUltrasonicLevel();
            waitForNextHardwareCycle();
        }
        nav.drive(nav.BRAKE, 0); // Make sure robot is stopped
    }

    public void set_light_sensor(double pos) {
        light_sensor_sv_pos = pos;
        //light_sensor_sv.setPosition(light_sensor_sv_pos);
    }

    public void goUntilWhite(double power) throws InterruptedException {
        initAutoOpTime = getRuntime();
        while (!detectWhite() && (getRuntime() - initAutoOpTime < 0.5)) {
            driveTT(power, power);
            waitForNextHardwareCycle();
        }
        while (!detectWhite() && (getRuntime() - initAutoOpTime < 2)) {
            driveTT(power, power);
            waitForNextHardwareCycle();
        }
        stop_chassis();
    }

    public void auto_part1(boolean is_red, boolean is_in) throws InterruptedException {

        DbgLog.msg(String.format("Gyro current heading = %d, power L/R = %.2f/%.2f",
                gyro.getHeading(), leftPower, rightPower));

        if (false) {  // change true to skip part1
            return;
        }
        if (false) {
            arm_up();
            arm_slider.setPosition(SLIDER_LENGHTEN);
            StraightIn(1, 51);
            arm_slider.setPosition(SLIDER_STOP);
        }
         else {
            StraightIn(1, 51);
            //sleep(300);
        }

        DbgLog.msg(String.format("Gyro current heading = %d, power L/R = %.2f/%.2f",
                gyro.getHeading(), leftPower, rightPower));

        if (is_red){
            // StraightIn(1, 10);
            driveTT(1,1); sleep(1500);driveTT(0,0);
        }
        else{
            //StraightIn(1, 14);
            driveTT(1, 1); sleep(1500);driveTT(0,0);
        }

        if (!is_in) { // move more
            StraightIn(1, 38);
        }
        DbgLog.msg(String.format("Gyro current heading = %d, power L/R = %.2f/%.2f",
                gyro.getHeading(), leftPower, rightPower));
        sleep(400);

        if (is_red) {
            TurnRightD(1, 45, true);
            sleep(400);
            if (is_in) {
                StraightIn(1, 20);
            }
        } else {
            TurnLeftD(1, 42, true);
            sleep(400);
            if (is_in) {
                StraightIn(1, 18);
            }
        }
        DbgLog.msg(String.format("Gyro current heading = %d, power L/R = %.2f/%.2f",
                gyro.getHeading(), leftPower, rightPower));
        // driveTT(0.5,0.5); sleep(1);driveTT(0,0);
    }

    public void auto_part2(boolean is_red) throws InterruptedException {
        if (true) {
            sleep(400);
            goUntilWhite(-0.3);
            // StraightIn(0.5, 0.5);
            driveTT(0.75, 0.75);
            if(is_red){
                sleep(500);
            }
            else{
                sleep(500);
            }
            driveTT(0, 0);
            sleep(400);
        }

        blue_detected = false;
        red_detected = false;
        if (true) {
            if (is_red) {
                heading = 315;
                TurnLeftD(1, 88, true);
            } else { // must be blue zone
                heading = 50;
                TurnRightD(1, 90, true);
                driveTT(0.75, 0.75);sleep(200);driveTT(0, 0);
            }
        }
        if (use_gyro) {
            sleep(400);
            int cur_heading = gyro.getHeading();
            int degree = 0;
            int left = 0;
            if (!is_red) {
                use_encoder = false; // for bl zone, use time correction for now
            }
            if (true) {
                if (cur_heading > heading) {
                    degree = (int) (cur_heading - heading);
                    TurnLeftD(1, degree, true);
                } else if (cur_heading < heading) {
                    degree = (int) (heading - cur_heading);
                    TurnRightD(1, degree, true);
                }
            } else {
                if (cur_heading > heading) {
                    degree = (int) (cur_heading - heading);
                    left = 1;
                } else if (cur_heading < heading) {
                    degree = (int) (heading - cur_heading);
                }
                telemetry.addData("8. Heading go / cur / degree (left)", String.format("%d / %d / %d (%d)", heading, cur_heading, degree, left));
                sleep(5000);
            }
            if (!is_red) {
                use_encoder = true;
            }
        }
        if (true) {
            // Follow line until optical distance sensor detect 0.2 value to the wall (about 6cm)
             forwardTillUltra(12, 0.5, 5);

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
                    hit_right_button();
                } else {
                    hit_left_button();
                }
            } else if (cur_co == TT_ColorPicker.Color.RED) {
                red_detected = true;
                should_dump = true;
                if (is_red) {
                    hit_left_button();
                } else {
                    hit_right_button();
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
    int cur_sum_ada_colors = 0; // coAda.alpha()+coAda.blue()+coAda.red()+coAda.green();
        if (cur_sum_ada_colors < WHITE_ADA) {
            return false;
        }
    //    if (opSensor.getLightDetected() < WHITE_OP) { // to-do
    //        return false;
    //    }
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
