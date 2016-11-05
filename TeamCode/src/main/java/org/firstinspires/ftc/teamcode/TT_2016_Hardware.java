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
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * TobotHardware
 * <p/>
 * Define all hardware (e.g. motors, servos, sensors) used by Tobot
 */
public class TT_2016_Hardware extends LinearOpMode {

    // CONSTANT VALUES.
    final static double THRESHOLD = 0.1;
    final static double SERVO_SCALE = 0.001;
    final static double PUSHER_UP = 0.14;
    final static double PUSHER_DOWN = 0.52;
    final static double PUSHER_EXTRA = 0.99;
    final static double GATE_CLOSED = 0.55;
    final static double GATE_OPEN = 0.001;
    final static double LIGHT_SENSOR_UP = 0.03;
    final static double LIGHT_SENSOR_DOWN = 0.5;
    final static double LEFT_BEACON_PRESS = 0.6;
    final static double LEFT_BEACON_INIT = 0.05;
    final static double RIGHT_BEACON_PRESS = 0.4;
    final static double RIGHT_BEACON_INIT = 0.95;
    final static double WHITE_MAX = 0.79;
    final static double WHITE_MIN = 0.55;
    final static double WHITE_OP = 0.08; // optical distance sensor white color number
    final static int WHITE_ADA = 9000  ;
    final static double RANGE_WALL = 186.2;

    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 1;

    final static int ONE_ROTATION = 1120; // for AndyMark motor encoder one rotation
    // final static double RROBOT = 11;  // number of wheel turns to get chassis 360-degree
    final static double RROBOT = 6.63;  // number of wheel turns to get chassis 360-degree turn
    final static double INCHES_PER_ROTATION = 12.57; // inches per chassis motor rotation based on 16/24 gear ratio
    final static double GYRO_ROTATION_RATIO_L = 0.80; // 0.83; // Ratio of Gyro Sensor Left turn to prevent overshooting the turn.
    final static double GYRO_ROTATION_RATIO_R = 0.85; // 0.84; // Ratio of Gyro Sensor Right turn to prevent overshooting the turn.
    int numOpLoops = 1;

    //
    // following variables are used by Arm/slider
    //

    double armDelta = 0.1;
    int slider_counter = 0;

    // position of servos

    double light_sensor_sv_pos=0;
    double left_beacon_sv_pos=0;
    double right_beacon_sv_pos=0;
    double gate_sv_pos=0;
    double pusher_sv_pos=0;
    // amount to change the claw servo position by

    boolean blue_detected = false;
    boolean red_detected = false;
    int detectwhite = 0;

    // variables for sensors
      /* This is the port on the Core Device Interace Module */
  /* in which the navX-Micro is connected.  Modify this  */
  /* depending upon which I2C port you are using.        */
    private final int NAVX_DIM_I2C_PORT = 4;

    AHRS navx_device;
    double yaw;
    ColorSensor coSensor;
    ColorSensor coSensor2;
    ColorSensor coAda;
    DeviceInterfaceModule cdim;
    TouchSensor tSensor;
    ModernRoboticsI2cRangeSensor rangeSensor;
    UltrasonicSensor ultra;
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
    Boolean use_navx = false;
    Boolean use_gyro = false;
    Boolean use_encoder = true;
    Boolean use_ultra = false;
    Boolean use_range = true;
    Boolean use_adacolor = false;

    public enum State {
        STATE_TELEOP,    // state to test teleop
        STATE_AUTO,        // state to test auto routines
        STATE_TUNEUP    // state to manually tune up servo positions and arm positions
    }


    float speedScale = (float) 0.5; // controlling the speed of the chassis in teleOp state
    float leftPower = 0;
    float rightPower = 0;
    float SW_power = 0;
    float SH_power = 0;
    double initAutoOpTime = 0;
    float currRaw = 0;
    DcMotor motorR;
    DcMotor motorL;
    DcMotor sweeper;
    DcMotor shooter;
    Servo light_sensor_sv;
    Servo left_beacon_sv;
    Servo right_beacon_sv;
    Servo gate_sv;
    Servo pusher_sv;
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


        light_sensor_sv = init_servo("light_sensor_sv");
        left_beacon_sv = init_servo("left_beacon_sv");
        right_beacon_sv = init_servo("right_beacon_sv");
        gate_sv = init_servo("gate_sv");
        pusher_sv = init_servo("pusher_sv");
        //DbgLog.msg(String.format("TOBOT-INIT  light_sensor_sv -"));
        set_light_sensor(LIGHT_SENSOR_DOWN);
        set_left_beacon(LEFT_BEACON_INIT);
        set_right_beacon(RIGHT_BEACON_INIT);
        set_gate(GATE_CLOSED);
        set_pusher(PUSHER_UP);


        long systemTime = System.nanoTime();




        // initialize chassis variables
        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        shooter = hardwareMap.dcMotor.get("shooter");


        motorR.setDirection(DcMotor.Direction.REVERSE);
        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL.setPower(0);
        motorR.setPower(0);
        sweeper.setPower(0);
        shooter.setPower(0);
        sweeper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        state = st;

        if (state == State.STATE_TELEOP) {

        } else { // tune up or Auto

        }

        if (state == State.STATE_AUTO || state == State.STATE_TELEOP) {
            set_drive_modes(DcMotor.RunMode.RUN_USING_ENCODER);
        } else { // State.STATE_TUNE
            set_drive_modes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        // initialize sensores
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        coSensor = hardwareMap.colorSensor.get("co");
        coSensor.setI2cAddress(I2cAddr.create8bit(0x60));

        coSensor2 = hardwareMap.colorSensor.get("co2");
        coSensor2.setI2cAddress(I2cAddr.create8bit(0x3c));
        coSensor2.enableLed(true);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        // get a reference to our ColorSensor object.
        if(use_adacolor){
            coAda = hardwareMap.colorSensor.get("color");
        }

        // bEnabled represents the state of the LED.
        boolean bEnabled = true;

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        cdim.setDigitalChannelState(LED_CHANNEL, bEnabled);

        //tSensor = hardwareMap.touchSensor.get("to");
        //opSensor = hardwareMap.opticalDistanceSensor.get("op");
        if(use_ultra) {
            ultra = hardwareMap.ultrasonicSensor.get("ultra");
        }

        //LL = hardwareMap.lightSensor.get("ll");
        //LR = hardwareMap.lightSensor.get("lr");
        if (use_gyro) {
            gyro = hardwareMap.gyroSensor.get("gyro");
            // calibrate the gyro.
            gyro.calibrate();
        }
        //Instantiate ToborTech Nav object
        colorPicker = new TT_ColorPicker(coSensor2);
        if (true) {
            navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                    NAVX_DIM_I2C_PORT,
                    AHRS.DeviceDataType.kProcessedData);

            double init_time = getRuntime();
            boolean navx_ok = false;
            while (!navx_ok && (getRuntime() - init_time < 3)) { // wait for three sec to get connected
                navx_ok = navx_device.isConnected();
            }
            if (navx_ok) {
                boolean navx_cal = true;
                while (navx_cal && (getRuntime() - init_time < 5)) { // wait for 2 sec to get calibration
                    navx_cal = navx_device.isCalibrating();
                }
                if (navx_cal)
                    navx_ok = false;
            }
            if (!navx_ok) {
                DbgLog.msg(String.format("TOBOT-INIT: NaxX IMU is not connected!"));
            } else {
                navx_device.zeroYaw();
                use_navx = true;
            }
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
        double cur_heading = 0;
        if (use_navx){
            cur_heading = navx_device.getYaw();
        }
        else if (use_gyro){
            cur_heading = mapHeading(gyro.getHeading());
        }
        telemetry.addData("0. Program State: ", state.toString());
        telemetry.addData("1. use NavX/ use Gyro:", String.format("%s / %s ", use_navx.toString(), use_gyro.toString()));
        telemetry.addData("2. IMU_heading/Current heading:", String.format("%.2f/%.2f", imu_heading, cur_heading));
        // telemetry.addData("3. sv ls/l_b/r_b  = ", String.format("%.2f / %.2f / %.2f", light_sensor_sv_pos, left_beacon_sv_pos, right_beacon_sv_pos));
        telemetry.addData("4. Color1 R/G/B  = ", String.format("%d / %d / %d", coSensor.red(), coSensor.green(), coSensor.blue()));
        telemetry.addData("5. Color2 R/G/B  = ", String.format("%d / %d / %d", coSensor2.red(), coSensor2.green(), coSensor2.blue()));
        telemetry.addData("6. White / range = ", String.format("%d / %.2f",detectwhite, rangeSensor.getDistance(DistanceUnit.CM)  ));
        if(use_adacolor){
            telemetry.addData("7. Ada C/B/R/G/Sum  = ", String.format("%d/%d/%d/%d/%d", coAda.alpha(), coAda.blue(), coAda.red(), coAda.green(),
                    (coAda.alpha() + coAda.blue() + coAda.red() + coAda.green())));
        }
        telemetry.addData("8. drive power: L=", String.format("%.2f", leftPower) + "/R=" + String.format("%.2f", rightPower));
        telemetry.addData("9. gate/ pusher  = ", String.format("%.2f / %.2f", gate_sv_pos, pusher_sv_pos));
        telemetry.addData("10. sv ls/l_b/r_b  = ", String.format("%.2f / %.2f / %.2f", light_sensor_sv_pos, left_beacon_sv_pos, right_beacon_sv_pos));


        //telemetry.addData("7. left  cur/tg enc:", motorBL.getCurrentPosition() + "/" + leftCnt);
        //telemetry.addData("8. right cur/tg enc:", motorBR.getCurrentPosition() + "/" + rightCnt);
        //show_heading();
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

    public void StraightR(double power, double n_rotations) throws InterruptedException {
        reset_chassis();
        // set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = motorL.getCurrentPosition();
        int rightEncode = motorR.getCurrentPosition();
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
            double cur_heading = navx_device.getYaw();
            if (cur_heading - imu_heading > 2.5) { // cook to right,  slow down left motor
                if (lp > 0) lp *= 0.9;
                else rp *= 0.9;
            } else if (cur_heading - imu_heading < -2.5) {
                if (lp > 0) rp *= 0.9;
                else lp *= 0.9;
            }
        }
        motorR.setPower(rp);
        motorL.setPower(lp);
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
        int leftEncode = motorL.getCurrentPosition();
        int rightEncode = motorR.getCurrentPosition();
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
            imu_heading = current_pos - adjust_degree ;
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
        else if (use_gyro) {
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
        driveTT(0, 0);

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
        int leftEncode = motorL.getCurrentPosition();
        int rightEncode = motorR.getCurrentPosition();
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
        driveTT(0, 0);
        // sleep(500);
    }

    void set_drive_modes(DcMotor.RunMode mode) {
        motorL.setMode(mode);
        motorR.setMode(mode);
    }

    void stop_chassis() {
        motorL.setPower(0);
        motorR.setPower(0);
    }

    void stop_tobot() {
        stop_chassis();
        sweeper.setPower(0);
        shooter.setPower(0);
    }

    void reset_chassis() throws InterruptedException {
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (motorR.getCurrentPosition() != 0 && motorL.getCurrentPosition() != 0) {
            // && motorBR.getCurrentPosition()!=0) && motorFL.getCurrentPosition()!=0) {
            waitOneFullHardwareCycle();
        }
        leftCnt = 0;
        rightCnt = 0;
        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void reset_motors() throws InterruptedException {
        reset_chassis();
    }

    boolean has_left_drive_encoder_reached(double p_count) {
        if (leftPower < 0) {
            //return (Math.abs(motorFL.getCurrentPosition()) < p_count);
            return (motorL.getCurrentPosition() <= p_count);
        } else {
            //return (Math.abs(motorFL.getCurrentPosition()) > p_count);
            return (motorL.getCurrentPosition() >= p_count);
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
            return (motorR.getCurrentPosition() <= p_count);
        } else {
            return (motorR.getCurrentPosition() >= p_count);
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
            double diff = Math.abs(p_right_count - motorR.getCurrentPosition())/2;
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
            double diff = Math.abs(p_left_count - motorL.getCurrentPosition())/2;
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

    void hit_right_button() throws InterruptedException {
        set_right_beacon(RIGHT_BEACON_PRESS);
        sleep(500);
        bump_beacon();
        set_right_beacon(RIGHT_BEACON_INIT);
    }


    void bump_beacon() throws InterruptedException {
        driveTT(0.2, 0.2);
        sleep(1000);
        driveTT(0, 0);
        StraightIn(-0.5, 3);
    }

    void hit_left_button() throws InterruptedException {
        set_left_beacon(LEFT_BEACON_PRESS);
        sleep(500);
        bump_beacon();
        set_left_beacon(LEFT_BEACON_INIT);
    }

    public void forwardTillUltra(double us_stop_val, double power, double max_sec) throws InterruptedException {
        double us_val = 0;
        if (use_range) {
            us_val = rangeSensor.getDistance(DistanceUnit.CM);
        } else if (use_ultra) {
            us_val = ultra.getUltrasonicLevel();
        }
        double init_time = getRuntime();
        while ((us_val > us_stop_val) && ((getRuntime() - init_time) < max_sec)) {
            driveTT(power, power);
            // us_val = ultra.getUltrasonicLevel();
            if (use_range) {
                us_val = rangeSensor.getDistance(DistanceUnit.CM);
            } else if (use_ultra) {
                us_val = ultra.getUltrasonicLevel();
            }
        }
        driveTT(0, 0); // Make sure robot is stopped
    }

    public void set_light_sensor(double pos) {
        light_sensor_sv_pos = pos;
        light_sensor_sv.setPosition(light_sensor_sv_pos);
    }

    public void set_gate(double pos) {
       gate_sv_pos = pos;
       gate_sv.setPosition(gate_sv_pos);
    }

    public void set_pusher(double pos) {
        pusher_sv_pos = pos;
        pusher_sv.setPosition(pusher_sv_pos);
    }

    public void set_left_beacon(double pos) {
        left_beacon_sv_pos = pos;
        left_beacon_sv.setPosition(left_beacon_sv_pos);
    }

    public void set_right_beacon(double pos) {
        right_beacon_sv_pos = pos;
        right_beacon_sv.setPosition(right_beacon_sv_pos);
    }



    public void goUntilWhite(double power) throws InterruptedException {
        initAutoOpTime = getRuntime();
        while ((!detectWhite() || !detectWall()) && (getRuntime() - initAutoOpTime < 0.5)) {
            driveTT(power, power);
        }
        while ((!detectWhite() || !detectWall())&& (getRuntime() - initAutoOpTime < 2)) {
            driveTT(power, power);
        }
        stop_chassis();
    }

    public void goUntilWall(double power) throws InterruptedException {
        initAutoOpTime = getRuntime();
        while (!detectWall() && (getRuntime() - initAutoOpTime < 2)) {
            driveTT(power, power);
        }
        stop_chassis();
    }


    public void auto_part1(boolean is_red, boolean is_in) throws InterruptedException {

        if (use_gyro) {
            DbgLog.msg(String.format("Gyro current heading = %d, power L/R = %.2f/%.2f",
                    gyro.getHeading(), leftPower, rightPower));
        }

        if (false) {  // change true to skip part1
            return;
        }

            StraightIn(0.6, 34);
            //sleep(300);

        if (use_gyro) {
            DbgLog.msg(String.format("Gyro current heading = %d, power L/R = %.2f/%.2f",
                    gyro.getHeading(), leftPower, rightPower));
        }

        if (is_in) {
            if (is_red){
                TurnRightD(0.6,35,true);
            }
            else {
                TurnLeftD(0.6,45,true);
            }
        }
        else {
            if (is_red){
                TurnLeftD(0.6,35,true);
                StraightIn(0.6,29);
                TurnRightD(0.6,85,true);
            }
            else {
                TurnRightD(0.6,40,true);
                StraightIn(0.6,26);
                TurnLeftD(0.6,90,true);
            }
        }

        StraightIn(0.6, 5);

        if (use_gyro) {
            DbgLog.msg(String.format("Gyro current heading = %d, power L/R = %.2f/%.2f",
                    gyro.getHeading(), leftPower, rightPower));
        }
        // driveTT(0.5,0.5); sleep(1);driveTT(0,0);
    }

    public void auto_part2 (boolean is_red) throws InterruptedException{
        if (is_red){
            goBeacon(true);
        }
        else{
            goBeacon(false);
        }
        // Additional Autonomous code (launch ball, second beacon, etc.) goes here
    }

    public void goBeacon (boolean is_red) throws InterruptedException {
        if (true) {
            //goUntilWhite(0.3);
            goUntilWall(0.3);
            // StraightIn(0.5, 0.5);
            if(is_red){
                TurnLeftD(0.5, 81, true);
            }
            else{
                TurnRightD(0.5, 81, true);
            }
        }
        sleep(500);
        //forwardTillUltra(10, 0.25, 3);
        blue_detected = false;
        red_detected = false;

        if (true) {
            //sleep(1000);
            // Follow line until optical distance sensor detect 0.2 value to the wall (about 6cm)
             forwardTillUltra(10, 0.25, 5);

            // StraightIn(0.3, 1.0);
            //hit_left_button();
            TT_ColorPicker.Color cur_co = TT_ColorPicker.Color.UNKNOWN;
            double initTime = getRuntime();
            // sense color up to 2 secs
            do {
                cur_co = colorPicker.getColor();
            } while (cur_co==TT_ColorPicker.Color.UNKNOWN && getRuntime()-initTime<2);
            // Detect Beacon color and hit the right side
            if (cur_co == TT_ColorPicker.Color.BLUE) {
                blue_detected = true;
                if (is_red) {
                    hit_right_button();
                } else {
                    hit_left_button();
                }
            } else if (cur_co == TT_ColorPicker.Color.RED) {
                red_detected = true;
                if (is_red) {
                    hit_left_button();
                } else {
                    hit_right_button();
                }
            } else { // unknown, better not do anything than giving the credit to the opponent
                // doing nothing. May print out the message for debugging
            }

        }
    }

    public boolean detectWhite() {
    int cur_sum_ada_colors = 0;
        if(use_adacolor) {
            cur_sum_ada_colors = coAda.alpha()+coAda.blue()+coAda.red()+coAda.green();
        }

        if (cur_sum_ada_colors < WHITE_ADA) {
            return false;
        }
    //    if (opSensor.getLightDetected() < WHITE_OP) { // to-do
    //        return false;
    //    }
        return true;
    }

    public boolean detectWall() {
        double range = rangeSensor.getDistance(DistanceUnit.CM);
        if (range  > RANGE_WALL) {
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
