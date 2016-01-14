package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Tobortech on 7/29/2015.
 *
 * TeleOpOld Mode
 * <p>
 * Test of the first ToBot TeleOpOld via the new controller
 */
public class TeleOpOld extends ArmTest {

    /*
     * Note: the configuration of the servos is such that
     * as the arm servo approaches 0, the arm position moves up (away from the floor).
     * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
     */
    // TETRIX VALUES.
    final static int    ONE_ROTATION = 1120; // for AndyMark motor encoder
    final static double  RROBOT = 9;  // number of wheel turns to get chassis 360-degree turn
    int numOpLoops = 1;

    State state;
    public enum State {
        STATE_TELEOP,
        STATE_AUTO
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
    boolean satisfyBothEnc; // used by chassis motor encoders. True when both encoders must meet the target positions.
    // RotationSensor mRSensor; // rotation sensor object

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        chassis_init();
        arm_init();
    }

    public void chassis_init () {

        //mRSensor = new RotationSensor();

        // mRSensor.register();


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */
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
        state = State.STATE_TELEOP;
        satisfyBothEnc = true;
    }
    

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        chassis_loop();
        arm_loop();
    }
    public void chassis_loop() {

        switch (state) {

            case STATE_TELEOP:
		        /*
		         * Gamepad 1
		         *
		         * Gamepad 1 controls the motors via the left/right sticks, and it controls the
		         * speed via the a, y buttons
		         */
                motorBL.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorBR.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorFL.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorFR.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                float right = gamepad1.left_stick_y;
                float left = gamepad1.right_stick_y;

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
                    TurnLeftD(0.8, 90, true);
                }
                if (gamepad1.dpad_right) { //right spot turn 90 Degrees
                    TurnRightD(0.8, 90, true);
                }
                if (gamepad1.right_trigger>0.1) { // Sweeper Forward
                    SW_power = (float)-1.0;
                    //if (SW_power<-0.5) {
                    //    SW_power = 0;
                    //} else {
                    //    SW_power = (float)-0.7;
                    //}
                }
                if (gamepad1.right_bumper) { // Sweeper Backward
                    SW_power = (float)1.0;
                    //if (SW_power>0.5) {
                    //    SW_power = 0;
                    //} else {
                    //    SW_power = (float)0.7;
                    //}
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

                /*
		         * Gamepad 2
		         *
		         * Gamepad 2 controls the ARM via the left/right sticks, and it controls the
		         * speed via the a, y buttons
		         * motor name: "elbow" control the arm up-down
		         * servo name: "sholuder" control the arm lefe-right
		         * serve name: "wrist" control the tube up-down
		         * servo name: "gate" control the tube gate
		         */



                break;
            case STATE_AUTO:
                if (have_drive_encoders_reached(leftCnt, rightCnt)) {
                    motorFR.setPower(0);
                    motorFL.setPower(0);
                    motorBR.setPower(0);
                    motorBL.setPower(0);
                    leftPower = rightPower = 0;
                    state = State.STATE_TELEOP;
                    motorFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorFR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorBL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorBR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorFL.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    motorFR.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    motorBL.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    motorBR.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                } else {
                    motorFR.setPower(rightPower);
                    motorFL.setPower(leftPower);
                    motorBR.setPower(rightPower);
                    motorBL.setPower(leftPower);
                }
                // TODO: use the sensors (e.g. rotation sensor) to improve the precision here

                break;
        }
        numOpLoops++;
        motorRightCurrentEncoder = motorFR.getCurrentPosition();
        motorRightTargetEncoder = motorFR.getTargetPosition();
        motorLeftCurrentEncoder = motorFL.getCurrentPosition();
        motorLeftTargetEncoder = motorFL.getTargetPosition();
        // currRaw = mRSensor.getCurrRaw();
        // telemetry.addData("State/Raw: ", state.toString() + "("+mRSensor.getState().toString()+")"+"/" + String.format("%.4f",currRaw));
        telemetry.addData("State: ", state.toString());
        telemetry.addData("L/R  power: ", String.format("%.2f", leftPower) + "/" + String.format("%.2f", rightPower));
        telemetry.addData("L/R runmode: ", motorFR.getMode().toString());
        telemetry.addData("motorSW power: ", String.format("%.2f", SW_power));
        telemetry.addData("left  cur/tg enc:", motorFL.getCurrentPosition()+"/"+motorFL.getTargetPosition());
        telemetry.addData("right cur/tg enc:", motorFR.getCurrentPosition() + "/" + motorFR.getTargetPosition());
        telemetry.addData("L/R target cnt:", leftCnt + "/" + rightCnt);
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {
        // mRSensor.unregister();
    }

    // move straight for n_rotations with motor power > 0 being forward, power<0 being backward
    public void StraightR(double power, double n_rotations) {
        int leftEncode = motorFL.getCurrentPosition();
        int rightEncode = motorFR.getCurrentPosition();
        initAutoOpTime = this.time;
        motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftCnt = (int) (ONE_ROTATION * n_rotations);
        rightCnt = (int) (ONE_ROTATION * n_rotations);
        leftPower = rightPower = (float) power;
        if (power<0) { // move backward
            leftCnt = leftEncode - leftCnt;
            rightCnt = rightEncode - rightCnt;
        } else {
            leftCnt += leftEncode;
            rightCnt += rightEncode;
        }
        motorFR.setPower(rightPower);
        motorFL.setPower(leftPower);
        motorBR.setPower(rightPower);
        motorBL.setPower(leftPower);
        state = State.STATE_AUTO;
        satisfyBothEnc = false;
    }
    public void TurnLeftD(double power, int degree, boolean spotTurn) {
        initAutoOpTime = getRuntime();
        motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = motorFL.getCurrentPosition();
        int rightEncode = motorFR.getCurrentPosition();
        if (spotTurn) { // use both motors for spot turn
            leftCnt = (int) (-ONE_ROTATION * RROBOT * degree / 700.0);
            rightCnt = (int) (ONE_ROTATION * RROBOT * degree / 700.0);
            leftPower = (float) -power;
            satisfyBothEnc = false;
        } else { // swing turn. only use right motor
            leftCnt = 0;
            rightCnt = (int) (ONE_ROTATION * RROBOT * degree / 360.0);
            leftPower = (float) 0;
            satisfyBothEnc = true;
        }
        leftCnt += leftEncode;
        rightCnt += rightEncode;
        rightPower = (float) power;
        motorFR.setPower(rightPower);
        motorFL.setPower(leftPower);
        motorBR.setPower(rightPower);
        motorBL.setPower(leftPower);
        state = State.STATE_AUTO;
    }

    public void TurnRightD(double power, int degree, boolean spotTurn) {
        initAutoOpTime = getRuntime();
        motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = motorFL.getCurrentPosition();
        int rightEncode = motorFR.getCurrentPosition();
        if (spotTurn) { // use both motors for spot turn
            leftCnt = (int) (ONE_ROTATION * RROBOT * degree / 700.0);
            rightCnt = (int) (-ONE_ROTATION * RROBOT * degree / 700.0);
            rightPower = (float) -power;
            satisfyBothEnc = false;
        } else { // swing turn. only use right motor
            leftCnt = 0;
            rightCnt = (int) (ONE_ROTATION * RROBOT * degree / 360.0);
            rightPower = (float) 0;
            satisfyBothEnc = true;
        }
        leftCnt += leftEncode;
        rightCnt += rightEncode;
        leftPower = (float) power;
        motorFR.setPower(rightPower);
        motorFL.setPower(leftPower);
        motorBR.setPower(rightPower);
        motorBL.setPower(leftPower);
        state = State.STATE_AUTO;
    }

    //--------------------------------------------------------------------------
    //
    // has_left_drive_encoder_reached
    //
    /**
     * Indicate whether the left drive motor's encoder has reached a value.
     */
    boolean has_left_drive_encoder_reached (double p_count)
    {
        if (leftPower<0) {
            //return (Math.abs(motorFL.getCurrentPosition()) < p_count);
            return (motorFL.getCurrentPosition() < p_count);
        } else {
            //return (Math.abs(motorFL.getCurrentPosition()) > p_count);
            return (motorFL.getCurrentPosition() > p_count);
        }
    } // has_left_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // has_right_drive_encoder_reached
    //
    /**
     * Indicate whether the right drive motor's encoder has reached a value.
     */
    boolean has_right_drive_encoder_reached (double p_count)

    {
        if (rightPower<0) {
            //return (Math.abs(motorFR.getCurrentPosition()) < p_count);
            return (motorFR.getCurrentPosition() < p_count);
        } else {
            //return (Math.abs(motorFR.getCurrentPosition()) > p_count);
            return (motorFR.getCurrentPosition() > p_count);
        }

    } // has_right_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reached
    //
    /**
     * Indicate whether the drive motors' encoders have reached a value.
     */
    boolean have_drive_encoders_reached
    ( double p_left_count, double p_right_count) {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Have the encoders reached the specified values?
        //
        if (has_left_drive_encoder_reached (p_left_count) &&
                has_right_drive_encoder_reached (p_right_count))
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // have_encoders_reached

    public void reset_drive_encoders() {
        motorFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

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

    private double autoOpTime() {
        return this.time - initAutoOpTime;
    }

    private boolean bothEncodersZero() {
        return motorRightCurrentEncoder == 0 && motorLeftCurrentEncoder == 0;
    }

    private boolean reachPosition() {
        if (satisfyBothEnc) {
            return (withinMarginOfError(motorFR.getCurrentPosition(), motorFR.getTargetPosition()) &&
                    withinMarginOfError(motorFL.getCurrentPosition(), motorFL.getTargetPosition()));
        } else {
            return (withinMarginOfError(motorFR.getCurrentPosition(), motorFR.getTargetPosition()) ||
                    withinMarginOfError(motorFL.getCurrentPosition(), motorFL.getTargetPosition()));
        }
    }

    private boolean withinMarginOfError(int goal, int value) {
        int lowerMargin = goal - 4;
        int upperMargin = goal + 4;
        return (value >= lowerMargin && value <= upperMargin);
    }
}