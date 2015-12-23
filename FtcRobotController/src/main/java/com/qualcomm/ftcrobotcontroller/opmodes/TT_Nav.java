package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

// import org.swerverobotics.library.interfaces.IBNO055IMU;

public class TT_Nav {

    // Defining constants for drive direction
    final static int FORWARD    = 0 ;
    final static int BACKWARD   = 1 ;
    final static int LEFT       = 2 ;
    final static int RIGHT      = 3 ;
    final static int BRAKE      = 4 ;

    //final static double MOTOR_POWER = 0.2;// Higher values will cause the robot to move faster
    final static double MIN_PWR = 0.3 ;
    final static double MAX_PWR = 0.99 ;
    final static double ERR_MRGN = 0.05 ;

    final static double P = 0.0045;
    double cntAcc;

    private DcMotor _motorLeft1 ;
    private DcMotor _motorLeft2 ;
    private DcMotor _motorRight1;
    private DcMotor _motorRight2 ;
    private OpticalDistanceSensor _op;
    // private IBNO055IMU _imu ;
    private LightSensor _reflectedLightLeft, _reflectedLightRight ;

    final static double LIGHT_THRESHOLD = 0.4;


    TT_Nav(DcMotor motorR1, DcMotor motorR2, DcMotor motorL1, DcMotor motorL2,
           OpticalDistanceSensor op, boolean enableFollowLine,
           LightSensor reflectedLightLeft,LightSensor reflectedLightRight){
        _motorLeft1  = motorL1 ;
        _motorLeft2  = motorL2 ;
        _motorRight1 = motorR1 ;
        _motorRight2 = motorR2 ;
        _op = op;
        // _imu = imu ;
        if (enableFollowLine) {
            _reflectedLightLeft = reflectedLightLeft;
            _reflectedLightLeft.enableLed(true);  // turn on LED of light sensor.
            _reflectedLightRight = reflectedLightRight;
            _reflectedLightRight.enableLed(true);  // turn on LED of light sensor.
        }
    }

    // public void set_imu (IBNO055IMU imu ){
    //    _imu = imu ;
    //}

    public void TT_drive(double lp, double rp) {
        _motorLeft1.setPower(lp);
        _motorLeft2.setPower(lp);
        _motorRight1.setPower(rp);
        _motorRight2.setPower(rp);
    }

    public void drive (int direction , double power ){

        switch (direction) {
            case FORWARD :
                TT_drive(power, power);
                break;
            case BACKWARD:
                TT_drive(-power, -power);
                break;
            case LEFT:
                TT_drive(-power, power);
                break;
            case RIGHT:
                TT_drive(power, -power);
                break;
            case BRAKE:
                TT_drive(0, 0);
                break;
        }
    }


    private void adjustAngle(double offsetFromTarget){

        double rightPower, leftPower ;

        rightPower =  offsetFromTarget * P;
        // set variables for max and min power
        if (Math.abs(rightPower)< MIN_PWR){
            if ( rightPower < 0 ) {
                rightPower = -MIN_PWR;
            }
            else{
                rightPower = MIN_PWR ;
            }
        }
        if (Math.abs(rightPower)> MAX_PWR){
            if ( rightPower < 0 ) {
                rightPower = -MAX_PWR;
            }
            else{
                rightPower = MAX_PWR ;
            }
        }
        leftPower  = - rightPower;

        _motorRight1.setPower(rightPower);
        _motorRight2.setPower(rightPower);
        _motorLeft1.setPower(leftPower);
        _motorLeft2.setPower(leftPower);
    }


    //returns 0 if the heading == TargetHeading
    private int setHeading(double TargetHeading){
        double currentHeading ;
        double theta ;
        int returnVal = 0 ;

        currentHeading = 0; //_imu.getAngularOrientation().heading;
        theta = (TargetHeading - currentHeading) % 360;
        // mess with margin of error
        returnVal = 1 ;
        if ( Math.abs(theta) > ERR_MRGN ){
            adjustAngle(theta);
        }
        else {
            cntAcc ++ ;
            if (cntAcc > 50){
                returnVal = 0 ;
                cntAcc = 0 ;
            }
        }

        return returnVal ;
    }

    public void moveTo(double heading) {
        while (setHeading(heading)== 1){
        }
    }

    // ********************  //
    //  Follow Line Methods
    // ********************  //
    public int getFollowLineDirection() {
        double random = Math.random(); // user this to flip a coin
        int direction = FORWARD;
        int left_on = 0;
        int right_on = 0;
        int dir = 0;

        if (_reflectedLightLeft.getLightDetected() > LIGHT_THRESHOLD) {
            left_on = 1;
        }
        if (_reflectedLightRight.getLightDetected() > LIGHT_THRESHOLD) {
            right_on = 1;
        }
        dir = left_on * 10 + right_on; // gives 0,1,10,11
        switch (dir) {
            case 11:
                direction = FORWARD;
                break;
            case 01:
                direction = LEFT;
                break;
            case 10:
                direction = RIGHT;
                break;
            case 00:
                if (random < 0.50) {
                    direction = LEFT;
                } else {
                    direction = RIGHT;
                }
                break;
        }
        return direction;
    }


}
