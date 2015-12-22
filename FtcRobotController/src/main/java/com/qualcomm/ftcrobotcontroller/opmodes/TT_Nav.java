package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

// import org.swerverobotics.library.interfaces.IBNO055IMU;

public class TT_Nav {

    // Defining constants for drive direction
    final static int FORWARD    = 0 ;
    final static int BACKWARD   = 1 ;
    final static int LEFT       = 2 ;
    final static int RIGHT      = 3 ;
    final static int BRAKE      = 4 ;

    //final static double MOTOR_POWER = 0.2;// Higher values will cause the robot to move faster
    final static double MIN_PWR = 0.15 ;
    final static double MAX_PWR = 0.99 ;
    final static double ERR_MRGN = 0.05 ;

    final static double P = 0.0045;
    double cntAcc;

    private DcMotor _motorLeft ;
    private DcMotor _motorRight ;
    // private IBNO055IMU _imu ;
    private LightSensor _reflectedLightLeft, _reflectedLightRight ;

    final static double LIGHT_THRESHOLD = 0.4;

    TT_Nav(DcMotor motorR, DcMotor motorL, boolean enableFollowLine, LightSensor reflectedLightLeft,LightSensor reflectedLightRight){
        _motorLeft  = motorL ;
        _motorRight = motorR ;
        // _imu = imu ;
        if (enableFollowLine) {
            _reflectedLightLeft = reflectedLightLeft;
            _reflectedLightLeft.enableLed(true);  // turn on LED of light sensor.
            _reflectedLightRight = reflectedLightRight;
            _reflectedLightRight.enableLed(true);  // turn on LED of light sensor.
        }
    }

    // The setter functions
    public void set_motorRight ( DcMotor motor) {
            _motorRight = motor;
        }
    public void set_motorLeft ( DcMotor motor) {
        _motorLeft = motor;
    }
    // public void set_imu (IBNO055IMU imu ){
    //    _imu = imu ;
    //}


    public void drive (int direction , double power ){

        switch (direction) {
            case FORWARD :
                _motorRight.setDirection(DcMotor.Direction.FORWARD);
                _motorLeft.setDirection(DcMotor.Direction.REVERSE);
                _motorRight.setPower(power);
                _motorLeft.setPower(power);
                break;
            case BACKWARD:
                _motorRight.setDirection(DcMotor.Direction.REVERSE);
                _motorLeft.setDirection(DcMotor.Direction.FORWARD);
                _motorRight.setPower(power);
                _motorLeft.setPower(power);
                break;
            case LEFT:
                _motorRight.setPower(0);
                _motorLeft.setPower(power);
                break;
            case RIGHT:
                _motorRight.setPower(power);
                _motorLeft.setPower(0);
                break;
            case BRAKE:
                _motorRight.setPower(0);
                _motorLeft.setPower(0);
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

        _motorRight.setPower(rightPower);
        _motorLeft.setPower(leftPower);
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
public int getFollowLineDirection(){
    double random = Math.random(); // user this to flip a coin
    int direction = FORWARD ;
    int left_on  = 0 ;
    int right_on = 0 ;
    int dir = 0 ;

    if ( _reflectedLightLeft.getLightDetected() > LIGHT_THRESHOLD ){
        left_on = 1 ;
    }
    if ( _reflectedLightRight.getLightDetected() > LIGHT_THRESHOLD ){
        right_on = 1 ;
    }
    dir = left_on * 10 + right_on  ; // gives 0,1,10,11
    switch (dir){
        case 11: direction = FORWARD ; break;
        case 01: direction = LEFT    ; break;
        case 10: direction = RIGHT   ; break;
        case 00 :
            if ( random < 0.50 ){
                direction = LEFT ;
            }
            else{
                direction = RIGHT ;
            }
            break;
    }
    return direction ;
}


}
