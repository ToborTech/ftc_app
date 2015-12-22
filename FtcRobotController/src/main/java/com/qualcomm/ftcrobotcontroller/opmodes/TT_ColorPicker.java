package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Niki Monsef on 12/10/2015.
 */

public class TT_ColorPicker {
    ColorSensor _colorSensor;
    //DeviceInterfaceModule cdim;
    //TouchSensor t;
    final static int BLUE = 1;
    final static int RED = 2;
    final static int UNKNOWN = 0;
    final static float COLOR_THRESHOLD = 2;
    float _currentRed  ;
    float _currentBlue ;

    float[] _runningRed  = new float[10] ;
    float[] _runningBlue = new float[10] ;

    TT_ColorPicker(ColorSensor colorSensor){
        _colorSensor = colorSensor ;
        for ( int i = 0 ; i < 10 ; i++){
            _runningRed[i] = 0 ;
            _runningBlue[i]= 0 ;
        }
    }

    static  int count = 0;
    float red_acc = 0 , blue_acc = 0 , red_final = 0 , blue_final = 0;

    // 1 = blue ; 2 = red ; 0 = not sure
    public int getColor(){

        insertNewSamples( _colorSensor.red(), _colorSensor.blue());
        calcFinal();
        if ( _currentBlue > ( _currentRed + COLOR_THRESHOLD) ){
            return  BLUE; // 1 = Blue
        }
        else if ( _currentRed > ( _currentBlue + COLOR_THRESHOLD )){
            return RED ; // 2 = Red
        }
        else {
            return UNKNOWN ; // not sure ( not enough difference )
        }
    }

    private void insertNewSamples(float red, float blue){
        for ( int i = 9 ; i >= 1 ; i-- ) {
            _runningRed[i]=_runningRed[i-1] ; // 0->1 ; 1->2 ; ... ; 10-> drop
            _runningBlue[i]=_runningBlue[i-1] ; // 0->1 ; 1->2 ; ... ; 10-> drop
        }
        _runningRed[0]  = red ;
        _runningBlue[0] = blue ;
    }

    private void calcFinal(){
        float sumOfRed  = 0 ;
        float sumOfBlue = 0 ;
        for ( int i = 0 ; i < 10 ; i++ ){
            sumOfBlue += _runningBlue[i] ;
            sumOfRed  += _runningRed[i]  ;
        }
        _currentRed  = sumOfRed / 10 ;
        _currentBlue = sumOfBlue / 10  ;
    }
}
