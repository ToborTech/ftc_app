package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Niki Monsef on 12/10/2015.
 */


public class TT_ColorPicker {
    public enum Color {
        UNKNOWN,     // unknown color (other than blue or red)
        BLUE,        // blue color
        RED          // red color
    }

    ColorSensor _colorSensor;
    //DeviceInterfaceModule cdim;
    //TouchSensor t;
    final static double COLOR_THRESHOLD = 0.95;
    double _currentRed  ;
    double _currentBlue ;

    double[] _runningRed  = new double[10] ;
    double[] _runningBlue = new double[10] ;

    TT_ColorPicker(ColorSensor colorSensor){
        _colorSensor = colorSensor ;
        for ( int i = 0 ; i < 10 ; i++){
            _runningRed[i] = 0 ;
            _runningBlue[i]= 0 ;
        }
    }

    public Color getColor(){

        insertNewSamples( _colorSensor.red(), _colorSensor.blue());
        calcFinal();
        if ( _currentBlue > ( _currentRed + COLOR_THRESHOLD) ){
            return  Color.BLUE; // 1 = Blue
        }
        else if ( _currentRed > ( _currentBlue + COLOR_THRESHOLD )){
            return Color.RED ; // 2 = Red
        }
        else {
            return Color.UNKNOWN ; // not sure ( not enough difference )
        }
    }

    private void insertNewSamples(double red, double blue){
        for ( int i = 9 ; i >= 1 ; i-- ) {
            _runningRed[i]=_runningRed[i-1] ; // 0->1 ; 1->2 ; ... ; 10-> drop
            _runningBlue[i]=_runningBlue[i-1] ; // 0->1 ; 1->2 ; ... ; 10-> drop
        }
        _runningRed[0]  = red ;
        _runningBlue[0] = blue ;
    }

    private void calcFinal(){
        double sumOfRed  = 0 ;
        double sumOfBlue = 0 ;
        for ( int i = 0 ; i < 10 ; i++ ){
            sumOfBlue += _runningBlue[i] ;
            sumOfRed  += _runningRed[i]  ;
        }
        _currentRed  = sumOfRed / 10 ;
        _currentBlue = sumOfBlue / 10  ;
    }
}
