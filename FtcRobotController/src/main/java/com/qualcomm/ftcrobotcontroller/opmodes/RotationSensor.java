/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

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

package com.qualcomm.ftcrobotcontroller;

import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Binder;
import android.os.IBinder;

/**
 * Created by hanst on 8/18/2015.
 */


public class RotationSensor extends Service implements SensorEventListener {
  private SensorManager mSensorManager;
  private Sensor mRotationSensor;
  float currRaw=0;
  State state=State.INIT;
  public enum State {
    INIT,
    REGISTER,
    UNREGISTER
  }
  /*
      public RotationSensor() {

          // mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
          // mRotationSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
          currRaw = 0;
          state = State.INIT;

      }
  */
  @Override
  public void onCreate() {
    mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
    mRotationSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
    currRaw = 0;
    state = State.INIT;
    register();
  }

  @Override
  public IBinder onBind(Intent intent) {
    if (mSensorManager==null)
      mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
    if (mRotationSensor==null)
      mRotationSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
    register();
    return mBinder;
  }

  @Override
  public void onDestroy() {
    unregister();
  }

  // This is the object that receives interactions from clients.  See
  // RemoteService for a more complete example.

  public class LocalBinder extends Binder {
    RotationSensor getService() {
      return RotationSensor.this;
    }
  }
  private final IBinder mBinder = new LocalBinder();


  public void register() {
    if (null != mRotationSensor) {
      mSensorManager.registerListener(this, mRotationSensor,
              SensorManager.SENSOR_DELAY_FASTEST);
      state = State.REGISTER;
    }
  }

  public void unregister() {
    if (mSensorManager!=null) {
      mSensorManager.unregisterListener(this);
      state = State.UNREGISTER;
    }
  }

  public void onSensorChanged(SensorEvent event) {
    currRaw = event.values[0];
  }

  public float getCurrRaw() { return currRaw; }
  public State getState() { return state; }
  public void onAccuracyChanged(Sensor sensor, int accuracy) {

  }

}


