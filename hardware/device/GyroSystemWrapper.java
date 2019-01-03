package org.firstinspires.ftc.teamcode8515.hardware.device;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**

 * hub正面水平放置、文字在上面时
 * AxesOrder.ZYX
 * heading：angles.firstAngle；
 *      代表z轴变化，沿垂直hub平面垂线的轴，旋转hub。从正面上空向下观察，顺时针变小，逆时针变大
 * roll：angles.secondAngle；
 *      代表y轴变化，即沿文字方向为轴，旋转hub。从左向右（R-E-V方向）观察，顺时针变小，逆时针变大
 * pitch angles.thirdAngle；
 *      代表x轴变化，即沿垂直文字的直线为轴，旋转hub。从上（usb接口）向下（servo口）观察，顺时针变小，逆时针变大
 *
 */

public class GyroSystemWrapper implements SensorEventListener {

    private SensorManager mSensorManager;
    private Sensor mSensor;
    //zyx
    public volatile float[] mRawData = new float[3];
    public volatile float[] mGyroData = new float[3];
    private volatile float timestamp= 0;
    private static final float NS2S = 1.0f / 1000000000.0f;
    //private static final float NS2S = 1.0f / 1000000000.0f;

    private double initAngle = 0;

    public GyroSystemWrapper(SensorManager sensorManager){
        mSensorManager = sensorManager;
        mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mSensorManager.registerListener(this, mSensor, SensorManager.SENSOR_DELAY_FASTEST);
    }

    public void registSysGyro(SensorManager sensorManager){
        mSensorManager = sensorManager;
        mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mSensorManager.registerListener(this, mSensor, SensorManager.SENSOR_DELAY_FASTEST);
    }

    public void unregistSysGyro(){
        mSensorManager.unregisterListener(this);
    }

    public double getAngularRotationX() {
        return Math.toDegrees(mGyroData[2] - initAngle);
        //return imu.getAngularOrientation(ref, order, unit).thirdAngle + IMU_ADDER;
    }

    public double getAngularRotationY() {
        return Math.toDegrees(mGyroData[1] - initAngle);
        //return imu.getAngularOrientation(ref, order, unit).secondAngle + IMU_ADDER;
    }

    public double getAngularRotationZ() {
        return Math.toDegrees(mGyroData[0]) - initAngle;
        //return imu.getAngularOrientation(ref, order, unit).firstAngle + IMU_ADDER - initAngle;
    }
    public void calibrate(){
        timestamp = 0;
        initAngle = 0;
        mGyroData[0] = 0;
        mGyroData[1] = 0;
        mGyroData[2] = 0;

        //initAngle = imu.getAngularOrientation(ref, order, unit).firstAngle;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() != Sensor.TYPE_GYROSCOPE)
            return;

        if (timestamp != 0) {
            mRawData = event.values;
            final float dT = (event.timestamp - timestamp) * NS2S;
            mGyroData[0] += event.values[0] * dT;// * 100;
            mGyroData[1] += event.values[1] * dT;// * 100;
            mGyroData[2] += event.values[2] * dT;// * 100;

            mGyroData[0] -= event.values[0] * dT /10;// * 100;
            mGyroData[1] -= event.values[1] * dT /10;// * 100;
            mGyroData[2] -= event.values[2] * dT /10;// * 100;

        }
        timestamp = event.timestamp;
//
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

}
