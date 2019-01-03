package org.firstinspires.ftc.teamcode8515.hardware.device;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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

public class GyroWrapper {
    private static double IMU_ADDER = 0;

    public BNO055IMU imu;
    public BNO055IMU.Parameters imuParameters;
    private AxesOrder order = AxesOrder.ZYX;
    private AxesReference ref = AxesReference.INTRINSIC;
    private AngleUnit unit = AngleUnit.DEGREES;
    private double initAngle = 0;

    public GyroWrapper(BNO055IMU imu) {
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(imuParameters);
        this.imu = imu;
    }


    public double getAngularRotationX() {
         return imu.getAngularOrientation(ref, order, unit).thirdAngle + IMU_ADDER;
    }

    public double getAngularRotationY() {
         return imu.getAngularOrientation(ref, order, unit).secondAngle + IMU_ADDER;
    }

    public double getAngularRotationZ() {
        return imu.getAngularOrientation(ref, order, unit).firstAngle + IMU_ADDER - initAngle;
    }
    public void calibrate(){
        initAngle = imu.getAngularOrientation(ref, order, unit).firstAngle;
    }

}
