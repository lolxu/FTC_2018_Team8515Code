package org.firstinspires.ftc.teamcode8515.hardware.subModule;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by zhuxu on 2018/4/19.
 */

public interface DriveSystemInterface {

    void robotCalibration();

    void setSpeedScale(double value);
    void setSpeedScaleHigh();
    void setSpeedScaleSlow();

    void setMotor(String name, DcMotor motor);

    void setPower(double power);

    void setPower(String name, double power);

    boolean isAllMotorBusy();

    void driveStop();

    void setMode(DcMotor.RunMode mode);

    int getDrivePosition(String name);

    void setDirection(String name, DcMotorSimple.Direction direction);

    void setTargetPosition(String name, int newTarget);

    void runToNewTargetInit(double distance);

    void runToNewTargetInit(double leftInches, double rightInches);

    void driveByStick(double xOffset, double yOffset, double yawOffset);

    void driveByAngle(double robotSpeed, double robotAngle, double yawSpeed);
}
