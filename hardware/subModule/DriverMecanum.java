package org.firstinspires.ftc.teamcode8515.hardware.subModule;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode8515.hardware.C;
import org.firstinspires.ftc.teamcode8515.hardware.D;
import org.firstinspires.ftc.teamcode8515.utils.FtcLog;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by zhuxu on 2018/4/19.
 */
public class DriverMecanum implements DriveSystemInterface {

    //用于驱动底盘上的4个麦轮的马达
    private Map<String,DcMotor> motors = new HashMap<>();
    private double driveSpeedScale = C.DRIVE_SPEED_SCALE_MAX;


    @Override
    public void robotCalibration() {

    }

    @Override
    public void setSpeedScale(double value) {
        driveSpeedScale = value;
    }

    @Override
    public void setSpeedScaleHigh() {
        driveSpeedScale = C.DRIVE_SPEED_SCALE_MAX;
    }

    @Override
    public void setSpeedScaleSlow() {
        driveSpeedScale = C.DRIVE_SPEED_SCALE_MIN;
    }

    @Override
    public void setMotor(String name, DcMotor motor){
        motors.put(name,motor);
    }

    @Override
    public void setPower(double power){
        for (DcMotor motor : motors.values()){
            motor.setPower(power);
        }
    }

    @Override
    public void setPower(String name, double power){
        motors.get(name).setPower(power);
    }

    @Override
    public boolean isAllMotorBusy(){
        boolean isBusy = false;
        for (DcMotor motor : motors.values()){
            if(motor.isBusy()){
                isBusy = true;
                break;
            }
        }
        return isBusy;
    }

    @Override
    public void driveStop() {
        setPower(0);
    }


    @Override
    public void setMode(DcMotor.RunMode mode) {
        for (DcMotor motor : motors.values()){
            motor.setMode(mode);
        }
    }

    @Override
    public int getDrivePosition(String name) {
        return motors.get(name).getCurrentPosition();
    }


    @Override
    public void setDirection(String name, DcMotorSimple.Direction direction) {
        motors.get(name).setDirection(direction);
    }

    @Override
    public void setTargetPosition(String name, int newTarget) {
        motors.get(name).setTargetPosition(newTarget);
    }

    @Override
    public void runToNewTargetInit(double distance) {
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        moveCounts = (int)(distance * C.COUNTS_PER_INCH);
        newLeftTarget = getDrivePosition(D.MOTOR_DRIVE_LEFT_FRONT) + moveCounts;
        newRightTarget = getDrivePosition(D.MOTOR_DRIVE_RIGHT_FRONT) + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        setTargetPosition(D.MOTOR_DRIVE_LEFT_FRONT, newLeftTarget);
        setTargetPosition(D.MOTOR_DRIVE_LEFT_REAR, newLeftTarget);
        setTargetPosition(D.MOTOR_DRIVE_RIGHT_FRONT, newRightTarget);
        setTargetPosition(D.MOTOR_DRIVE_RIGHT_REAR, newRightTarget);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void runToNewTargetInit(double leftInches, double rightInches) {
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCountsLeft;
        int     moveCountsRight;
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        moveCountsLeft = (int)(leftInches * C.COUNTS_PER_INCH);
        moveCountsRight = (int)(rightInches * C.COUNTS_PER_INCH);
        newLeftTarget = getDrivePosition(D.MOTOR_DRIVE_LEFT_FRONT) + moveCountsLeft;
        newRightTarget = getDrivePosition(D.MOTOR_DRIVE_RIGHT_FRONT) + moveCountsRight;

        // Set Target and Turn On RUN_TO_POSITION
        setTargetPosition(D.MOTOR_DRIVE_LEFT_FRONT,newLeftTarget);
        setTargetPosition(D.MOTOR_DRIVE_LEFT_REAR,newLeftTarget);
        setTargetPosition(D.MOTOR_DRIVE_RIGHT_FRONT,newRightTarget);
        setTargetPosition(D.MOTOR_DRIVE_RIGHT_REAR,newRightTarget);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


    /**
     * 按控制杆输入控制机器人行进方向
     */
    public void driveByStick(double xOffset, double yOffset, double yawOffset) {
        //如果x和y的值都接近或等于0，则停止电机
        if (Math.abs(xOffset) < 0.01
                && Math.abs(yOffset) < 0.01
                && Math.abs(yawOffset) < 0.01
                ) {
            driveStop();

            return;
        }

        //确保不超范围
        xOffset = Math.min(xOffset * C.DRIVE_MAX_SPEED, C.DRIVE_MAX_SPEED);
        yOffset = Math.min(yOffset * C.DRIVE_MAX_SPEED, C.DRIVE_MAX_SPEED);

        FtcLog.i("power1",""+xOffset+"==="+yOffset);
        //按设置调整力度
        xOffset *= driveSpeedScale;
        yOffset *= driveSpeedScale;
        yawOffset *= driveSpeedScale;
        FtcLog.i("power2",""+xOffset+"==="+yOffset);

        // 根据勾股定理计算stick离中心点的距离，以计算速度
        // 再根据角度计算各个马达的力度
        double speed = Math.hypot(xOffset, yOffset);//根据勾股定理获取stick的真实距离
        double robotAngle = Math.atan2(yOffset, xOffset);//
        FtcLog.i("power3",""+speed+" "+robotAngle);

        driveByAngle(speed, robotAngle, yawOffset);

    }
    /**
     * 按角度驾驶
     *
     * @param robotSpeed 机器人速度，-1 到 1
     * @param robotAngle 角度，-PI 到 PI，以机器人正方向为基准取值0，机器人背面为PI或者-PI；
     * @param yawSpeed   转向速度，-1 到 1
     */
    public void driveByAngle(double robotSpeed, double robotAngle, double yawSpeed) {

        // 转换角度到麦轮控制坐标系
        // 机器人俯视角度，正方向向上。
        // 麦轮坐标系：
        //      y轴方向为右下轮轴到左上轮轴，左上为正方向；
        //      x轴方向为左下轮轴到右上轮轴，右上为正方向。
        robotAngle -= Math.PI / 4;
        final double vLeftFront = robotSpeed * Math.cos(robotAngle) + yawSpeed;
        final double vRightFront = robotSpeed * Math.sin(robotAngle) - yawSpeed;
        final double vLeftRear = robotSpeed * Math.sin(robotAngle) + yawSpeed;
        final double vRightRear = robotSpeed * Math.cos(robotAngle) - yawSpeed;
        FtcLog.i("power4","lf"+vLeftFront+"lr"+vLeftRear+"rf"+vRightFront+"rr"+vRightRear,
                true,true);

        setPower(D.MOTOR_DRIVE_LEFT_FRONT,vLeftFront);
        setPower(D.MOTOR_DRIVE_LEFT_REAR,vLeftRear);
        setPower(D.MOTOR_DRIVE_RIGHT_FRONT,vRightFront);
        setPower(D.MOTOR_DRIVE_RIGHT_REAR,vRightRear);

    }

}
