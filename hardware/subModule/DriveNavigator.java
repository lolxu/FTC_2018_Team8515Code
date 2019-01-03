package org.firstinspires.ftc.teamcode8515.hardware.subModule;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode8515.arena.ArenaMap;
import org.firstinspires.ftc.teamcode8515.hardware.C;
import org.firstinspires.ftc.teamcode8515.hardware.D;
import org.firstinspires.ftc.teamcode8515.hardware.device.GyroSystemWrapper;
import org.firstinspires.ftc.teamcode8515.hardware.device.GyroWrapper;
import org.firstinspires.ftc.teamcode8515.utils.FtcLog;

import static java.lang.String.*;

/**
 * Created by zhuxu on 2018/4/21.
 */

public class DriveNavigator {
    //用于驱动底盘上的4个麦轮的马达

    private LinearOpMode op = null;

    private ArenaMap map = null;

    private DriveSystemInterface drive = null;

    private GyroWrapper imu1 = null;
    private GyroSystemWrapper imu = null;

    private VisionDetector vision = null;

    public DriveNavigator(){
        this.map = new ArenaMap();
    }

    public DriveNavigator(LinearOpMode op, DriveSystemInterface drive ){
        this.op = op;
        this.drive = drive;
        this.map = new ArenaMap();
    }

    public DriveNavigator(LinearOpMode op, DriveSystemInterface drive ,VisionDetector vision){
        this.op = op;
        this.drive = drive;
        this.map = new ArenaMap();
        this.vision = vision;
    }

    private double angleInitOffset = 0;

    public void setImu(GyroWrapper imu){
        //this.imu = imu;
    }

    public void setImu(GyroSystemWrapper imu){
        this.imu = imu;
    }

    public void initForAuto(){
        if(vision!=null){
            angleInitOffset = vision.getAngle();
        }
    }

    public void robotCalibration(){
        if(vision!=null){
            angleInitOffset = vision.getAngle();
        }
        if(imu!=null){
            imu.calibrate();
        }
    }

    public void gyroCalibration(){
        if(imu!=null){
            imu.calibrate();
        }
    }

    /**
     *  ==========路径规划、驾驶相关===============
     * */

    public void driveDirect(double speed,
                            double distance,
                            double angle,
                            double timeOutMs){
        if(imu != null){
            //angleInitOffset = 0;
            driveByGyro(speed,distance,angle + angleInitOffset,timeOutMs);
            FtcLog.i("angleInitOffset:",""+angleInitOffset,true,true);
            FtcLog.i("angle:",""+angle+angleInitOffset,true,true);
            angleInitOffset = 0;
        }else{
            driveByEncoder(speed,distance,distance,timeOutMs);
        }

    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angleOffset      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void driveTurn(double speed, double angleOffset) {
        if(imu != null){
            driveTurnByGyro(speed,angleOffset);
            angleInitOffset = 0;
        }else{
            double turnDistance = (7.5 * Math.PI/2)/angleOffset*20;
            driveByEncoder(speed,turnDistance,turnDistance,3000);
        }
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      目标角度，正负180度。驾驶过程中将保持机器人头部指向这个角度。
     * @param timoutMs      最大执行时间。
     */
    private void driveByGyro(double speed,
                            double distance,
                            double angle,
                             double timoutMs) {

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        ElapsedTime timer = new ElapsedTime();

        // Ensure that the opmode is still active
        if (op.opModeIsActive()) {
            drive.runToNewTargetInit(distance);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            drive.setPower(speed);
            //op.sleep(50);
            timer.reset();
            // keep looping while we are still active, and BOTH motors are running.
            double loopCount = 0;
            while (op.opModeIsActive()
                   && (drive.isAllMotorBusy()||timer.milliseconds()<100)
                   && timer.milliseconds()< timoutMs) {
                loopCount++;
                // 调整角度偏差.
                error = getError(angle);
                steer = getSteer(error, C.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                if(max > Math.abs(speed)){
                    leftSpeed *= Math.abs(speed);
                    rightSpeed *= Math.abs(speed);
                }

                drive.setPower(D.MOTOR_DRIVE_LEFT_FRONT, leftSpeed);
                drive.setPower(D.MOTOR_DRIVE_LEFT_REAR, leftSpeed);
                drive.setPower(D.MOTOR_DRIVE_RIGHT_FRONT, rightSpeed);
                drive.setPower(D.MOTOR_DRIVE_RIGHT_REAR, rightSpeed);

                // Display drive status for the driver.
                FtcLog.i("Err/St/loop",  format("%5.1f/%5.1f/%5.1f",  error, steer,loopCount));
                FtcLog.i("Actual",  format("%7d:%7d",
                        drive.getDrivePosition(D.MOTOR_DRIVE_LEFT_FRONT),
                        drive.getDrivePosition(D.MOTOR_DRIVE_RIGHT_FRONT)));
                FtcLog.i("Speed",  format("%5.2f:%5.2f",  leftSpeed, rightSpeed),true,true);

            }

            // Stop all motion;
            drive.driveStop();
            // Turn off RUN_TO_POSITION
            drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void driveByEncoder(double speed,
                               double leftInches, double rightInches,
                               double timeoutMS) {

        drive.runToNewTargetInit(leftInches,rightInches);
        // reset the timeout time and start motion.
        drive.setPower(Math.abs(speed));

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (op.opModeIsActive() &&
                (runtime.milliseconds() < timeoutMS) &&
                (drive.isAllMotorBusy())) {
            op.idle();
        }
        // Stop all motion;
        drive.driveStop();
        // Turn off RUN_TO_POSITION
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveTurnByGyro(double speed, double angleOffset) {
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.calibrate();
        double angle = getCurrentAngle() + angleOffset;
        // 按角度变化，持续转向
        double loopCount= 0 ;
        while (op.opModeIsActive() && !onHeading(speed, angle, C.P_TURN_COEFF)) {
            FtcLog.i("target angle:",""+angle+" curr="+getCurrentAngle());
            FtcLog.i("loop:",""+loopCount);
            op.idle();
        }
    }

    /**
     * 旋转到指定角度.
     *
     * @param speed     Desired speed of turn.
     * @param angle     旋转度数绝对数值，即停止旋转的角度。正负180之间。顺时针为正，逆时针为负。.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed=0;
        double rightSpeed=0;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= C.HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }
        FtcLog.i("onHeading:","err="+error+" steer="+steer +" speed="+rightSpeed,
                true,true);

        // 用微调后的power数值设置马达.
        drive.setPower(D.MOTOR_DRIVE_LEFT_FRONT,leftSpeed);
        drive.setPower(D.MOTOR_DRIVE_LEFT_REAR,leftSpeed);
        drive.setPower(D.MOTOR_DRIVE_RIGHT_FRONT,rightSpeed);
        drive.setPower(D.MOTOR_DRIVE_RIGHT_REAR,rightSpeed);

        return onTarget;
    }


    public double getCurrentAngle(){
        return imu.getAngularRotationY();
    }
    /**
     * 获取期望的调整力returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        //
        //return Range.clip(error * PCoeff, -1, 1);
        return Range.clip(error * PCoeff, -1, 1);
    }

    /**
     * getError 检测当前陀螺仪角度和目标角度之间的误差值。
     * todo:应当设计机制避免瞬时的大数值影响。
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getCurrentAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public void stop() {
        imu.unregistSysGyro();
    }
}
