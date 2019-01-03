package org.firstinspires.ftc.teamcode8515.hardware;

import android.content.Context;
import android.hardware.SensorManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode8515.hardware.device.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode8515.hardware.device.GyroSystemWrapper;
import org.firstinspires.ftc.teamcode8515.hardware.device.GyroWrapper;
import org.firstinspires.ftc.teamcode8515.hardware.device.ServoWrapper;
import org.firstinspires.ftc.teamcode8515.hardware.subModule.DriveNavigator;
import org.firstinspires.ftc.teamcode8515.hardware.subModule.DriveSystemInterface;
import org.firstinspires.ftc.teamcode8515.hardware.subModule.DriverMecanum;
import org.firstinspires.ftc.teamcode8515.hardware.subModule.VisionDetector;

/**
 *
 * 前面吞，后面吐的机器人方案
 * 底盘使用4个万向轮驱动
 *      目前的问题是横向运动受地面摩擦力影响，准确度不高
 *
 *
 */
public class HardwareBotGulosity {

    private LinearOpMode _hostOpMode = null;

    //驱动底盘
    public DriveSystemInterface drive = null;
    public DriveNavigator navigator = null;
    public VisionDetector vision = null ;

    //用于驱动吸取符文的滚轮的马达
    private DcMotor motorWheelLeft = null;
    private DcMotor motorWheelRight = null;
    private ServoWrapper servoPushGlyph = null;

    //用于驱动升降 存放符文的平台 的马达
    private DcMotor motorLiftLeft = null;
    private DcMotor motorLiftRight = null;

    //用于驱动翻转装置的马达，使存放符文的平台可以翻转，将符文放置到密码箱中
    private ServoWrapper servoFlipLeft = null;
    private ServoWrapper servoFlipRight = null;

    //用于识别宝珠的硬件
    private ServoWrapper servoSensorArm = null;
    private ServoWrapper servoJewelKick = null;

    private ColorSensorWrapper sensorColor = null;
    private DistanceSensor sensorDistance = null;
    private GyroSystemWrapper gyro = null;
    //private GyroWrapper gyro = null;

    /* local OpMode members. */
    private HardwareMap hwMap = null;

    /************************************************************************
     * Vuforia
     * **********************************************************************/

    //识别出的密码项，取值在left，center，right，unknown
    public String vumarkCode="unknown";
    //起点
    public  int startPoint        = 1;
    //当前自己所属联盟的颜色；默认红色
    public  String startColor     = C.COLOR_OF_RED_ALLIANCE;

    /* Constructor */
    public HardwareBotGulosity(LinearOpMode op) {
        _hostOpMode = op;
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
        // Save reference to Hardware map
        hwMap = _hostOpMode.hardwareMap;
        // Define and Initialize Motors for drive
        drive = new DriverMecanum();
        try {
            drive.setMotor(D.MOTOR_DRIVE_LEFT_FRONT,
                    hwMap.dcMotor.get(D.MOTOR_DRIVE_LEFT_FRONT));
            drive.setMotor(D.MOTOR_DRIVE_RIGHT_FRONT,
                    hwMap.dcMotor.get(D.MOTOR_DRIVE_RIGHT_FRONT));
            drive.setMotor(D.MOTOR_DRIVE_LEFT_REAR,
                    hwMap.dcMotor.get(D.MOTOR_DRIVE_LEFT_REAR));
            drive.setMotor(D.MOTOR_DRIVE_RIGHT_REAR,
                    hwMap.dcMotor.get(D.MOTOR_DRIVE_RIGHT_REAR));

            drive.setDirection(D.MOTOR_DRIVE_LEFT_FRONT,
                    DcMotor.Direction.FORWARD);
            drive.setDirection(D.MOTOR_DRIVE_LEFT_REAR,
                    DcMotor.Direction.FORWARD);
            drive.setDirection(D.MOTOR_DRIVE_RIGHT_FRONT,
                    DcMotor.Direction.REVERSE);
            drive.setDirection(D.MOTOR_DRIVE_RIGHT_REAR,
                    DcMotor.Direction.REVERSE);

            drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.driveStop();
        } catch (Exception ex) {
        }

        //用于驱动吞吐符文的滚轮的马达
        try {
            motorWheelLeft = hwMap.dcMotor.get(D.MOTOR_WHEEL_LEFT);
            motorWheelRight = hwMap.dcMotor.get(D.MOTOR_WHEEL_RIGHT);

            motorWheelLeft.setDirection(DcMotor.Direction.REVERSE);
            motorWheelRight.setDirection(DcMotor.Direction.FORWARD);

            motorWheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorWheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorWheelLeft.setPower(0);
            motorWheelRight.setPower(0);

        } catch (Exception ex) {
        }

        //用于驱动升降 存放符文的平台 的马达
        try {
            motorLiftLeft = hwMap.dcMotor.get(D.MOTOR_LIFT_LEFT);
            motorLiftLeft.setDirection(DcMotor.Direction.FORWARD);
            motorLiftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLiftLeft.setPower(0);

            motorLiftRight = hwMap.dcMotor.get(D.MOTOR_LIFT_RIGHT);
            motorLiftRight.setDirection(DcMotor.Direction.REVERSE);
            motorLiftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLiftRight.setPower(0);
        } catch (Exception ex) {
        }

        //用于驱动翻转装置的马达，使存放符文的平台可以翻转，将符文放置到密码箱中
        try {
            servoFlipLeft = new ServoWrapper( hwMap.servo.get(D.SERVO_FLIP_LEFT),
                    0,
                    1,
                    C.SERVO_FLIP_INIT_POSITION,
                    2000);
            servoFlipRight = new ServoWrapper( hwMap.servo.get(D.SERVO_FLIP_RIGHT),
                    0,
                    1,
                    1-C.SERVO_FLIP_INIT_POSITION,
                    2000);
        } catch (Exception ex) {
        }

        // 控制颜色传感器上下俯仰的伺服
        try {
            servoSensorArm = new ServoWrapper(hwMap.servo.get(D.SERVO_SENSOR_ARM),
                    0,
                    1,
                    C.SERVO_SENSOR_ARM_INIT,
                    2000);
        } catch (Exception ex) {
        }

        // 控制颜色传感器上下俯仰的伺服
        try {
            servoPushGlyph = new ServoWrapper(hwMap.servo.get(D.SERVO_PUSH_GLYPH),
                    0,
                    1,
                    C.SERVO_GLYPH_PUSH_INIT,
                    2000);
        } catch (Exception ex) {
        }

        // 控制左右旋转击打宝珠的伺服
        try {
            servoJewelKick = new ServoWrapper(hwMap.servo.get(D.SERVO_JEWEL_KICK),
                    0,
                    1,
                    C.SERVO_INIT_POS,
                    2000);
        } catch (Exception ex) {
        }

        // 颜色识别传感器
        try {
            sensorColor = new ColorSensorWrapper(hwMap.get(ColorSensor.class, D.SENSOR_COLOR));
        } catch (Exception ex) {
        }

        vision = new VisionDetector(_hostOpMode);
        navigator = new DriveNavigator(_hostOpMode, drive,vision);

        // REV内部传感器
        try {
            Context context = _hostOpMode.hardwareMap.appContext;
            gyro = new GyroSystemWrapper((SensorManager)context.getSystemService(context.SENSOR_SERVICE));
//            gyro = new GyroWrapper( _hostOpMode.hardwareMap.get(BNO055IMU.class, D.IMU));
            navigator.setImu(gyro);
        } catch (Exception ex) {
        }

    }

    public void initForAuto() {
        //servoPushGlyph.setPosition(SERVO_PUSH_GLYPH_READY_POSITION);
        flipHor();
        //vision.init();
        glyphPushDown();
    }

    public void start(){
        //vision.start();
        robotCalibration();
    }


    /**
     *
     * */
    public void robotCalibration(){
        drive.robotCalibration();
        navigator.robotCalibration();
    }

    public void initForTele() {
    }

    /**
     * =====自动阶段==================
     * */
    double jewelKickElapseTick = 0;

    public void setServoJewelKickPosition(double position) {
        servoJewelKick.setPosition(position);
    }

    public void setCode(String vumarkCode) {
        this.vumarkCode = vumarkCode;
    }

    public void setServoSensorArmPosition(double position){
        servoSensorArm.setPosition(position);
    }

    public boolean servoSensorArmDown(double timeInMilsec){
        return servoSensorArm.turnToPosition(C.SERVO_SENSOR_ARM_DOWN_POSITION,timeInMilsec);
    }

    public boolean servoSensorArmUp(double timeInMilsec){
        return servoSensorArm.turnToPosition(C.SERVO_SENSOR_ARM_UP_POSITION,timeInMilsec);
    }

    public void autoJewelKickStart(double tick){
        jewelKickElapseTick = tick;
        int tryCount = 0;
        while (_hostOpMode.opModeIsActive() && tryCount<4) {
            if (isColorBlue()) {
                if (startColor.equals(C.COLOR_OF_BLUE_ALLIANCE)) {
                    servoJewelKick.setPosition(C.SERVO_JEWEL_KICK_RIGHT_POSITION);
                } else {
                    servoJewelKick.setPosition(C.SERVO_JEWEL_KICK_LEFT_POSITION);
                }
                break;
            } else if (isColorRed()) {
                if (startColor.equals(C.COLOR_OF_BLUE_ALLIANCE)) {
                    servoJewelKick.setPosition(C.SERVO_JEWEL_KICK_LEFT_POSITION);
                } else {
                    servoJewelKick.setPosition(C.SERVO_JEWEL_KICK_RIGHT_POSITION);
                }
                break;
            }

            tryCount++;
            _hostOpMode.sleep(500);
        }
    }

    public void returnJewel(){servoJewelKick.setPosition(C.SERVO_JEWEL_KICK_INIT_POSITION);}

    /**
     * 将角度正则化到-180到180度之间
     */
    private double robotAngleNormalize(double robotAngle) {
        if (robotAngle >= -180 && robotAngle <= 180)
            return robotAngle;
        return (robotAngle + 180) % 360 - 180;
    }

    //==============手动阶段功能====================
    /**
     * 滚轮正向启动
     * 捕获、吞入符文
     */
    public void wheelSwallowStart() {
        motorWheelLeft.setPower(C.WHEEL_SPEED);
        motorWheelRight.setPower(C.WHEEL_SPEED);
    }

    /**
     * 滚轮逆向启动
     * 吐出符文
     */
    public void wheelVomitStart() {
        motorWheelLeft.setPower(-C.WHEEL_SPEED);
        motorWheelRight.setPower(-C.WHEEL_SPEED);
    }

    /**
     * 滚轮停止运动
     */
    public void wheelStop() {
        motorWheelLeft.setPower(0);
        motorWheelRight.setPower(0);
    }

    /**
     * 抬升 存放符文的平台
     * 此处应当根据
     */
    public void liftUp() {
        motorLiftLeft.setPower(C.LIFT_SPEED);
        motorLiftRight.setPower(C.LIFT_SPEED);
    }

    /**
     * 降低 存放符文的平台
     */
    public void liftDown() {
        motorLiftLeft.setPower(-C.LIFT_SPEED);
        motorLiftRight.setPower(-C.LIFT_SPEED);
    }

    /**
     * 停止升降动作
     */
    public void liftStop() {
        motorLiftLeft.setPower(0);
        motorLiftRight.setPower(0);
    }

    /**
     * 翻转平台，将符文放入密码箱
     */
    public boolean flipUp(double timeInMilsec) {
        servoFlipRight.turnToPosition(1-C.SERVO_FLIP_UP_POSITION,timeInMilsec);
        return servoFlipLeft.turnToPosition(C.SERVO_FLIP_UP_POSITION,timeInMilsec);
    }

    /**
     * 翻转平台，将符文放入密码箱
     */
    public void flipUp() {
        servoFlipLeft.setPosition(C.SERVO_FLIP_UP_POSITION);
        servoFlipRight.setPosition(1-C.SERVO_FLIP_UP_POSITION);
    }


    /**
     * 平台向下倾斜，可以接收方块
     */
    public void flipDown() {
        servoFlipLeft.setPosition(C.SERVO_FLIP_READY_POSITION);
        servoFlipRight.setPosition(C.SERVO_FLIP_READY_POSITION);
    }

    /**
     * 平台返回水平状态
     */
    public void flipHor() {
        servoFlipLeft.setPosition(C.SERVO_FLIP_INIT_POSITION);
        servoFlipRight.setPosition(1-C.SERVO_FLIP_INIT_POSITION);
    }

    /**
     * 当前颜色传感器读取的值是不是蓝色
     */
    public boolean isColorBlue() {
        return sensorColor.isColorBlue();
    }

    /**
     * 当前颜色传感器读取的值是不是红色
     */
    public boolean isColorRed() {
        return sensorColor.isColorRed();
    }

    public void glyphPushInit() {
        servoPushGlyph.setPosition(C.SERVO_GLYPH_PUSH_INIT);
    }

    public void glyphPushDown() {
        servoPushGlyph.setPosition(C.SERVO_GLYPH_PUSH_DOWN);
    }
}
