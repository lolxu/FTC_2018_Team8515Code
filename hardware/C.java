package org.firstinspires.ftc.teamcode8515.hardware;

/**
 * Created by zhuxu on 2018/4/25.
 */

public class C {
    /************************************************************************
     * 常量
     * **********************************************************************/
    // 底盘最大速度。将以此值按比例缩减从遥控器传递的值。
    public static final double DRIVE_MAX_SPEED = 1.0;
    public static final double DRIVE_TURN_SPEED_AUTO = 0.3;
    public static final double DRIVE_SPEED_AUTO = 0.3;
    public static final double SPEED_MS_PER_INCH = 200;

    //底盘，手动阶段速度收缩最大幅度
    public static final double DRIVE_SPEED_SCALE_MAX = 0.8;
    //底盘，手动阶段速度收缩最小幅度
    public static final double DRIVE_SPEED_SCALE_MIN = 0.2;
    // 车轮驱动马达的编码，每圈的总计数
    public static final double COUNTS_PER_MOTOR_REV = 28 * 40;    // 每圈：28，reduction：40
    // gear up 比例，通过驱动马达到车轮间的传动齿轮确定。
    public static final double DRIVE_GEAR_REDUCTION = 28/18;     //
    // 车轮直径
    public static final double WHEEL_DIAMETER_INCHES = 6;     // For figuring circumference
    // 机器人总宽度
    public static final double ROBOT_WIDTH_DIAMETER_INCHES = 13.39;     // For figuring circumference

    // 每行进一英寸的编码数
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);


    // 吞吐符文的轮子的马达的速度
    public static final double WHEEL_SPEED = 0.3;
    // 抬升符文平台的马达
    public static final double LIFT_SPEED = 0.5;

    // 翻转符文平台的伺服上电时的初始位置，接近水平位置
    public static final double SERVO_FLIP_INIT_POSITION = 0.3;//0.1
    // 翻转符文平台的伺服的最低角度，也是收块的角度
    public static final double SERVO_FLIP_READY_POSITION = 0.2;//0.1
    // 翻转符文平台的伺服的最高角度
    public static final double SERVO_FLIP_UP_POSITION = 0.9;//0.1
    // 这个值似乎没什么用了。
    public static final double SERVO_FLIP_TIME = 2000;

    // 控制拨打宝珠的长杆的伺服，初始化状态的位置
    public static final double SERVO_SENSOR_ARM_INIT = 0.9;
    public static final double SERVO_SENSOR_ARM_UP_POSITION = 1;//0.1
    // 控制拨打宝珠的长杆的伺服，旋转到拨打状态时的角度。
    public static final double SERVO_SENSOR_ARM_DOWNING_POSITION = 0.4;//0.75
    // 控制拨打宝珠的长杆的伺服，旋转到拨打状态时的角度。
    public static final double SERVO_SENSOR_ARM_DOWN_POSITION = 0.5;//0n.75
    // 长杆完成下降总时间，单位tick
    public static final double SERVO_GLYPH_PUSH_INIT = 0;//
    public static final double SERVO_GLYPH_PUSH_DOWN = 1;//

    // 拨打宝珠的伺服，上电时的位置
    public static final double SERVO_INIT_POS = 1;
    public static final double SERVO_JEWEL_KICK_INIT_POSITION = 0.5; //0.15
    // 拨打宝珠的伺服，预备拨打时的角度
    public static final double SERVO_JEWEL_KICK_READY_POSITION = 0.5; //0.15
    // 拨打宝珠的伺服，拨打左侧球时达到的最大角度
    public static final double SERVO_JEWEL_KICK_LEFT_POSITION = 0.6;//0.0
    // 拨打宝珠的伺服，拨打右侧球时达到的最大角度
    public static final double SERVO_JEWEL_KICK_RIGHT_POSITION = 0.4;//0.3

    public static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    public static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    public static final double     P_DRIVE_COEFF           = 0.1;     // Larger is more responsive, but also less stable
    public static final double     ANGLE_ROBOT_CYBERBOX           = 24;     // Larger is more responsive, but also less stable


    public static final String COLOR_OF_RED_ALLIANCE          = "r";
    public static final String COLOR_OF_BLUE_ALLIANCE         = "b";

}
