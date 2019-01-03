package org.firstinspires.ftc.teamcode8515.arena;

/**
 * Created by zhuxu on 2018/4/6.
 */

public class CruiseSegment {
    public double[] pointValue = new double[4];
    public int type = 0;

    public CruiseSegment(){    }

    /**
     * 按行进距离进行初始化
     * @param robotSpeed 行驶时的给电机的驱动力
     * @param leftDistance 按全向模式时，行驶方向，按机器人正方向偏移度测算，向该方向平移。按常规模式时，记录左轮行驶距离
     * @param rightDistance 按全向模式时，记录yawSpeed 转向驱动力；按常规模式，记录右轮行驶距离
     * @param timeInMs 运行时间
     * */
    public CruiseSegment(int type, double robotSpeed, double leftDistance, double rightDistance, double timeInMs ){
        type = 0;
        pointValue[0] = robotSpeed;
        pointValue[1] = leftDistance;
        pointValue[2] = rightDistance;
        pointValue[3] = timeInMs;

    }
    /**
     * 按行进距离进行初始化
     * @param robotSpeed 行驶时的给电机的驱动力
     * @param leftDistance 按全向模式时，行驶方向，按机器人正方向偏移度测算，向该方向平移。按常规模式时，记录左轮行驶距离
     * @param rightDistance 按全向模式时，记录yawSpeed 转向驱动力；按常规模式，记录右轮行驶距离
     * @param timeInMs 运行时间
     * */
    public CruiseSegment(double robotSpeed, double leftDistance, double rightDistance, double timeInMs ){
        type = 0;
        pointValue[0] = robotSpeed;
        pointValue[1] = leftDistance;
        pointValue[2] = rightDistance;
        pointValue[3] = timeInMs;

    }

    /**
     * 按转向角度进行初始化
     * @param robotSpeed 行驶时的给电机的驱动力
     * @param angleOffset 角度偏移量。即旋转多少度。
     * @param timeInMs 运行时间
     * */
    public CruiseSegment( double robotSpeed, double angleOffset, double timeInMs ){
        type = 1;
        pointValue[0] = robotSpeed;
        pointValue[1] = angleOffset;
        pointValue[3] = timeInMs;
    }

    public int getType(){return type;}
    public double getSpeed(){
        return pointValue[0];
    }
    public double getValue1(){
        return pointValue[1];
    }
    public double getValue2(){
        return pointValue[2];
    }
    public double getAngle(){return pointValue[1];}
    public double getLeftSpeed(){return pointValue[1];}
    public double getRightSpeed(){return pointValue[2];}

    public double getTime(){
        return pointValue[3];
    }
}
