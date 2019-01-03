package org.firstinspires.ftc.teamcode8515.hardware.device;

import com.qualcomm.robotcore.hardware.Servo;


public class ServoWrapper {

    private String name ="";
    private Servo servo;
    private double physicalMin = 0;
    private double physicalMax = 1;
    private double logicMin;
    private double logicMax;
    private double logicMid;
    private double logicInit;
    private double speedInMilsec;
    private double target;

    public ServoWrapper(Servo servo,
                        double min,
                        double max,
                        double init,
                        double speedInMilsec
    ){
        this.setServo(servo);
        this.setLogicMin(min);
        this.setLogicMax(max);
        this.setLogicMid((max - min)/2);
        this.setLogicInit(init);
        this.setSpeedInMilsec(speedInMilsec);
        this.servo.setPosition(init);

    }

    public ServoWrapper(Servo servo,
                        double min,
                        double max,
                        double init,
                        Servo.Direction direction,
                        double speedInMilsec
    ){
        this.setServo(servo);
        this.setLogicMin(min);
        this.setLogicMax(max);
        this.setLogicMid((max - min)/2);
        this.setLogicInit(init);
        this.getServo().setDirection(direction);
        this.setSpeedInMilsec(speedInMilsec);
        this.servo.setPosition(init);
    }

    /**
     * 按指定速度向设置的最大方向转动
     * @param timeInMilsec 旋转的时间间隔，本次设置与上次设置间隔的时间，单位毫秒
     * */
    public boolean turnToMax(double timeInMilsec){
        return turnToPosition(getLogicMax(),timeInMilsec);
    }

    /**
     *
     * @param timeInMilsec 旋转的时间间隔，本次设置与上次设置间隔的毫秒数
     * */
    public boolean turnToMin(double timeInMilsec){
        return turnToPosition(getLogicMin(),timeInMilsec);
    }

    /**
     *
     * @param timeInMilsec 旋转的时间间隔，本次设置与上次设置间隔的毫秒数
     * */
    public boolean turnToMid(double timeInMilsec){
        return turnToPosition(getLogicMid(),timeInMilsec);
    }


    /**
     * @param timeInMilsec 旋转的时间间隔，本次设置与上次设置间隔的毫秒数
     * */
    public boolean turnToPosition(double targetPosition,
                                  double timeInMilsec){
        double pos = getServo().getPosition();
        if(Math.abs(pos - targetPosition)<0.02){
            getServo().setPosition(targetPosition);
            return true;
        }
        double offset = timeInMilsec/ getSpeedInMilsec();
        int direct = (int)Math.round((targetPosition - pos)/Math.abs(targetPosition - pos));
        pos += offset * direct;
        if(direct * (pos - targetPosition)>0){
            getServo().setPosition(targetPosition);
            return true;
        }
        getServo().setPosition(pos);
        return false;
    }


    /**
     * @param timeInMilsec 旋转的时间间隔，本次设置与上次设置间隔的毫秒数
     * */
    public boolean turnToTarget(double timeInMilsec){
        return turnToPosition(target,timeInMilsec);
    }



    public double getPosition(){
        return this.getServo().getPosition();
    }

    public void setPosition(double position){
        this.getServo().setPosition(position);
    }


    public Servo getServo() {
        return servo;
    }

    public void setServo(Servo servo) {
        this.servo = servo;
    }

    public double getLogicMin() {
        return logicMin;
    }

    public void setLogicMin(double logicMin) {
        this.logicMin = logicMin;
    }

    public double getLogicMax() {
        return logicMax;
    }

    public void setLogicMax(double logicMax) {
        this.logicMax = logicMax;
    }

    public double getLogicMid() {
        return logicMid;
    }

    public void setLogicMid(double logicMid) {
        this.logicMid = logicMid;
    }

    public double getLogicInit() {
        return logicInit;
    }

    public void setLogicInit(double logicInit) {
        this.logicInit = logicInit;
    }

    public double getSpeedInMilsec() {
        return speedInMilsec;
    }

    public void setSpeedInMilsec(double speedInMilsec) {
        this.speedInMilsec = speedInMilsec;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double getPhysicalMin() {
        return physicalMin;
    }

    public void setPhysicalMin(double physicalMin) {
        this.physicalMin = physicalMin;
    }

    public double getPhysicalMax() {
        return physicalMax;
    }

    public void setPhysicalMax(double physicalMax) {
        this.physicalMax = physicalMax;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }
}
