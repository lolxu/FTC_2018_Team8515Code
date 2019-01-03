package org.firstinspires.ftc.teamcode8515.arena;

/**
 * Created by zhuxu on 2018/4/21.
 */

public class ArenaPoint {
    private double x;
    private double y;

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public ArenaPoint(){

    }

    public ArenaPoint(double x,double y){
        this.x = x;
        this.y = y;
    }
}
