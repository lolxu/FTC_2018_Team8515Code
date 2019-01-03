package org.firstinspires.ftc.teamcode8515.arena;

public class ArenaKeyPoints {

    public ArenaPoint codePoint;
    public ArenaPoint balanceCenterPoint;
    public ArenaPoint safeZonePoint;

    public ArenaKeyPoints(double[] codePoint,double[] balanceCenterPoint,double[] safeZonePoint){
        this.codePoint = new ArenaPoint(codePoint[0],codePoint[1]);
        this.balanceCenterPoint = new ArenaPoint(balanceCenterPoint[0],balanceCenterPoint[1]);
        this.safeZonePoint = new ArenaPoint(safeZonePoint[0],safeZonePoint[1]);
    }

}
