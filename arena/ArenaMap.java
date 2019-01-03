package org.firstinspires.ftc.teamcode8515.arena;

import org.firstinspires.ftc.teamcode8515.hardware.C;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * 以左下红点为(0,0)，每英寸加1
 * 则4个点参考坐标为：
 * B1密码图：(144, 128.5) 	平衡板中心坐标：(120,120)   安全区头部坐标：(120,84)
 * B2密码图：(144, 56.5) 	平衡板中心坐标：(120,48)    安全区头部坐标：(108,48)
 * R1密码图：(0, 111.5) 		平衡板中心坐标：(24,120)    安全区头部坐标：(24,84)
 * R2密码图：(0, 39.5) 		平衡板中心坐标：(24,48)     安全区头部坐标：(36,48)
 */

public class ArenaMap {
    double[] b1Code = {144, 128.5};
    double[] b2Code = {144, 56.5};
    double[] r1Code = {0, 111.5};
    double[] r2Code = {0, 39.5};

    double[] b1BalanceCenter = {120,120};
    double[] b2BalanceCenter = {120,48};
    double[] r1BalanceCenter = {24,120};
    double[] r2BalanceCenter = {24,48};

    double[] b1SafeZone = {120,84};
    double[] b2SafeZone = {108,48};
    double[] r1SafeZone = {24,84};
    double[] r2SafeZone = {36,48};


    public Map<String, ArenaKeyPoints> arenaPoints = new HashMap<String, ArenaKeyPoints>(){
        {
            put("b1",new ArenaKeyPoints(b1Code, b1BalanceCenter, b1SafeZone));
            put("b2",new ArenaKeyPoints(b2Code, b2BalanceCenter, b2SafeZone));
            put("r1",new ArenaKeyPoints(r1Code, r1BalanceCenter, r1SafeZone));
            put("r2",new ArenaKeyPoints(r2Code, r2BalanceCenter, r2SafeZone));
        }
    };

    //按旋转中心到轮子的距离为半径，旋转90度的距离。
    double robotTurnDistance = 7.5 * Math.PI/2;
    double turnAngle = 78;

    public Map<String, ArrayList<CruiseSegment>> cruisePoints = new HashMap<String, ArrayList<CruiseSegment>>(){
        {
            //靠近遗骸保护区的蓝色平衡板开始
            put("b1",new ArrayList<CruiseSegment>(){{
                //后退到三角区顶点
                add(new CruiseSegment(C.DRIVE_SPEED_AUTO, -7,-7, C.SPEED_MS_PER_INCH *36));
                add(new CruiseSegment(C.DRIVE_SPEED_AUTO, -10,-10, C.SPEED_MS_PER_INCH *36));
                //把distance改为-14
                //左转，尾部对密码箱方向，停在三角区顶点
                add(new CruiseSegment(C.DRIVE_TURN_SPEED_AUTO, turnAngle, C.SPEED_MS_PER_INCH *robotTurnDistance));
            }});
            //远离遗骸保护区的蓝色平衡板开始
            put("b2",new ArrayList<CruiseSegment>(){{
                add(new CruiseSegment(C.DRIVE_SPEED_AUTO-0.1, -8,-8, C.SPEED_MS_PER_INCH *16));
                add(new CruiseSegment(C.DRIVE_SPEED_AUTO, -8,-8, C.SPEED_MS_PER_INCH *10));
                //左转
                add(new CruiseSegment(C.DRIVE_TURN_SPEED_AUTO, -turnAngle, C.SPEED_MS_PER_INCH *robotTurnDistance));
                //前进到三角区顶点
                add(new CruiseSegment(C.DRIVE_SPEED_AUTO, -12,-12, C.SPEED_MS_PER_INCH *18));
                //右转，尾部对密码箱方向，停在三角区顶点
                add(new CruiseSegment(C.DRIVE_TURN_SPEED_AUTO, turnAngle, C.SPEED_MS_PER_INCH *robotTurnDistance));
            }});
            //靠近遗骸保护区的红色平衡板开始
            put("r1",new ArrayList<CruiseSegment>(){{
                //前进到三角区顶点
                add(new CruiseSegment(C.DRIVE_SPEED_AUTO, 7,7, C.SPEED_MS_PER_INCH *36));
                add(new CruiseSegment(C.DRIVE_SPEED_AUTO, 10,10, C.SPEED_MS_PER_INCH *36));
                //把distance改为14
                //左转，尾部对密码箱方向，停在三角区顶点
                add(new CruiseSegment(C.DRIVE_TURN_SPEED_AUTO, turnAngle, C.SPEED_MS_PER_INCH *robotTurnDistance));
            }});
            //远离遗骸保护区的红色平衡板开始
            put("r2",new ArrayList<CruiseSegment>(){{
                //前进
                add(new CruiseSegment(C.DRIVE_SPEED_AUTO-0.1, 8,8, C.SPEED_MS_PER_INCH *16));
                add(new CruiseSegment(C.DRIVE_SPEED_AUTO, 8,8, C.SPEED_MS_PER_INCH *10));
                //左转
                add(new CruiseSegment(C.DRIVE_TURN_SPEED_AUTO, turnAngle, C.SPEED_MS_PER_INCH *robotTurnDistance));
                //前进到三角区顶点
                add(new CruiseSegment(C.DRIVE_SPEED_AUTO, 12,12, C.SPEED_MS_PER_INCH *18));
                //左转，尾部对密码箱方向，停在三角区顶点
                add(new CruiseSegment(C.DRIVE_TURN_SPEED_AUTO, turnAngle, C.SPEED_MS_PER_INCH *robotTurnDistance));
            }});
        }
    };

}
