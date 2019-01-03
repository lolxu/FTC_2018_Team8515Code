package org.firstinspires.ftc.teamcode8515.utils;

/**
 * Created by zhuxu on 2017/11/21.
 */

public class ColorHelper {

    /**
     * test if a hsv value in range of red
     * */
    public static boolean isRed(double hsvValue){
        return hsvValue<30 || hsvValue>330;
    }

    /**
     * test if a hsv value in range of blue
     * */
    public static boolean isBlue(double hsvValue){
        return hsvValue>=180 && hsvValue<240;
    }
}
