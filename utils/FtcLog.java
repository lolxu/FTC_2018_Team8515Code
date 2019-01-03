package org.firstinspires.ftc.teamcode8515.utils;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by zhuxu on 2018/4/6.
 */

public class FtcLog {

    public static LinearOpMode opMode = null;

    public static void i(String title, String content){
        i(title,content,true,false);
    }

    public static void i(String title, String content,boolean isTelemetry){
        i(title,content,isTelemetry,false);
    }

    public static void i(String title, String content,boolean isTelemetry,boolean isTelemetryUpdate){
        Log.i("Ftc",title+"="+content);
        if(isTelemetry && opMode!=null){
            opMode.telemetry.addData(title,content);
            if(isTelemetryUpdate){
                opMode.telemetry.update();
            }
        }
    }

}
