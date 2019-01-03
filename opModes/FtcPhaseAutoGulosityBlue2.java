package org.firstinspires.ftc.teamcode8515.opModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode8515.hardware.C;

/**
 * Created by zhuxu on 2017/12/4.
 */

@Autonomous(name="Gulosity: Blue 2", group ="Auto")
//@Disabled
public class FtcPhaseAutoGulosityBlue2 extends FtcPhaseAutoGulosity {

    public FtcPhaseAutoGulosityBlue2(){
        startColor = C.COLOR_OF_BLUE_ALLIANCE;
        startPoint = 2;
    }

}
