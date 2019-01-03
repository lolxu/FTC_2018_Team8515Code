package org.firstinspires.ftc.teamcode8515.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode8515.hardware.C;

/**
 * Created by zhuxu on 2017/12/4.
 */

@Autonomous(name="Gulosity: Red 2", group ="Auto")
//@Disabled
public class FtcPhaseAutoGulosityRed2 extends FtcPhaseAutoGulosity {

    public FtcPhaseAutoGulosityRed2(){
        startColor = C.COLOR_OF_RED_ALLIANCE;
        startPoint = 2;
    }

}
