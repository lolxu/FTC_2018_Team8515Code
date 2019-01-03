package org.firstinspires.ftc.teamcode8515.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode8515.hardware.C;

/**
 * Created by zhuxu on 2017/12/4.
 */

@Autonomous(name="Gulosity: Blue 1", group ="Auto")
//@Disabled
public class FtcPhaseAutoGulosityBlue1 extends FtcPhaseAutoGulosity {

    public FtcPhaseAutoGulosityBlue1(){
        startColor = C.COLOR_OF_BLUE_ALLIANCE;
        startPoint = 1;
    }

}
