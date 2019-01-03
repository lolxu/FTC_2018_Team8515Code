/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode8515.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode8515.arena.ArenaKeyPoints;
import org.firstinspires.ftc.teamcode8515.arena.ArenaMap;
import org.firstinspires.ftc.teamcode8515.arena.CruiseSegment;
import org.firstinspires.ftc.teamcode8515.hardware.C;
import org.firstinspires.ftc.teamcode8515.hardware.HardwareBotGulosity;
import org.firstinspires.ftc.teamcode8515.utils.FtcLog;

import java.util.ArrayList;


/**
 * This 2016-2017 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The two vision target are located on the two walls closest to the audience, facing in.
 * The Stones are on the RED side of the field, and the Chips are on the Blue side.
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 *
 * 8515
 */

@Autonomous(name="Gulosity: Stable Auto phase", group ="Auto")
@Disabled
public class FtcPhaseAutoGulosity extends LinearOpMode {

    static final String TAG = "FtcPhaseAutoGulosity";

    /**
     * 击打宝珠的机构不变
     * 1个电机控制装载密码的平台的升降；可能需要使用编码器
     * 2个电机旋转轮子，吞吐密码
     * 1个伺服翻转平台，将密码送入密码箱
     * 1个颜色传感器用于识别球，尾部一个颜色传感器用于做密码箱定位
     * todo:基于现在这个全向轮的特点，在自动阶段，可以考虑直接用手机摄像头通过识别密码图的位置来做粗略定位。
     * 方法是做完拨球动作后，旋转机器人，让手机摄像头对着密码图，
     * 然后控制机器人走向密码箱，过程中实时通过vuforia Markid的识别功能确定密码图到机器人坐标系，以此调整位置。
     * 行进到密码箱三角区后，用后置的颜色传感器来探测详细位置。
     **/

    /**for debug**************************************************************/
    //1. 打开初始化vuforia探测 UNKNOWN, LEFT, CENTER, RIGHT

    //2. 将机械臂转下，期间检测颜色传感器，以确定所对角度为蓝色还是红色，然后恢复姿态

    //3. 将车子带着 符文 驶向密码箱

    //4. 检测密码箱位置，驶向合适的角度，将符文推入密码箱，释放符文

    //5. 调整位置，停靠在安全区


    /*************************************************************************/
    //联盟颜色常量：1：红色；2：蓝色；

    /*************************************************************************/
    //起点
    public  int startPoint        = 1;
    //当前自己所属联盟的颜色；默认红色
    public  String startColor        = "r";
    //识别出的密码项，取值在left，center，right，unknown
    public String vumarkCode="unknown";

    ArenaMap arenaMap ;
    ArenaKeyPoints keyPoints;
    ArrayList<CruiseSegment> curisePoints;

    /*************************************************************************/
    HardwareBotGulosity robot ;   // Use a Pushbot's hardware

    /*************************************************************************/
    //当前总执行时间
    public ElapsedTime timeInterval   = new ElapsedTime();

    @Override public void runOpMode() {
        FtcLog.opMode = this;
        FtcLog.i("0","0000000000000000",true,true);

        robot = new HardwareBotGulosity(this);
        robot.startColor = startColor;
        robot.startPoint = startPoint;
        FtcLog.i("0","1",true,true);

        robot.init();
        FtcLog.i("0","2",true,true);
        robot.initForAuto();
        FtcLog.i("0","3",true,true);

        arenaMap = new ArenaMap();
        keyPoints = arenaMap.arenaPoints.get(startColor+startPoint);
        curisePoints = arenaMap.cruisePoints.get(startColor+startPoint);
        actionCodeDetectInit();
        /** Wait for the game to begin */
        FtcLog.i(">", "Press Play to start",true,true);

        waitForStart();
        robot.setServoJewelKickPosition(C.SERVO_JEWEL_KICK_INIT_POSITION);
        robot.setServoSensorArmPosition(C.SERVO_SENSOR_ARM_UP_POSITION);

        /** Auto phase start ------------------------------ */
        FtcLog.i("1","111111111111111",true,true);
        //1. 打开初始化vuforia探测 UNKNOWN, LEFT, CENTER, RIGHT
        robot.start();
        FtcLog.i("2","222222222",true,true);
        //robot.vision.init();
        //sleep(2000);
        actionCodeDetecting();
        actionCodeDetectFinish();

//        timeInterval.reset();
//        while (opModeIsActive() && timeInterval.milliseconds()<5000 && vumarkCode.equals("unknown")) {
//            vumarkCode = robot.vision.detectCode();
//            double[] angles = new double[3];
//            robot.vision.detectAngle(angles);
//            FtcLog.i("vumarkCode", vumarkCode + angles[0] + "==" + angles[1] + "==" + angles[2]
//                    ,true,true);
//        }
//        //sleep(2000);
//        robot.setCode(vumarkCode);

        //2. 将机械臂转下，期间检测颜色传感器，以确定所对角度为蓝色还是红色，然后恢复姿态
        //支撑杆前进到中间位置
        FtcLog.i("kick：", "-- starting --"+vumarkCode,true,true);
        double lastPush = 0;
        timeInterval.reset();
        //控制缓慢下落
        while (opModeIsActive() && timeInterval.milliseconds()<4000){
            double ticks = timeInterval.milliseconds() - lastPush;
            if(ticks < 25){
                idle();
                continue;
            }
            lastPush = timeInterval.milliseconds();
            if(robot.servoSensorArmDown(ticks)){
                break;
            }
        }

        FtcLog.i("kick：", "-- kicking --"+vumarkCode,true,true);
        //开始拨打，拨打时让击打杆运转到底，则可以在支撑杆收回之后成水平状态。
        timeInterval.reset();
        robot.autoJewelKickStart(timeInterval.milliseconds());
        while (opModeIsActive() && timeInterval.milliseconds()<500){
            idle();
        }
        robot.returnJewel();

        FtcLog.i("kick：", "-- backing --"+vumarkCode,true,true);
        //支撑杆收回到初始竖直位置
        timeInterval.reset();
        lastPush = 0;
        while (opModeIsActive() && timeInterval.milliseconds()<1500){
            double ticks = timeInterval.milliseconds() - lastPush;
            if(ticks < 25){
                idle();
                continue;
            }
            lastPush = timeInterval.milliseconds();
            if(robot.servoSensorArmUp(ticks)){
                break;
            }
        }

        FtcLog.i("navigate：", "-- starting --",true,true);
        //3. 将车子带着 符文 驶向密码箱
        // 如无误差，将行驶到密码箱前三角区顶点位置，并以后部对着密码箱。
        for (CruiseSegment s : curisePoints){
            FtcLog.i("navigate:","type="+s.getType()+" value="+s.getValue1()+" time="+s.getTime(),
                    true,true);
            if(s.getType()==1){
                //方向不同，需要转弯
                robot.navigator.driveTurn(s.getSpeed(), s.getAngle());
            }else{
                //
                robot.navigator.driveDirect(
                        s.getSpeed(),
                        s.getLeftSpeed(),
                        0,
                        s.getTime());
            }
            //sleep(1000);
        }

        FtcLog.i("safe zone：", "-- starting --",true,true);
        //4. 检测密码箱位置，旋转合适的角度后，驶向密码箱并将，将符文推入密码箱
        // 转角定向，可以调整turnDistance来让机器人旋转到合适的角度
        // 当前代码旋转20度，如需调整，可将20改为需要的角度。（90度需要行进的距离。除以90乘以20）
        if(!vumarkCode.toLowerCase().equals("unknown")){
            double angle = 0;
            if(vumarkCode.toLowerCase().equals("right")){
                angle = -C.ANGLE_ROBOT_CYBERBOX;
            }
            if(vumarkCode.toLowerCase().equals("left")){
                angle = C.ANGLE_ROBOT_CYBERBOX-3;
            }
            robot.navigator.gyroCalibration();
            robot.navigator.driveTurn(C.DRIVE_SPEED_AUTO,angle);
        }
        //后退，让后部靠着密码箱
        robot.navigator.gyroCalibration();
        robot.navigator.driveDirect(C.DRIVE_SPEED_AUTO,-12,0,800);


        //送方块
        timeInterval.reset();
        //robot.flipUp();
        double lastFlip =0;
        while (opModeIsActive() && timeInterval.milliseconds()< 2500){
            double ticks = timeInterval.milliseconds() - lastFlip;
            if(ticks<50){
                idle();
                continue;
            }
            if(robot.flipUp(ticks))
                break;
            lastFlip = timeInterval.milliseconds();
        }
//        //送完恢复水平状态
//        robot.flipHor();
//        while (opModeIsActive() && timeInterval.milliseconds()< 50){
//            idle();
//        }
        //再拍一下
//        robot.flipUp();
//        while (opModeIsActive() && timeInterval.milliseconds()< 1000){
//            idle();
//        }

        robot.navigator.gyroCalibration();
        robot.navigator.driveDirect(C.DRIVE_SPEED_AUTO+0.1,16,0,1000);
        while (opModeIsActive() && timeInterval.milliseconds()< 1000){
            idle();
        }


        robot.navigator.gyroCalibration();
        robot.navigator.driveDirect(C.DRIVE_SPEED_AUTO+0.4,-20,0,1400);
        //恢复到收块位置
        robot.flipDown();
//        while (opModeIsActive() && timeInterval.milliseconds()< 1000){
//            idle();
//        }
//        while (opModeIsActive() && timeInterval.milliseconds()< 500){
//            idle();
//        }
        robot.navigator.driveDirect(C.DRIVE_SPEED_AUTO,14,0,1100);
        robot.drive.driveStop();
        robot.navigator.stop();

    }

    public String vuforiaLicenseKey = "AUIw3yT/////AAAAGcndj86d9kgpizskvOOdjEIBCCSPjGvt3PKJlbt37pRD6cSvy0Kc7Q3wMotLfLCOOc7R6Kxnmq9mk4bbJh8yLE/oFTamvqb1m53hdkQMjvW71a6Z7kw1/iWPE2/6p3/Wex3iEo8sQWrnngPf5RbLjFCNKMvMCqav5/9czxnSnsNdYY4KaAW0Z7QE2ymH1pW4r2AXPtdbB2hWnFNiO/NwBVo1QCBgwuQA92i2v74uE0XFHZN/FQDT3U7bjiG21vUoPPnZ0Ro8ivtT42NJkbt/JMqk3PFs9K/L1v9lfWCF+OPPOV6j+mXYbXUg1LUOfBnMGkCBaMw5MNCPDyFgvusujM+s+10UitJR3stxPoLVoLjx";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;


    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    public void actionCodeDetectInit(){
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AUIw3yT/////AAAAGcndj86d9kgpizskvOOdjEIBCCSPjGvt3PKJlbt37pRD6cSvy0Kc7Q3wMotLfLCOOc7R6Kxnmq9mk4bbJh8yLE/oFTamvqb1m53hdkQMjvW71a6Z7kw1/iWPE2/6p3/Wex3iEo8sQWrnngPf5RbLjFCNKMvMCqav5/9czxnSnsNdYY4KaAW0Z7QE2ymH1pW4r2AXPtdbB2hWnFNiO/NwBVo1QCBgwuQA92i2v74uE0XFHZN/FQDT3U7bjiG21vUoPPnZ0Ro8ivtT42NJkbt/JMqk3PFs9K/L1v9lfWCF+OPPOV6j+mXYbXUg1LUOfBnMGkCBaMw5MNCPDyFgvusujM+s+10UitJR3stxPoLVoLjx";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

    }

    public void actionCodeDetecting(){

        relicTrackables.activate();
        ElapsedTime time = new ElapsedTime();
        time.reset();
        int tryCount = 0;
        while(opModeIsActive() && tryCount<4){

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                vumarkCode = vuMark.toString();
                robot.setCode(vumarkCode);
                FtcLog.i(TAG,"--actionDetectCode:" + vumarkCode);
                break;
            }
            else {
                FtcLog.i("VuMark", "not visible",true,true);
            }
            tryCount++;
            sleep(500);

        }

    }

    private void actionCodeDetectFinish() {
//        relicTrackables.deactivate();
//        CameraDevice.getInstance().deinit();

    }


}
