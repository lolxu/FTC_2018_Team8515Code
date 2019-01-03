package org.firstinspires.ftc.teamcode8515.hardware.subModule;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode8515.utils.FtcLog;


/**
 * Created by zhuxu on 2018/4/21.
 */
public class VisionDetector {
    //public static final String
    public String vuforiaLicenseKey = "AUIw3yT/////AAAAGcndj86d9kgpizskvOOdjEIBCCSPjGvt3PKJlbt37pRD6cSvy0Kc7Q3wMotLfLCOOc7R6Kxnmq9mk4bbJh8yLE/oFTamvqb1m53hdkQMjvW71a6Z7kw1/iWPE2/6p3/Wex3iEo8sQWrnngPf5RbLjFCNKMvMCqav5/9czxnSnsNdYY4KaAW0Z7QE2ymH1pW4r2AXPtdbB2hWnFNiO/NwBVo1QCBgwuQA92i2v74uE0XFHZN/FQDT3U7bjiG21vUoPPnZ0Ro8ivtT42NJkbt/JMqk3PFs9K/L1v9lfWCF+OPPOV6j+mXYbXUg1LUOfBnMGkCBaMw5MNCPDyFgvusujM+s+10UitJR3stxPoLVoLjx";

    private VuforiaTrackables relicTrackables = null;
    private VuforiaTrackable relicTemplate = null;
    private RelicRecoveryVuMark vuMark = null;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    double angleOfCode = 0;
    String vumarkCode = "unknown";
    LinearOpMode op = null;
    VectorF trans = null;
    Orientation rot = null;

    AxesOrder order = AxesOrder.XYZ;
    AxesReference ref = AxesReference.EXTRINSIC;
    AngleUnit unit = AngleUnit.DEGREES;

    public VisionDetector(){}

    public VisionDetector(LinearOpMode op){
        this.op = op;
    }

    public void init(){
        //初始化vuforia库
        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.op.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = vuforiaLicenseKey;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    public void start(){
        //relicTrackables.activate();
    }

    public void stop(){
        relicTrackables.deactivate();
        CameraDevice.getInstance().deinit();
    }

    public String detectCode(){
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        ElapsedTime time = new ElapsedTime();
        time.reset();
        vumarkCode = "unknown";

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            vumarkCode = vuMark.toString();
            //robot.setCode(vumarkCode);
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            //telemetry.addData("Pose", format(pose));

            /* We further illustrate how to decompose the pose into useful rotational and
             * translational components */
            if (pose != null) {
                trans = pose.getTranslation();
                rot = Orientation.getOrientation(pose, ref, order, unit);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                // 初始化与密码的相对角度。其中一个角度就是
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;

                //此值，用于确定机器人摆放位置，与场地边缘的角度，从而在自动驾驶阶段可以进行矫正。
                //选择的角度，一般应是从场内观察密码图时，指向左侧方向的角度。
                angleOfCode = rY;
            }
        }
        return vumarkCode;
    }
    /**
     *
     * 当手机竖直向上，背部向外，放置在机器人右侧时
     * 通常取secondAngle的值作为机器人与密码图之间的夹角。
     * 该值越大，机器人头部越偏右，即头部靠近密码图，尾部远离密码图。
     * */
    public boolean detectAngle(double[] angle){
        boolean isFound = false;
        String vuMarkCode = "not visible";
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            /* We further illustrate how to decompose the pose into useful rotational and
             * translational components */
            vuMarkCode = vuMark.toString();
            if (pose != null) {
                trans = pose.getTranslation();
                rot = Orientation.getOrientation(pose, ref, order, unit);

                // Extract the rotational components of the target relative to the robot
                //此值，用于确定机器人摆放位置，与场地边缘的角度，从而在自动驾驶阶段可以进行矫正。
                //选择的角度，一般应是从场内观察密码图时，指向左侧方向的角度。
                angle[0] = rot.firstAngle;
                angle[1] = rot.secondAngle;
                angle[2] = rot.thirdAngle;
            }
            isFound = true;
        }
        FtcLog.i("VuMark", "--actionDetectCode:" + vuMarkCode,true,true);
        return isFound;
    }

    public double getAngle(){
        double [] angle = new double[3];
        detectAngle(angle);
        return angle[1];//xyz,侧面取0，垂直取1，平躺取2
    }

    /**
     *
     * */
    public boolean detectPosition(double[] angle){
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        ElapsedTime time = new ElapsedTime();

        time.reset();
        boolean isFound = false;
        while (op.opModeIsActive()
                && time.milliseconds()<50){
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    trans = pose.getTranslation();
                    rot = Orientation.getOrientation(pose, ref, order, unit);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    angle[0]= trans.get(0);
                    angle[1]= trans.get(1);
                    angle[2]= trans.get(2);

                }
                isFound = true;
                break;
                //FtcLog.i(TAG,"--actionDetectCode:" + vumarkCode);
            }
            else {
                //FtcLog.i("VuMark", "not visible",true,true);
            }
        }
        return isFound;
    }

    private void finish() {
        relicTrackables.deactivate();
        CameraDevice.getInstance().deinit();
    }

}
