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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode8515.hardware.HardwareBotGulosity;
import org.firstinspires.ftc.teamcode8515.utils.FtcLog;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test: Manually", group="Pushbot")
//@Disabled
public class PushbotTeleopTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareBotGulosity robot           = null;   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        FtcLog.opMode = this;
        robot           = new HardwareBotGulosity(this);

        robot.init();

        // Send telemetry message to signify robot waiting;
        FtcLog.i("Say", "I'm hungry, Feed Me!",true,true);    //

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        FtcLog.i("Say", "I'm on the way!",true,true);    //

        while (opModeIsActive()) {

            //1号 左摇杆：全向平移
            //1号 右摇杆：左右方向，转向
            //1号 y 调节动力为高速，x 调节动力为低速
            //1号 左右bumper：right_bumper left_bumper 控制，按下吸块，放开停止转动

            //2号 a 平台设置为水平状态
            //2号 右摇杆上下方向：平台翻转，放开摇杆平台回复水平
            //2号 左摇杆上下方向：平台抬升

            /**===============gamepad1================= **/
            // 调节动力比例，切换行驶速度
            if(gamepad1.y){
                robot.drive.setSpeedScaleHigh();
            }
            // 调节动力比例，切换行驶速度
            if(gamepad1.x){
                robot.drive.setSpeedScaleSlow();
            }

            // 根据左stick的x、y值决定位移量、右stick的x值决定旋转量，来驱动底盘
            if(Math.abs(gamepad1.right_stick_x)>0.1
                    || Math.abs(gamepad1.left_stick_y)>0.1
                    || Math.abs(gamepad1.left_stick_x)>0.1 ){
                robot.drive.driveByStick(gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x);
            }
            else{
                robot.drive.driveStop();
            }

        }
    }
}
