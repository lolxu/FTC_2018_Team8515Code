package org.firstinspires.ftc.teamcode8515.hardware.device;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode8515.utils.ColorHelper;

/**
 * Created by zhuxu on 2018/4/21.
 */

public class ColorSensorWrapper {
    // 颜色识别时，从0-1转换到0-255，以符合rgb的需要
    static final double COLOR_SCALE_FACTOR = 255;

    private ColorSensor sensorColor = null;

    public ColorSensorWrapper(){

    }

    public ColorSensorWrapper(ColorSensor sensorColor){
        this.sensorColor = sensorColor;
    }

    /**
     * 当前颜色传感器读取的值是不是蓝色
     */
    public boolean isColorBlue() {
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV((int) (sensorColor.red() * COLOR_SCALE_FACTOR),
                (int) (sensorColor.green() * COLOR_SCALE_FACTOR),
                (int) (sensorColor.blue() * COLOR_SCALE_FACTOR),
                hsvValues);
        //根据颜色决定转动角度，后续逻辑中根据角度来拨动宝珠
        //传感器指向左侧的情况下。hsvValues[0]中保存的是左侧宝珠的hsv颜色
        return ColorHelper.isBlue(hsvValues[0]);
    }

    /**
     * 当前颜色传感器读取的值是不是红色
     */
    public boolean isColorRed() {
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV((int) (sensorColor.red() * COLOR_SCALE_FACTOR),
                (int) (sensorColor.green() * COLOR_SCALE_FACTOR),
                (int) (sensorColor.blue() * COLOR_SCALE_FACTOR),
                hsvValues);
        //根据颜色决定转动角度，后续逻辑中根据角度来拨动宝珠
        //传感器指向左侧的情况下。hsvValues[0]中保存的是左侧宝珠的hsv颜色
        return ColorHelper.isRed(hsvValues[0]);
    }

}
