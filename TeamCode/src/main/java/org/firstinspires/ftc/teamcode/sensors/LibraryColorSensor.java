package org.firstinspires.ftc.teamcode.sensors;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareBeep;

/*
 * This is an example LinearOpMode that shows how to use a color sensor in a generic
 * way, insensitive which particular make or model of color sensor is used. The opmode
 * assumes that the color sensor is configured with a name of "sensor_color".
 *
 * If the color sensor has a light which is controllable, you can use the X button on
 * the gamepad to toggle the light on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
public class LibraryColorSensor {

  /**
   * The colorSensor field will contain a reference to our color sensor hardware object
   */
  NormalizedColorSensor colorSensor;
  HardwareBeep robot = null;
  /**
   * The relativeLayout field is used to aid in providing interesting visual feedback
   * in this sample application; you probably *don't* need something analogous when you
   * use a color sensor on your robot
   */
  View relativeLayout;
  ElapsedTime timer;

  public float readSaturation(HardwareBeep myRobot, String myCS) {
    robot = myRobot;
    ElapsedTime timer = new ElapsedTime();
    timer.reset();
    // values is a reference to the hsvValues array.
    float[] hsvValues = new float[3];
    final float values[] = hsvValues;

    // bPrevState and bCurrState keep track of the previous and current state of the button
    boolean bPrevState = false;
    boolean bCurrState = false;
    String currColor = "";


    colorSensor = myRobot.hwMap.get(NormalizedColorSensor.class, myCS);


    // If possible, turn the light on in the beginning (it might already be on anyway,
    // we just make sure it is if we can).
    if (colorSensor instanceof SwitchableLight) {
      ((SwitchableLight) colorSensor).enableLight(false);
    }

    // Read the sensor
    NormalizedRGBA colors = colorSensor.getNormalizedColors();
    Color.colorToHSV(colors.toColor(), hsvValues);

    return hsvValues[1];

  }
}

