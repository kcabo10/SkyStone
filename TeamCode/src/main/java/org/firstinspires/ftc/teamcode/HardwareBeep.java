//Declare package
package org.firstinspires.ftc.teamcode;

//Import Hardware

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.sensors.REVColorSensor;
import org.firstinspires.ftc.teamcode.sensors.LibraryColorSensor;
import org.firstinspires.ftc.teamcode.sensors.SensorMB1242;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our Hardware Map that contains all the motors, servos, and sensors that we use on the
 * robot. We pull this Hardware Map in all the programs we use a part of the robot. In this program
 * we intialize the encoders on the motors we want to call the encoder for.
 */
public class HardwareBeep {

    // Set Public OpMode Members
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public CRServo rightIntake = null;
    public CRServo leftIntake = null;
    public DcMotor droidLifterLeft = null;
    public DcMotor droidLifterRight = null;
    public DcMotor outExtrusion = null;
    public Servo claw = null;
    public Servo clawTurner = null;
    public Servo foundation1 = null;
    public Servo foundation2 = null;
    public Servo clawAid = null;
    public BNO055IMU imuActual = null;
    public SensorMB1242 rightSonic = null;
    public SensorMB1242 leftSonic = null;
    public NormalizedColorSensor colorSensor = null;
//    public WebcamName webcam = null;

    // Set local OpMode Members
    public HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Constructor
    public HardwareBeep() {
    }

    /**
     * Initializes standard hardware interfaces
     *
     * @param ahwMap A reference to Hardware Map
     */
    public void init(HardwareMap ahwMap) {

        // Telemetry Switches
        boolean GRID_NAV_TELEMETRY_ON = true;

        // Save Reference To Hardware Map
        hwMap = ahwMap;

        // Define Motors, Servos, and Sensors
        leftFront = hwMap.get(DcMotor.class, "left_front");
        leftBack = hwMap.get(DcMotor.class, "left_back");
        rightFront = hwMap.get(DcMotor.class, "right_front");
        rightBack = hwMap.get(DcMotor.class, "right_back");
        droidLifterLeft = hwMap.get(DcMotor.class, "droid_left");
        droidLifterRight = hwMap.get(DcMotor.class, "droid_right");
        outExtrusion = hwMap.get(DcMotor.class, "out_extrusion");
        leftIntake = hwMap.get(CRServo.class, "left_intake");
        rightIntake = hwMap.get(CRServo.class, "right_intake");
        claw = hwMap.get(Servo.class, "claw");
        clawTurner = hwMap.get(Servo.class, "claw_turner");
        foundation1 = hwMap.get(Servo.class, "foundation1");
        foundation2 = hwMap.get(Servo.class, "foundation2");
        clawAid = hwMap.get(Servo.class, "claw_aid");


        imuActual = hwMap.get(BNO055IMU.class, "imu_actual");
        leftSonic = hwMap.get(SensorMB1242.class, "left_sonic");
        rightSonic = hwMap.get(SensorMB1242.class, "right_sonic");
        colorSensor = hwMap.get(NormalizedColorSensor.class, "sensor_color");
        //        webcam = hwMap.get(WebcamName.class, "webcam");
        //rightSonic.changeI2cAddress(0xe2);

        // Set Motor and Servo Direction
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        droidLifterRight.setDirection(DcMotor.Direction.REVERSE);
        droidLifterLeft.setDirection(DcMotor.Direction.REVERSE);
        outExtrusion.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
        foundation1.setDirection(Servo.Direction.FORWARD);
        foundation1.setDirection(Servo.Direction.FORWARD);
        clawTurner.setDirection(Servo.Direction.FORWARD);
        clawAid.setDirection(Servo.Direction.FORWARD);


        // Set Motor to Zero Power Behavior
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Servos to Zero Power
        leftIntake.setPower(0);
        leftIntake.setPower(0);


        // Set Motors to Run Without Encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outExtrusion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        droidLifterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        droidLifterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set IMU Parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Initialize IMU
        imuActual.initialize(parameters);
    }
}