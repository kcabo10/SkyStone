package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Testing Wheels", group = "Beep")
public class TestingWheels extends LinearOpMode {

    // Declaring a timer
    public ElapsedTime runtime = new ElapsedTime();
    // Calling hardware map
    HardwareBeepTest robot = new HardwareBeepTest();

    @Override
    public void runOpMode() {

        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();
        //initializing the hardware map
        robot.init(hardwareMap);
        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();
        telemetry.update();


        //wait for start
        waitForStart();
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //go-ing-ing
        robot.leftBack.setTargetPosition(12000);
        robot.leftFront.setTargetPosition(12000);
        robot.rightFront.setTargetPosition(12000);
        robot.rightBack.setTargetPosition(12000);

        robot.rightBack.setPower(.5);
        robot.rightFront.setPower(.5);
        robot.leftBack.setPower(.5);
        robot.leftFront.setPower(.5);


        sleep(1234);
    }
}