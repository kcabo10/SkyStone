package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "Skystone Red Depot Support With Strafing", group = "Beep")
public class SkystoneRedDepotSupportWithStrafing extends LinearOpMode {

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

        //wait for start
        waitForStart();

        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightFront.setTargetPosition(5500);
        robot.leftBack.setTargetPosition(5500);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.rightFront.isBusy() && robot.leftBack.isBusy() && opModeIsActive()) {
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(1);
            robot.leftBack.setPower(1);
            robot.rightBack.setPower(0);
        }

        sleep(2000);

        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightFront.setTargetPosition(-5700);
        robot.leftBack.setTargetPosition(-5700);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.rightFront.isBusy() && robot.leftBack.isBusy() && opModeIsActive()) {
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(1);
            robot.leftBack.setPower(1);
            robot.rightBack.setPower(0);
        }

        sleep(2000);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setTargetPosition(3400);
        robot.rightFront.setTargetPosition(-3400);
        robot.leftBack.setTargetPosition(-3400);
        robot.rightBack.setTargetPosition(3400);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.rightFront.isBusy() && robot.leftBack.isBusy() && opModeIsActive()) {
            robot.leftFront.setPower(1);
            robot.rightFront.setPower(1);
            robot.leftBack.setPower(1);
            robot.rightBack.setPower(1);
        }

    }

}