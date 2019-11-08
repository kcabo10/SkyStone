package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Blue Foundation", group = "Beep")
public class RedFoundation extends LinearOpMode {

    // Declaring a timer
    public ElapsedTime runtime = new ElapsedTime();
    // Calling hardware map
    HardwareBeep robot = new HardwareBeep();

    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();

    LibraryGyro gyroTurn = new LibraryGyro();
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();

    @Override
    public void runOpMode() {

        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();
        //initializing the hardware map
        robot.init(hardwareMap);
        gyroDrive.init(robot, telemetry, robot.rightBack);
        gyroTurn.init(robot, telemetry);
        gridNavigation.init(robot, gyroTurn, telemetry);
        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();

        //wait for start
        waitForStart();

        //Grid nav set in perspective on positive x,y and blue build site

        gridNavigation.setGridPosition(0.296,  2.1041, 0);

        gridNavigation.driveToPosition(2.375, 2.1041,.6);
        gridNavigation.driveToPosition(2.375, 1.8,.6);

        gyroDrive.gyroDrive(0.5, -500, 0);

        gyroTurn.turnGyro(-60);

        gyroDrive.gyroDrive(0.5, 1500, 0);

//        gyroTurn.turnGyro(-60);
//
//        gyroDrive.gyroDrive(0.5, -1500, 0);
        robot.leftFront.setPower(0.5);
        robot.rightFront.setPower(-0.5);
        robot.leftBack.setPower(-0.5);
        robot.rightBack.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.7)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.leftFront.setPower(0.5);
        robot.rightFront.setPower(0.5);
        robot.leftBack.setPower(0.5);
        robot.rightBack.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .3)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        gyroTurn.turnGyro(60);

        gyroDrive.gyroDrive(0.5, 1500, 0);
    }
}