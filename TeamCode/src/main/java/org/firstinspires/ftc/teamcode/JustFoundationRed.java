package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Just Foundation Red", group = "Beep")
public class JustFoundationRed extends LinearOpMode {

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
        telemetry.update();

        //wait for start
        waitForStart();

        robot.foundation1.setPosition(-1);
        robot.foundation2.setPosition(1);
        //Grid nav set in perspective on positive x,y and blue build site

        gridNavigation.setGridPosition(0.296,  1.3, 0);

        gridNavigation.driveToPosition(1.7, 1.3,.6);

        robot.foundation1.setPosition(.5);
        robot.foundation2.setPosition(.5);
        sleep(500);

        gyroDrive.gyroDriveVariableP(0.5, -500, 0, .01);

        gyroTurn.turnGyro(-45);

        gyroDrive.gyroDriveVariableP(0.5, 2300, 0,.01);

        robot.foundation1.setPosition(-1);
        robot.foundation2.setPosition(1);
        sleep(1000);

        gridNavigation.driveToPositionBackwards(0.296, 1.3, 0.6);

        gyroDrive.gyroStrafeLeft(0.6, 5500, 0);

//        gridNavigation.driveToPosition(2.375, 1.6,.6);
//
//        robot.foundation1.setPosition(.5);
//        robot.foundation2.setPosition(.5);
//        sleep(500);
//
//        gyroDrive.gyroDriveVariableP(0.5, -500, 0, .01);
//
//        gyroTurn.turnGyro(-45);
//
//        gyroDrive.gyroDriveVariableP(0.5, 2300, 0,.01);
//
//        robot.foundation1.setPosition(-1);
//        robot.foundation2.setPosition(1);
//        sleep(1000);
////
////        gyroTurn.turnGyro(60);
//
//        gyroDrive.gyroDriveVariableP(0.5, -1500, 0,.01);
    }
}