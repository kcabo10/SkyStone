package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue Foundation", group = "Beep")
public class BlueFoundation extends LinearOpMode {

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

        robot.foundation1.setPosition(1);
        robot.foundation2.setPosition(0);

        //Grid nav set in perspective on positive x,y and blue build site

        gridNavigation.setGridPosition(2.1041, 0.296, 90);

        gridNavigation.driveToPosition(2.1041, 2.375,.3);
        gridNavigation.driveToPosition(1.6, 2.375,.3);

        robot.foundation1.setPosition(0);
        robot.foundation2.setPosition(1);

        sleep(500);

        gyroDrive.gyroDrive(0.5, -500, 0);

        gyroTurn.turnGyro(45);

        gyroDrive.gyroDrive(0.5, 1500, 0);

        robot.foundation1.setPosition(1);
        robot.foundation2.setPosition(0);
        sleep(1000);

        gyroTurn.turnGyro(-60);

        gyroDrive.gyroDrive(0.5, -1500, 0);
    }
}