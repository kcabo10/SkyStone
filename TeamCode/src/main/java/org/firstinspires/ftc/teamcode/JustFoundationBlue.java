package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Just Foundation Blue", group = "Beep")
public class JustFoundationBlue extends LinearOpMode {

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

        gridNavigation.setGridPosition(1.5,  0.296, 90);

        gridNavigation.strafeToPosition(.9, 0.296, 0.6, 1);

        gridNavigation.driveToPosition(.9, 1.7,.6);

        robot.foundation1.setPosition(.5);
        robot.foundation2.setPosition(.5);
        sleep(500);

        gridNavigation.driveToPositionBackwards(.9,0.296,.6);

        robot.foundation1.setPosition(-1);
        robot.foundation2.setPosition(1);

        gridNavigation.strafeToPosition(3,.296,.6,0);

    }
}