package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our autonomous program for the depot side on the blue side of the field. This program runs
 * without the phone light for Tensor Flow. This is the go to program. This program... .
 */

@Autonomous(name = "TestingGridNav", group = "Beep")
public class TestingGridNav extends LinearOpMode {

    // Declaring a timer
    public ElapsedTime runtime = new ElapsedTime();
    public String foundTargetName = "";
    //Calling our hardware map
    HardwareBeep robot = new HardwareBeep();
    // Calling the Library Gyro program to use the gyro turn function
    LibraryGyro gyroTurn = new LibraryGyro();
    // Calling the Library Gyro Drive program to use the gyro drive function
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    // Calling the Library Grid Nav Library to use the grid navigation functions
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();

    // Calling the Library Tensor Flow No Light to use the Tensor Flow function without
    LibraryTensorFlowObjectDetectionWithLight tensorFlow =
            new LibraryTensorFlowObjectDetectionWithLight(robot, telemetry);
    // Declaring skystone position value to read what position Tensor Flow sees the skystone position
    String SkystonePosition = "";

    /**
     * This method is the main body of our code which contains the set of commands carried out in our crater side autonomous program.
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();
        //initializing the hardware map
        robot.init(hardwareMap);
        //initializing the grid Nav function
        gridNavigation.init(robot, gyroTurn, telemetry);
        //initializing the gyro turn function
        gyroTurn.init(robot, telemetry);
        //initializing the gyro drive function
        gyroDrive.init(robot, telemetry, robot.rightBack);
        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();

        waitForStart();

        gridNavigation.setGridPosition(0, 0, 0);
//
//        gridNavigation.strafeToPosition(1, 0, .5, 0);
//
////        sleep(3000);
////
//        gridNavigation.strafeToPosition(0, 0, .7, 1);
//
//
////        sleep(3000);
//
        gridNavigation.strafeToPositionBackwards(2, 0, .5, 0);
//
        sleep(3000);
//
        gridNavigation.strafeToPositionBackwards(0, 0, .7, 1);
//
//        gridNavigation.driveToPosition(0, 1, .3);
//
//        sleep(10000);

//        gyroDrive.gyroDriveVariableP(.3, 1440, 0, .03);

//        gridNavigation.driveToPosition(3, 0, .3);
//
        sleep(5000);


//        gridNavigation.driveToPositionBackwards(1, 0, .2);

//        sleep(5000);
//
//        sleep(3000);
//
//        gridNavigation.driveToPosition(3, 1, .3);
//
//        sleep(3000);
//
//
//        gridNavigation.driveToPosition(3, 3, .3);
//
//
//
//        sleep(3000);
//
//        gridNavigation.driveToPositionBackwards(3, 1, .3);
//
//        sleep(3000);
//
//        gridNavigation.driveToPositionBackwards(3, 0, .3);
//
//        sleep(3000);
//
//        gridNavigation.driveToPositionBackwards(1, 0, .3);


//
//        telemetry.addData("Angle", gridNavigation.turnAngle);
//        telemetry.update();

//        sleep(3000);

        int X = 0;
        int Y = 1;

        double[] TO_SKYSTONE_1 = {2.4, .3645};
        double[] TO_SKYSTONE_2 = {2, .3645};
        double[] SKYSTONE_POS_1 = {2.4, 2};
        double[] SKYSTONE_POS_2 = {2, 2};
        double[] SKYSTONE_POS_3 = {1.7, 2};
        double[] GRAB_SKYSTONE_POS_1 = {1.8, 2};
        double[] GRAB_SKYSTONE_POS_2 = {1.5, 2};
        double[] GRAB_SKYSTONE_POS_3 = {1.1, 2};
        double[] BACKING_UP_1 = {1.8, 1.5};
        double[] BACKING_UP_2 = {1.5, 1.5};
        double[] BACKING_UP_3 = {1.1, 1.5};


        gridNavigation.setGridPosition(1.7, .3645, 270);

                telemetry.addData("Telemetry", "Skystone Pos = 1");
                telemetry.update();

                gridNavigation.driveToPositionBackwards(TO_SKYSTONE_1[X], TO_SKYSTONE_1[Y], .2);

                sleep(3000);
                robot.rightIntake.setPower(1);
                robot.leftIntake.setPower(-1);
                gridNavigation.strafeToPositionBackwards(SKYSTONE_POS_1[X], SKYSTONE_POS_1[Y], .7, 0);

                sleep(3000);
                gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE_POS_1[X], GRAB_SKYSTONE_POS_1[Y], .3);

                sleep(3000);
                robot.rightIntake.setPower(0);
                robot.leftIntake.setPower(0);
                gridNavigation.strafeToPositionBackwards(BACKING_UP_1[X], BACKING_UP_1[Y], .7, 1);


            }
}
