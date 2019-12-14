package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our autonomous program for the depot side on the blue side of the field. This program runs
 * without the phone light for Tensor Flow. This is the go to program. This program... .
 */
@Disabled
@Autonomous(name = "SkystoneRedBuildingSite", group = "Beep")
public class SkystoneRedBuildingSite extends LinearOpMode {

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
    double offset = .31;

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

        /*
         * Use ultrasonic to read south wall
         * if > 1 set y pos to value
         * if < 1 set grid pos to (0.5,2.5)
         */
        // Set initial Grid Nav position
        //robot.leftSonic.ping();
        robot.rightSonic.ping();
        sleep(200);
        //double leftDistance = (double) robot.leftSonic.getDistance() / 2.54 / 24 + offset;
        double rightDistance = (double) robot.rightSonic.getDistance() / 2.54 / 24 + offset;

        //telemetry.addData("leftDistance", leftDistance);
        telemetry.addData("rightDistance", rightDistance);
        telemetry.update();

        double yDistance = .375;

//        if (leftDistance >= .5) {
//            telemetry.addData("leftDistance", leftDistance);
//            telemetry.update();
//            gridNavigation.setGridPosition(leftDistance, yDistance, 90);
//        }
        if (rightDistance >= .5) {
            gridNavigation.setGridPosition(rightDistance, yDistance, 90);
            telemetry.addData("rightDistance", rightDistance);
        } else {
            telemetry.addData("Default", "");
            gridNavigation.setGridPosition(1.3, yDistance, 90);
        }
        telemetry.update();

        // Start up Tensor Flow to read skystone position while landing

        //wait for start
        waitForStart();


        int X = 0;
        int Y = 1;
        /*
         * UPDATE GRID NAV WITH END ANGLE
         */
        //int END_ANGLE = 2;

        // Skystone pos 1
        double[] TOWARD_FOUNDATION = {.8, 1.0}; /* END_ANGLE = 0 */
        double[] FOUNDATION = {.8, 1.5}; /* END_ANGLE = 0 */

        gridNavigation.driveToPosition(TOWARD_FOUNDATION[X], TOWARD_FOUNDATION[Y], .5);
        gridNavigation.driveToPosition(FOUNDATION[X], FOUNDATION[Y], .5);

    }
}