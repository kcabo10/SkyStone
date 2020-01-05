package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "parking under bridge", group = "Beep")
public class autonomousSkystoneJosh extends LinearOpMode {

    // Declaring a timer
    public ElapsedTime runtime = new ElapsedTime();
    public String foundTargetName = "";
    //Calling our hardware map
    HardwareBeep robot = new HardwareBeep();
    // Calling the Library Gyro program to use the gyro turn function
    LibraryGyro gyroTurn = new LibraryGyro();
    // Calling the Library Gyro Drive program to use the gyro drive function
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();

    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();

    /**
     * This method is the main body of our code which contains the set of commands carried out in our crater side autonomous program.
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();
        //initializing the hardware map
        robot.init(hardwareMap);
        //initializing the gyro turn function
        gyroTurn.init(robot, telemetry);
        //initializing the gyro drive function
        gyroDrive.init(robot, telemetry, robot.rightBack);
        gridNavigation.init(robot, gyroTurn, telemetry);
        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();


        double SET_POS_X = 4.5;
        double SET_POS_Y = 0.4;
        double DRIVE_TO_SKYSTONE_X = 4.5;
        double DRIVE_TO_SKYSTONE_Y = 1.8;
        double POWER_FOR_MOTORS = 0.5;
        double REVERSE_FROM_SKYSTONE_X = 4.5;
        double REVERSE_FROM_SKYSTONE_Y = 1.2;


        //TODO read the skystone in init phase.
        waitForStart();

        gridNavigation.setGridPosition(SET_POS_X, SET_POS_Y,90);

        gridNavigation.driveToPosition(DRIVE_TO_SKYSTONE_X, DRIVE_TO_SKYSTONE_Y, POWER_FOR_MOTORS);

        gridNavigation.driveToPosition(REVERSE_FROM_SKYSTONE_X, REVERSE_FROM_SKYSTONE_Y, POWER_FOR_MOTORS);

        gridNavigation.strafeToPosition(REVERSE_FROM_SKYSTONE_X, REVERSE_FROM_SKYSTONE_Y, POWER_FOR_MOTORS);






    }
}