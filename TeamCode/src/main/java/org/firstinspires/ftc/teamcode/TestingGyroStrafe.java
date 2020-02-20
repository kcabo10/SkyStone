package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
@Autonomous(name = "Testing Gyro Strafe", group = "Beep")
public class TestingGyroStrafe extends LinearOpMode {

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

        waitForStart();

        while (opModeIsActive()) {
//            gridNavigation.setGridPosition(0, 0, 0);


            //Did a lot of testing with different values.  Narrowed down kp and kd to be within .5
            //degrees reliably
//            gyroTurn.kp = 0.05;
////            gyroTurn.ki = .85;
//            gyroTurn.ki = 0;
//            gyroTurn.kd = 0.13;
//
//            double kp_array[] = {.7, .75, .8, .85, .9};
//
////        gyroTurn.kp = .6;
////        gyroTurn.kd = .3;
//
//            for (int i = 0; (i < kp_array.length); i++) {
//
////               gyroTurn.ki = 0;
////               gyroTurn.SetTunings(0.06, .885, .13); Current Best
//               gyroTurn.SetTunings(.05, kp_array[i], .13);
//
//                telemetry.addData("kp value is ", kp_array[i]);
//                telemetry.update();

                gyroTurn.turnGyro(90);
            sleep(2000);
                gyroTurn.turnGyro(90);
            sleep(2000);
                gyroTurn.turnGyro(90);
                sleep(2000);
            }


    }
}