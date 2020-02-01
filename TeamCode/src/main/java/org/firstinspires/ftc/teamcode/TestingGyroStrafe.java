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

        // Stop all motion;
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

//        gridNavigation.setGridPosition(1,0,90);
//
//        gyroDrive.setPIDCoeff(.03,.0006,.01);
//
//        gridNavigation.strafeToPosition(1, 1,.5, 0);
//
//        gyroDrive.setPIDCoeff(.03,.0006,.1);
//
//        gridNavigation.strafeToPosition(1, 2,.7,0);
//
//        gyroDrive.setPIDCoeff(.03,.0006,.3);
//
//        gridNavigation.strafeToPosition(1, 3,.9, 0);



        //gyroDrive.setPIDCoeff(0,0.03,0);

        //gridNavigation.driveToPositionBackwards(1, 4.5,.5);


//
//        gyroDrive.gyroStrafeRight(.5, 300, 0);
//        sleep(1000);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // The next step is to set the encoders to run
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();

        while (runtime.seconds() <= 1) {
            robot.leftFront.setPower(.5);
            robot.leftBack.setPower(.5);
            robot.rightFront.setPower(.5);
            robot.rightBack.setPower(.5);

            telemetry.addData("rf Ticks", robot.rightFront.getCurrentPosition());
            telemetry.addData("rb Ticks", robot.rightBack.getCurrentPosition());
            telemetry.addData("lf Ticks", robot.leftFront.getCurrentPosition());
            telemetry.addData("lb Ticks", robot.leftBack.getCurrentPosition());
            telemetry.update();
        }
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        sleep(1000);

        runtime.reset();

        while (runtime.seconds() <= 1) {
            robot.leftFront.setPower(-.5);
            robot.leftBack.setPower(-.5);
            robot.rightFront.setPower(-.5);
            robot.rightBack.setPower(-.5);
            telemetry.addData("rf Ticks", robot.rightFront.getCurrentPosition());
            telemetry.addData("rb Ticks", robot.rightBack.getCurrentPosition());
            telemetry.addData("lf Ticks", robot.leftFront.getCurrentPosition());
            telemetry.addData("lb Ticks", robot.leftBack.getCurrentPosition());
            telemetry.update();
        }

        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        telemetry.addData("rf Ticks", robot.rightFront.getCurrentPosition());
        telemetry.addData("rb Ticks", robot.rightBack.getCurrentPosition());
        telemetry.addData("lf Ticks", robot.leftFront.getCurrentPosition());
        telemetry.addData("lb Ticks", robot.leftBack.getCurrentPosition());
        telemetry.update();

        sleep(3000);



        // Kc = .05
        // Pc = 1.5
        // Dt = 0.00000144

        //Kp = .03
        // KI = 0
        // Kd = 166
        int i = 0;
        runtime.reset();
        while (runtime.seconds() < 2)
        {
            i++;
        }

        telemetry.addData("i", i);
        telemetry.update();
        sleep(2);

//        gridNavigation.strafeToPositionBackwards(.5,1.5,.4,0);
//        sleep(3000);
//        gridNavigation.strafeToPositionBackwards(1.5,1.5,.4,1);
    }
}