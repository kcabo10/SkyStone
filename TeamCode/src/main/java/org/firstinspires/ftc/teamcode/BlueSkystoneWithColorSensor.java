package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.sensors.LibraryColorSensor;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our autonomous program for the depot side on the blue side of the field. This program runs
 * without the phone light for Tensor Flow. This is the go to program. This program... .
 */
@Disabled
@Autonomous(name = "blue side color sensor auto", group = "Beep")
public class BlueSkystoneWithColorSensor extends LinearOpMode {

    // Declaring a timer
    public ElapsedTime runtime = new ElapsedTime();
    public String foundTargetName = "";
    float StoneColor = 1;
    //Calling our hardware map
    HardwareBeep robot = new HardwareBeep();
    // Calling the Library Gyro program to use the gyro turn function
    LibraryGyro gyroTurn = new LibraryGyro();
    // Calling the Library Gyro Drive program to use the gyro drive function
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();

    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();

    LibraryColorSensor colorSensorLib = new LibraryColorSensor();

    private int readColorSensor = 0;

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
        //initializing the gyro turn function
        gyroTurn.init(robot, telemetry);
        //initializing the gyro drive function
        gyroDrive.init(robot, telemetry, robot.rightBack);

        gridNavigation.init(robot, gyroTurn, telemetry);

        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();


        int X = 0;
        int Y = 1;

        double[] SKYSTONE_POS_1 = {1.7, 1.5};
        double[] SKYSTONE_POS_2 = {1.3, 1.5};
        double[] SKYSTONE_POS_3 = {1, 1.5};
        double[] GRAB_SKYSTONE_POS_1 = {1.7, 2};
        double[] GRAB_SKYSTONE_POS_2 = {1.3, 2};
        double[] GRAB_SKYSTONE_POS_3 = {1, 2};
        double[] BACKING_UP_1 = {1.7, 1.6};
        double[] BACKING_UP_2 = {1.3, 1.6};
        double[] BACKING_UP_3 = {1, 1.6};

        //Same end to each case
        double[] DELIVERING_SKYSTONE = {5.3, 1.5};
        double[] GRAB_FOUNDATION = {5.3, 1.75};
        double[] BACK_UP = {5.3, .9};
        double[] REPOSITION_FOUNDATION = {5, 1.2};
        double[] PARKING_POS = {3.3, 1.6};


        // Start position for foundation hooks
        robot.foundation1.setPosition(-1);
        robot.foundation2.setPosition(1);

        waitForStart();

        gridNavigation.setGridPosition(1.7, .3645, 270);

        boolean stoneFound = false;

        while (!stoneFound) {

            telemetry.addData("Current State", readColorSensor);
            telemetry.addData("Stone Color", StoneColor);
            telemetry.addData("Stone found", stoneFound);
            telemetry.update();

            switch (readColorSensor) {

                case 0:
                    gridNavigation.driveToPositionBackwards(SKYSTONE_POS_1[X], SKYSTONE_POS_1[Y], .5);
                    sleep(1000);
                    StoneColor = colorSensorLib.readSaturation(robot, "sensor_color");
                    telemetry.addData("case 0 color sensor reading", StoneColor);
                    telemetry.update();

                    if (StoneColor >= .2) {
                        gridNavigation.strafeToPositionBackwards(SKYSTONE_POS_2[X], SKYSTONE_POS_2[Y], .5, 1);
                        readColorSensor++;

                        telemetry.addData("Code Pos 10", "");
                        telemetry.update();


                    } else if (StoneColor <= .2) {
                        telemetry.addData("Skystone found :)", "");
                        telemetry.update();
                        robot.rightIntake.setPower(1);
                        robot.leftIntake.setPower(-1);
                        gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE_POS_1[X], GRAB_SKYSTONE_POS_1[Y], .5);
                        robot.rightIntake.setPower(0);
                        robot.leftIntake.setPower(0);
                        gridNavigation.driveToPosition(BACKING_UP_1[X], BACKING_UP_1[Y], .5);

                        telemetry.addData("Code Pos 20", "");
                        telemetry.update();

                        stoneFound = true;
                    }
                    break;
                case 1:
                    StoneColor = colorSensorLib.readSaturation(robot, "sensor_color");

                    telemetry.addData("case 1 color sensor reading", StoneColor);
                    telemetry.update();


                    telemetry.addData("Code Pos 25", "");
                    telemetry.update();

                    if (StoneColor >= .2) {
                        gridNavigation.strafeToPositionBackwards(SKYSTONE_POS_3[X], SKYSTONE_POS_3[Y], .5, 1);
                        readColorSensor++;

                        telemetry.addData("Code Pos 30", "");
                        telemetry.update();


                    } else if (StoneColor <= .2) {
                        telemetry.addData("FOUND SKYSTONE HERE", "");
                        telemetry.update();
                        robot.rightIntake.setPower(1);
                        robot.leftIntake.setPower(-1);
                        gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE_POS_2[X], GRAB_SKYSTONE_POS_2[Y], .5);
                        robot.rightIntake.setPower(0);
                        robot.leftIntake.setPower(0);
                        gridNavigation.driveToPosition(BACKING_UP_2[X], BACKING_UP_2[Y], .5);
                        stoneFound = true;


                        telemetry.addData("Code Pos 40", "");
                        telemetry.update();
                        sleep(1000);

                    }
                    break;
                case 2:
                    telemetry.addData("Grabbing Pos 3 Stone", "");
                    telemetry.update();
                    sleep(1000);

                    robot.rightIntake.setPower(1);
                    robot.leftIntake.setPower(-1);
                    gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE_POS_3[X], GRAB_SKYSTONE_POS_3[Y], .3);
                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);
                    gridNavigation.driveToPosition(BACKING_UP_3[X], BACKING_UP_3[Y], .5);
                    stoneFound = true;

                    break;

                // should never get to this case but in case it can't find the skystone position
                // it goes to this default case
                default:

                    runtime.reset();
                    telemetry.addData("Telemetry", "Didn't see skystone pos");
                    telemetry.update();
                    break;
            }
        }

//        robot.rightSonic.ping();
//        sleep(200);
//        double rightDistance = (double) robot.rightSonic.getDistance() / 2.54 / 24 + offset;
//
//        telemetry.addData("rightDistance", rightDistance);
//        telemetry.update();
//
//        double yDistance = .375;
//
//        if (rightDistance >= .5) {
//            gridNavigation.setGridPosition (-rightDistance, yDistance, 90);
//            telemetry.addData("rightDistance", rightDistance);
//        } else {
//            telemetry.addData("Default", "");
//            gridNavigation.setGridPosition(1.5, yDistance, 90);
//        }
//        telemetry.update();

        gridNavigation.strafeToPosition(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y],.5,1);

        gridNavigation.driveToPosition(GRAB_FOUNDATION[X], GRAB_FOUNDATION[Y],.5);
        robot.foundation1.setPosition(.5);
        robot.foundation2.setPosition(.5);

        robot.rightIntake.setPower(-1);
        robot.leftIntake.setPower(1);
        gridNavigation.driveToPositionBackwards(BACK_UP[X], BACK_UP[Y],.5);

        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftBack.setTargetPosition(3656);
        robot.rightBack.setTargetPosition(-3656);

        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftBack.setPower(1);
        robot.rightBack.setPower(1);

        while (robot.rightBack.isBusy() && robot.leftBack.isBusy()) {
        }

        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        robot.rightIntake.setPower(0);
        robot.leftIntake.setPower(0);

        telemetry.addData("Should have turned", "");
        telemetry.update();

    }

}