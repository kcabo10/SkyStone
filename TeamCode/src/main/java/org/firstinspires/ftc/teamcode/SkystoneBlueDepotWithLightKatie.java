package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our autonomous program for the depot side on the blue side of the field. This program runs
 * without the phone light for Tensor Flow. This is the go to program. This program... .
 */
@Autonomous(name = "Skystone Blue Depot With Light Katie", group = "Beep")
public class SkystoneBlueDepotWithLightKatie extends LinearOpMode {

    // Declaring a timer
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime clawaidruntime = new ElapsedTime();
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

        //wait for start
        waitForStart();

        gridNavigation.setGridPosition(2.02, .375, 270);

        runtime.reset();
        while (runtime.seconds() <= 3){
            gridNavigation.driveToPositionBackwards(2.02,1,1);
            getSkystonePos();
        }

        // Start up Tensor Flow to read skystone position while driving towards it



        int X = 0;
        int Y = 1;
        /*
         * UPDATE GRID NAV WITH END ANGLE
         */
        //int END_ANGLE = 2;

        // Skystone pos 1
        double[] SKYSTONE_POS_1 = {2.02, 1.65};
        double[] GRAB_SKYSTONE_POS_1 = {2.02, 2.6};
        double[] BACKING_UP = {2.02, 1.5};

        // Skystone pos 2
        double[] SKYSTONE_POS_2 = {1.8, 1.65};
        double[] GRAB_SKYSTONE_POS_2 = {1.6, 2.3};
        double[] BACKING_UP2 = {1.6, 1.5};

        // Skystone pos 3
        double[] SKYSTONE_POS_3 = {1.6, 1.7};
        double[] GRAB_SKYSTONE_POS_3 = {1.4, 1.9};
        double[] BACKING_UP3 = {1.4, 1.5};

        //Same end to each case
        double[] DELIVERING_SKYSTONE = {5, 1.5};
        double[] GRAB_FOUNDATION = {5, 1.7};
        double[] BACK_UP = {5, 1.4};
        double[] REPOSITION_FOUNDATION = {5, 1.4};
        double[] PARKING_POS = {3.3, 1.6};

        // This is a switch block that plays the program in relation to the skystone position
        // Tensor Flow reads
        switch (SkystonePosition) {

            // If Tensor Flow reads the first skystone position then it plays this case
            case "Pos 1":
                telemetry.addData("Telemetry", "Skystone Pos = Pos 1");
                printTelemetry(20);
                if (SkystonePosition == "Pos 1") {

                    gridNavigation.driveToPositionBackwards(SKYSTONE_POS_1[X], SKYSTONE_POS_1[Y],1);

                    runtime.reset();
                    /**
                     * Intake Skystone
                     */
                    robot.rightIntake.setPower(-1);
                    robot.leftIntake.setPower(1);
                    gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE_POS_1[X], GRAB_SKYSTONE_POS_1[Y],.4);

                    gridNavigation.driveToPositionBackwards(BACKING_UP[X], BACKING_UP[Y],1);

                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);

//                        robot.claw.setPosition(0);
//                        clawaidruntime.reset();
//                        while (clawaidruntime.milliseconds() > 250) {
//                            //sleep
//                        }
//                        robot.clawAid.setPosition(1);
//                        clawaidruntime.reset();
//                        while (clawaidruntime.milliseconds() > 250){
//                            //sleep
//                        }
//                        robot.claw.setPosition(1);
//                        clawaidruntime.reset();
//                        while (clawaidruntime.milliseconds() > 250){
//                            //sleep
//                        }
//                        robot.clawAid.setPosition(0);

                    gridNavigation.strafeToPosition(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y],1,0);
                    robot.outExtrusion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.outExtrusion.setTargetPosition(-1000);
                    robot.outExtrusion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.outExtrusion.setPower(1);
                    gridNavigation.driveToPosition(GRAB_FOUNDATION[X], GRAB_FOUNDATION[Y],1);
                    robot.outExtrusion.setPower(0);
                    gridNavigation.driveToPositionBackwards(BACK_UP[X], BACK_UP[Y],1);
                    gridNavigation.driveToPosition(REPOSITION_FOUNDATION[X], REPOSITION_FOUNDATION[Y],1);
                    gridNavigation.driveToPositionBackwards(PARKING_POS[X], PARKING_POS[Y],1);


                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(30);
                }
                break;

            // If Tensor Flow reads the second skystone position then it plays this case
            case "Pos 2":
                telemetry.addData("Telemetry", "Skystone Pos = 2");
                printTelemetry(40);
                if (SkystonePosition == "Pos 2") {

                    gridNavigation.driveToPositionBackwards(SKYSTONE_POS_2[X], SKYSTONE_POS_2[Y],1);

                    runtime.reset();
                    /**
                     * Intake Skystone
                     */
                    robot.rightIntake.setPower(-1);
                    robot.leftIntake.setPower(1);
                    gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE_POS_2[X], GRAB_SKYSTONE_POS_2[Y],.4);

                    gridNavigation.driveToPositionBackwards(BACKING_UP2[X], BACKING_UP2[Y],1);

                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);

//                        robot.claw.setPosition(0);
//                        clawaidruntime.reset();
//                        while (clawaidruntime.milliseconds() > 250) {
//                            //sleep
//                        }
//                        robot.clawAid.setPosition(1);
//                        clawaidruntime.reset();
//                        while (clawaidruntime.milliseconds() > 250){
//                            //sleep
//                        }
//                        robot.claw.setPosition(1);
//                        clawaidruntime.reset();
//                        while (clawaidruntime.milliseconds() > 250){
//                            //sleep
//                        }
//                        robot.clawAid.setPosition(0);

                    gridNavigation.strafeToPosition(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y],1,0);
                    robot.outExtrusion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.outExtrusion.setTargetPosition(-1000);
                    robot.outExtrusion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.outExtrusion.setPower(1);
                    gridNavigation.driveToPosition(GRAB_FOUNDATION[X], GRAB_FOUNDATION[Y],1);
                    robot.outExtrusion.setPower(0);
                    gridNavigation.driveToPositionBackwards(BACK_UP[X], BACK_UP[Y],1);
                    gridNavigation.driveToPosition(REPOSITION_FOUNDATION[X], REPOSITION_FOUNDATION[Y],1);
                    gridNavigation.driveToPositionBackwards(PARKING_POS[X], PARKING_POS[Y],1);

                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(50);
                }
                break;

            // If Tensor Flow reads the third skystone position then it plays this case
            case "Pos 3":
                telemetry.addData("Telemetry", "Skystone Pos = 3");
                printTelemetry(60);
                if (SkystonePosition == "Pos 3") {

                    gridNavigation.driveToPositionBackwards(SKYSTONE_POS_3[X], SKYSTONE_POS_3[Y],.7);

                    runtime.reset();
                    /**
                     * Intake Skystone
                     */
                    robot.rightIntake.setPower(-1);
                    robot.leftIntake.setPower(1);
                    gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE_POS_3[X], GRAB_SKYSTONE_POS_3[Y],.2);
                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);

                    telemetry.addData("Now driving to position backwards", "");
                    telemetry.update();

//                    gridNavigation.driveToPosition(BACKING_UP[X], BACKING_UP[Y],1);
//                    gridNavigation.strafeToPositionBackwards(DELIVER_SKYSTONE[X], DELIVER_SKYSTONE[Y], 1,0);
//                    gridNavigation.strafeToPositionBackwards(SKYSTONE2_POS_1[X], SKYSTONE2_POS_1[Y], 1,1);
//                    gridNavigation.driveToPositionBackwards(TOWARD_SKYSTONE[X], TOWARD_SKYSTONE[Y], 1);
//                    gridNavigation.driveToPosition(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y], 1);
//                    gridNavigation.strafeToPositionBackwards(DELIVER_SKYSTONE[X], DELIVER_SKYSTONE[Y], 1,0);

                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(70);
                }
                break;
            // should never get to this case but in case it can't find the skystone position
            // it goes to this default case
            default:

                runtime.reset();
                telemetry.addData("Telemetry", "Didn't see skystone pos");
                telemetry.update();
                break;
        }

        telemetry.addData("Orig Coords X", gridNavigation.xOrigin);
        telemetry.addData("Orig Coords Y", gridNavigation.yOrigin);
        telemetry.update();


//        /*
//         * PLACE SKYSTONE
//         */
//        /*
//         * LOWER FOUNDATION ATTACHMENT
//         */
//        // drive to reposition foundation
//        gridNavigation.driveToPositionBackwards(REPOSITIONING_POS[X], REPOSITIONING_POS[Y], .7);
//        // parking under alliance sky bridge
//        gridNavigation.driveToPosition(PARKING_POS[X], PARKING_POS[Y], .5);


    }

    /**
     * This method prints telemetry for our autonomous program
     *
     * @param codePos This is the value we use in telemetry to see where in the code we are
     */
    private void printTelemetry(int codePos) {
        telemetry.addData("skystone Pos", SkystonePosition);
        telemetry.addData("Code Position", codePos);
        telemetry.update();
    }

    /**
     * This method calls Tensor Flow in order to read the skystone position
     */
    public void getSkystonePos() {
        int debounceCount = 0;
        long startTime = 0;
        String previousPosition;
        /*
         * UPDATE WITH NEW REPOSITORY
         */
        SkystonePosition = tensorFlow.findSkystone();

        // Switch block that indicated which skystone position it reads
        switch (SkystonePosition) {
            case ("Pos 1"):
                telemetry.addData("Telemetry", "right");
                telemetry.update();
                SkystonePosition = "Pos 3";
                break;
            case ("Pos 2"):
                telemetry.addData("Telemetry", "Middle");
                telemetry.update();
                break;
            case ("Pos 3"):
                telemetry.addData("Telemetry", "left");
                telemetry.update();
                SkystonePosition = "Pos 1";
                break;

            // If it reads unknown than it goes to this default case
            default:
                telemetry.addData("Telemetry", "Unknown Position");
                telemetry.update();
                // sets skystone pos to center as default
                SkystonePosition = "Pos 1";
                break;
        }

        telemetry.update();
    }
}