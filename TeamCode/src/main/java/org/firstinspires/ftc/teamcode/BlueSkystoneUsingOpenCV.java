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
@Autonomous(name = "Blue Side Skystone Delivery", group = "Beep")
public class BlueSkystoneUsingOpenCV extends LinearOpMode {

    // Declaring a timer
    public ElapsedTime runtime = new ElapsedTime();
    public String foundTargetName = "";
    String SkystonePosition = "";
    float StoneColor = 1;
    //Calling our hardware map
    HardwareBeep robot = new HardwareBeep();
    // Calling the Library Gyro program to use the gyro turn function
    LibraryGyro gyroTurn = new LibraryGyro();
    // Calling the Library Gyro Drive program to use the gyro drive function
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();

    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();

    LibraryOpenCVBlue opencv;

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

        opencv = new LibraryOpenCVBlue(robot, telemetry);

        gridNavigation.init(robot, gyroTurn, telemetry);

        // Start position for foundation hooks
        robot.foundation1.setPosition(1);
        robot.foundation2.setPosition(-1);

        getSkystonePos();

        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();


        int X = 0;
        int Y = 1;

        double[] SKYSTONE_POS_1 = {1.95, 2.9};
        double[] SKYSTONE_POS_2 = {1.4, 2.9};
        double[] SKYSTONE_POS_3 = {.8, 2.9};
        double[] BACKING_UP_1 = {2, 1.4};
        double[] BACKING_UP_2 = {1.5, 1.4};
        double[] BACKING_UP_3 = {1.4, 1.4};


        double[] SKYSTONE2_POS_1 = {1, 1.4};
        double[] SKYSTONE2_POS_2 = {.5, 1.4};
        double[] SKYSTONE2_POS_3 = {.8, 1.3};
        double[] GRAB_SKYSTONE2_POS_1 = {1, 2.6};
        double[] GRAB_SKYSTONE2_POS_2 = {.5, 2.6};
        double[] GRAB_SKYSTONE2_POS_3 = {.3, 1.7};
        double[] BACKING_UP2_1 = {.9, 1.4};
        double[] BACKING_UP2_2 = {.5, 1.4};
        double[] BACKING_UP2_3 = {.6, 1.4};


        //Same end to each case
        double[] DELIVERING_SKYSTONE = {3.5, 1.4};
        double[] PARKING_POS = {2.9, 1.4};

        waitForStart();

        gridNavigation.setGridPosition(1.7, translateForBlue(.3645), 90);


        switch (SkystonePosition) {
            /**
             * This first pos is working just need to add second Skystone
             */
            //Right Skystone position
            case "Pos 1":

            telemetry.addData("Telemetry", "Skystone Pos = 1");
                telemetry.update();

            intakeSkystone();
            gridNavigation.driveToPositionBackwards(SKYSTONE_POS_1[X], translateForBlue(SKYSTONE_POS_1[Y]), .2);
            gridNavigation.driveToPosition(BACKING_UP_1[X], translateForBlue(BACKING_UP_1[Y]), .5);

            sleep(500);

            gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], translateForBlue(DELIVERING_SKYSTONE[Y]), .5);
            pushOutSkystone();
            sleep(500);

//            gridNavigation.driveToPosition(SKYSTONE2_POS_1[X], translateForBlue(SKYSTONE2_POS_1[Y]), .5);
//            intakeSkystone();
//            sleep(500);
//            gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE2_POS_1[X], translateForBlue(GRAB_SKYSTONE2_POS_1[Y]), .2);
//
//            gridNavigation.driveToPosition(BACKING_UP2_1[X], translateForBlue(BACKING_UP2_1[Y]), .5);
//            sleep(500);
//
//            gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], translateForBlue(DELIVERING_SKYSTONE[Y]), .5);
//            pushOutSkystone();
//            sleep(400);

            gridNavigation.driveToPosition(PARKING_POS[X], translateForBlue(PARKING_POS[Y]), .5);

                break;

            //Middle Skystone position
            case "Pos 2":

                /**
                 * This second pos is working just needs a bit of tuning
                 * We should come at an angle for the second stone in order to save time
                 */

                telemetry.addData("Telemetry", "Skystone Pos = 2");
                telemetry.update();

                intakeSkystone();
                gridNavigation.driveToPositionBackwards(SKYSTONE_POS_2[X], translateForBlue(SKYSTONE_POS_2[Y]), .2);
                gridNavigation.driveToPosition(BACKING_UP_2[X], translateForBlue(BACKING_UP_2[Y]), .5);
                sleep(500);

                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], translateForBlue(DELIVERING_SKYSTONE[Y]), .5);
                pushOutSkystone();
                sleep(1000);

//                gridNavigation.driveToPosition(SKYSTONE2_POS_2[X], translateForBlue(SKYSTONE2_POS_2[Y]), .5);
//                intakeSkystone();
//                gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE2_POS_2[X], translateForBlue(GRAB_SKYSTONE2_POS_2[Y]), .2);
//
//                gridNavigation.driveToPosition(BACKING_UP2_2[X], translateForBlue(BACKING_UP2_2[Y]), .5);
//                sleep(500);
//
//                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], translateForBlue(DELIVERING_SKYSTONE[Y]), .5);
//                pushOutSkystone();
//                sleep(1000);

                gridNavigation.driveToPosition(PARKING_POS[X], translateForBlue(PARKING_POS[Y]), .5);

                break;

            //Position closest to skybridge
            case "Pos 3":

                telemetry.addData("Telemetry", "Skystone Pos = 3");
                telemetry.update();

                intakeSkystone();
                gridNavigation.driveToPositionBackwards(SKYSTONE_POS_3[X], translateForBlue(SKYSTONE_POS_3[Y]), .2);
                gridNavigation.driveToPosition(BACKING_UP_3[X], translateForBlue(BACKING_UP_3[Y]), .5);

                sleep(500);

                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], translateForBlue(DELIVERING_SKYSTONE[Y]), .5);
                pushOutSkystone();
                sleep(1000);

//                gridNavigation.driveToPosition(SKYSTONE2_POS_3[X], translateForBlue(SKYSTONE2_POS_3[Y]), .5);
//                intakeSkystone();
//                sleep(500);
//                gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE2_POS_3[X], translateForBlue(GRAB_SKYSTONE2_POS_3[Y]), .2);
//                sleep(500);
//                gridNavigation.driveToPosition(BACKING_UP2_3[X], translateForBlue(BACKING_UP2_3[Y]), .5);
//                sleep(500);
//
//                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], translateForBlue(DELIVERING_SKYSTONE[Y]), .5);
//                pushOutSkystone();
//                sleep(400);

                gridNavigation.driveToPosition(PARKING_POS[X], translateForBlue(PARKING_POS[Y]), .5);

                break;

            // should never get to this case but in case it can't find the skystone position
            // it goes to this default case
            default:

                telemetry.addData("Unknown Position", "Playing Pos 3");
                telemetry.update();

                intakeSkystone();
                gridNavigation.driveToPositionBackwards(SKYSTONE_POS_3[X], translateForBlue(SKYSTONE_POS_3[Y]), .2);
                gridNavigation.driveToPosition(BACKING_UP_3[X], translateForBlue(BACKING_UP_3[Y]), .5);

                sleep(500);

                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], translateForBlue(DELIVERING_SKYSTONE[Y]), .5);
                pushOutSkystone();
                sleep(1000);

//                gridNavigation.driveToPosition(SKYSTONE2_POS_3[X], translateForBlue(SKYSTONE2_POS_3[Y]), .5);
//                intakeSkystone();
//                sleep(500);
//                gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE2_POS_3[X], translateForBlue(GRAB_SKYSTONE2_POS_3[Y]), .2);
//                sleep(500);
//                gridNavigation.driveToPosition(BACKING_UP2_3[X], translateForBlue(BACKING_UP2_3[Y]), .5);
//                sleep(500);
//
//                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], translateForBlue(DELIVERING_SKYSTONE[Y]), .5);
//                pushOutSkystone();
//                sleep(400);

                gridNavigation.driveToPosition(PARKING_POS[X], translateForBlue(PARKING_POS[Y]), .5);
            break;
        }

    }

        public void intakeSkystone (){
            robot.rightIntake.setPower(-1);
            robot.leftIntake.setPower(1);
        }

        public void pushOutSkystone (){
            robot.rightIntake.setPower(1);
            robot.leftIntake.setPower(-1);
        }

        private double translateForBlue(double blue) {
            return 6-blue;
        }

        public void getSkystonePos () {
            int debounceCount = 0;
            long startTime = 0;
            String previousPosition;
            /*
             * UPDATE WITH NEW REPOSITORY
             */
            SkystonePosition = opencv.findSkystone();

            // Switch block that indicated which skystone position it reads
            switch (SkystonePosition) {
                case ("Pos 1"):
                    telemetry.addData("Telemetry", "left");
                    telemetry.update();
                    SkystonePosition = "Pos 3";
                    break;
                case ("Pos 2"):
                    telemetry.addData("Telemetry", "Middle");
                    telemetry.update();
                    SkystonePosition = "Pos 2";
                    break;
                case ("Pos 3"):
                    telemetry.addData("Telemetry", "right");
                    telemetry.update();
                    SkystonePosition = "Pos 1";
                    break;

                // If it reads unknown than it goes to this default case
                default:
                    telemetry.addData("Telemetry", "Unknown Position");
                    telemetry.update();
                    // sets skystone pos to center as default
                    SkystonePosition = "Pos 3";
                    break;
            }

            telemetry.update();
        }
    }