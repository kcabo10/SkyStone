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
//@Disabled
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

        robot.claw.setPosition(0);

        opencv = new LibraryOpenCVBlue(robot, telemetry);
        opencv.initOpenCV();

        telemetry.addData("OpenCV initialized", "");
        telemetry.update();

        while (!isStarted()) {
            SkystonePosition = opencv.findSkystone();
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        opencv.shutDownOpenCV();

        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();


        int X = 0;
        int Y = 1;

        double[] SKYSTONE_POS_1 = {1.9, translateForBlue(2.7)};
        double[] SKYSTONE_POS_2 = {1.5, translateForBlue(2.7)};
        double[] SKYSTONE_POS_3 = {1.1, translateForBlue(2.7)};
        double[] BACKING_UP_1 = {1.9, translateForBlue(1.4)};
        double[] BACKING_UP_2 = {1.5, translateForBlue(1.4)};
        double[] BACKING_UP_3 = {1.1, translateForBlue(1.4)};


        double[] SKYSTONE2_POS_1 = {1.3, 1.4};
        double[] SKYSTONE2_POS_2 = {.6, 1.4};
        double[] SKYSTONE2_POS_3 = {.8, 1.4};
        double[] GRAB_SKYSTONE2_POS_1 = {1.3, 2.6};
        double[] GRAB_SKYSTONE2_POS_2 = {.6, 2.6};
        double[] GRAB_SKYSTONE2_POS_3 = {.3, 1.7};
        double[] BACKING_UP2_1 = {1.3, 1.4};
        double[] BACKING_UP2_2 = {.6, 1.4};
        double[] BACKING_UP2_3 = {.6, 1.4};

        //Same end to each case
        double[] DELIVERING_SKYSTONE_ON_FOUNDATION = {5.1, 1.35};
        double[] GRAB_FOUNDATION = {5.1, 1.85};
        double[] REPOSITION_FOUNDATION = {5.1, .65};

        double[] DELIVERING_SKYSTONE = {3.5, 1.4};
        double[] PARKING_POS = {3.55, translateForBlue(1.4)};
        double[] STARTING_POS = {1.7, translateForBlue(.3645)};

//        waitForStart();

        //opencv.openCVIsNotActive();

        gridNavigation.setGridPosition(STARTING_POS[X], STARTING_POS[Y], 270);


        switch (SkystonePosition) {
            //Right Skystone position
            case "left":

                telemetry.addData("Telemetry", "Skystone Pos = 1/left");
                telemetry.update();

                /**
                 * Drive Toward Skystone and Intake
                 */
                intakeSkystone();
                gridNavigation.driveToPositionBackwards(calcTargetX(STARTING_POS, SKYSTONE_POS_1, 1.5), 1.5, .55);

                gridNavigation.driveToPositionBackwards(SKYSTONE_POS_1[X], SKYSTONE_POS_1[Y], .2);

                gridNavigation.driveToPosition(calcTargetX(SKYSTONE_POS_1, STARTING_POS, 1.4), BACKING_UP_1[Y], .55);

                placeStoneFoundationLeftMiddle();

                //            /**
                //             * Drive to pick up second Skystone and deliver
                //             */
                //            gridNavigation.driveToPositionBackwards(SKYSTONE2_POS_1[X], SKYSTONE2_POS_1[Y], .55);
                //            intakeSkystone();
                //            sleep(100);
                //            gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE2_POS_1[X], GRAB_SKYSTONE2_POS_1[Y], .3);
                //
                //            gridNavigation.driveToPosition(calcTargetX(SKYSTONE2_POS_1,GRAB_SKYSTONE2_POS_1,1.4), BACKING_UP2_1[Y], .55);
                //            sleep(150);
                //
                //            gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y], .55);
                //            pushOutSkystone();

                /**
                 * Navigate under Skybridge
                 */
                gridNavigation.driveToPositionBackwards(PARKING_POS[X], PARKING_POS[Y], .55);

                break;

            //Middle Skystone position
            case "middle":

                /**
                 * This second pos is working just needs a bit of tuning
                 * We should come at an angle for the second stone in order to save time
                 */

                telemetry.addData("Telemetry", "Skystone Pos = 2/middle");
                telemetry.update();

                /**
                 * Drive Toward Skystone and Intake
                 */
                intakeSkystone();
                gridNavigation.driveToPositionBackwards(calcTargetX(STARTING_POS, SKYSTONE_POS_2, 1.5), 1.5, .55);

                gridNavigation.driveToPositionBackwards(SKYSTONE_POS_2[X], SKYSTONE_POS_2[Y], .2);
                sleep(200);
                gridNavigation.driveToPosition(calcTargetX(SKYSTONE_POS_2, STARTING_POS, 1.4), BACKING_UP_2[Y], .55);

                /**
                 * Drive to foundation, place stone, grab foundation and reposition
                 */
                placeStoneFoundationLeftMiddle();

//                /**
//                 * Drive to pick up second Skystone and deliver
//                 */
//                gridNavigation.driveToPositionBackwards(SKYSTONE2_POS_2[X], SKYSTONE2_POS_2[Y], .6);
//                intakeSkystone();
//                gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE2_POS_2[X], GRAB_SKYSTONE2_POS_2[Y], .2);
//                sleep(200);
//                gridNavigation.driveToPosition(calcTargetX(SKYSTONE2_POS_2,GRAB_SKYSTONE2_POS_2,1.5), BACKING_UP2_2[Y], .6);
//                sleep(150);
//
//                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y], .6);
//                pushOutSkystone();

                /**
                 * Navigate under Skybridge
                 */
                gridNavigation.driveToPositionBackwards(PARKING_POS[X], PARKING_POS[Y], .55);

                break;

            //Position closest to skybridge
            case "right":

                telemetry.addData("Telemetry", "Skystone Pos = 1/right");
                telemetry.update();

                /**
                 * Drive Toward Skystone and Intake
                 */
                intakeSkystone();
                gridNavigation.driveToPositionBackwards(calcTargetX(STARTING_POS, SKYSTONE_POS_3, 1.5), 1.5, .55);

                gridNavigation.driveToPositionBackwards(SKYSTONE_POS_3[X], SKYSTONE_POS_3[Y], .2);
                sleep(200);
                gridNavigation.driveToPosition(calcTargetX(STARTING_POS, SKYSTONE_POS_3, 1.3), BACKING_UP_3[Y], .55);

                /**
                 * Drive to foundation, place stone, grab foundation and reposition
                 */
                placeStoneFoundationRight();

//                /**
//                 * Drive to pick up second Skystone and deliver
//                 */
//                gridNavigation.driveToPositionBackwards(SKYSTONE2_POS_3[X], SKYSTONE2_POS_3[Y], .6);
//                intakeSkystone();
//                gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE2_POS_3[X], GRAB_SKYSTONE2_POS_3[Y], .2);
//                sleep(200);
//                gridNavigation.driveToPosition(calcTargetX(SKYSTONE2_POS_3,GRAB_SKYSTONE2_POS_3,1.4), BACKING_UP2_3[Y], .6);
//                sleep(150);
//
//                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y], .6);
//                pushOutSkystone();

                /**
                 * Navigate under Skybridge
                 */
                gridNavigation.driveToPositionBackwards(PARKING_POS[X], PARKING_POS[Y], .55);

                break;

            // should never get to this case but in case it can't find the skystone position
            // it goes to this default case
            default:

                telemetry.addData("Unknown Position", "Running Pos right");
                telemetry.update();
                /**
                 * Drive Toward Skystone and Intake
                 */
                intakeSkystone();
                gridNavigation.driveToPositionBackwards(calcTargetX(STARTING_POS, SKYSTONE_POS_3, 1.4), 1.4, .55);

                gridNavigation.driveToPositionBackwards(SKYSTONE_POS_3[X], SKYSTONE_POS_3[Y], .2);
                sleep(200);
                gridNavigation.driveToPosition(calcTargetX(STARTING_POS, SKYSTONE_POS_3, 1.4), BACKING_UP_3[Y], .6);

                /**
                 * Drive to foundation, place stone, grab foundation and reposition
                 */
                placeStoneFoundationRight();

//                /**
//                 * Drive to pick up second Skystone and deliver
//                 */
//                gridNavigation.driveToPositionBackwards(SKYSTONE2_POS_3[X], SKYSTONE2_POS_3[Y], .6);
//                intakeSkystone();
//                gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE2_POS_3[X], GRAB_SKYSTONE2_POS_3[Y], .2);
//                sleep(200);
//                gridNavigation.driveToPosition(BACKING_UP2_3[X], BACKING_UP2_3[Y], .6);
//                sleep(150);
//
//                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y], .6);
//                pushOutSkystone();
//                sleep(150);

                /**
                 * Navigate under Skybridge
                 */
                gridNavigation.driveToPositionBackwards(PARKING_POS[X], PARKING_POS[Y], .6);
                break;
        }

    }

    public void intakeSkystone() {
        robot.rightIntake.setPower(.85);
        robot.leftIntake.setPower(-.85);
    }

    public void pushOutSkystone() {
        robot.rightIntake.setPower(-.85);
        robot.leftIntake.setPower(.85);
    }

    public void placeStoneFoundationLeftMiddle() {

        int X = 0;
        int Y = 1;

        double[] DELIVERING_SKYSTONE_ON_FOUNDATION = {4.7, translateForBlue(.5)};
        double[] GRAB_FOUNDATION = {4.7, translateForBlue(2)};
        double[] REPOSITION_FOUNDATION = {4.7, translateForBlue(.65)};


        robot.clawAid.setPosition(1); //move the claw aid up
        sleep(300);
        robot.claw.setPosition(1); //close claw

        gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE_ON_FOUNDATION[X], DELIVERING_SKYSTONE_ON_FOUNDATION[Y], .5);
        sleep(200);

        gridNavigation.driveToPosition(GRAB_FOUNDATION[X], GRAB_FOUNDATION[Y], .29);
        robot.outExtrusion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outExtrusion.setTargetPosition(-500);
        robot.outExtrusion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outExtrusion.setPower(1);

        robot.foundation1.setPosition(.6);
        robot.foundation2.setPosition(.4);

        gridNavigation.driveToPositionBackwards(REPOSITION_FOUNDATION[X], REPOSITION_FOUNDATION[Y], .6);
        robot.outExtrusion.setPower(0);
        robot.claw.setPosition(0);

        robot.outExtrusion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outExtrusion.setTargetPosition(600);
        robot.outExtrusion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outExtrusion.setPower(1);

        /**
         * Turn foundation into building site
         */
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setTargetPosition(-1200);
        robot.rightBack.setTargetPosition(1200);

        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setPower(1);
        robot.rightBack.setPower(1);

        while (robot.rightBack.isBusy() && robot.leftBack.isBusy()) {
        }

        /**
         * Stop running wheels and out extrusion
         */
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.outExtrusion.setPower(0);

        /**
         * Drive forward to push foundation into building site and detach from foundation
         */
        gyroDrive.gyroDriveVariableP(.5, (int) (145.6 * 1.4), 0, .03);
        robot.foundation1.setPosition(1);
        robot.foundation2.setPosition(-1);

        /**
         * Reset grid position
         */
        gridNavigation.setGridPosition(4.87, translateForBlue(1.35), 0);
        sleep(200);
    }

    public void placeStoneFoundationRight(){

        int X = 0;
        int Y = 1;

        double[] DELIVERING_SKYSTONE_ON_FOUNDATION = {4.8, 1.5};
        double[] GRAB_FOUNDATION = {4.8, 2};
        double[] REPOSITION_FOUNDATION = {4.8, .65};


        robot.clawAid.setPosition(1); //move the claw aid up
        sleep(300);
        robot.claw.setPosition(1); //close claw

        gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE_ON_FOUNDATION[X], DELIVERING_SKYSTONE_ON_FOUNDATION[Y], .5);
        sleep(200);

        gridNavigation.driveToPosition(GRAB_FOUNDATION[X], GRAB_FOUNDATION[Y], .29);
        robot.outExtrusion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outExtrusion.setTargetPosition(-500);
        robot.outExtrusion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outExtrusion.setPower(1);

        robot.foundation1.setPosition(.6);
        robot.foundation2.setPosition(.4);

        gridNavigation.driveToPositionBackwards(REPOSITION_FOUNDATION[X], REPOSITION_FOUNDATION[Y], .6);
        robot.outExtrusion.setPower(0);
        robot.claw.setPosition(0);

        robot.outExtrusion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outExtrusion.setTargetPosition(600);
        robot.outExtrusion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outExtrusion.setPower(1);

        /**
         * Turn foundation into building site
         */
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setTargetPosition(1200);
        robot.rightBack.setTargetPosition(-1200);

        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setPower(1);
        robot.rightBack.setPower(1);

        while (robot.rightBack.isBusy() && robot.leftBack.isBusy()) {
        }

        /**
         * Stop running wheels and out extrusion
         */
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.outExtrusion.setPower(0);

        /**
         * Drive forward to push foundation into building site and detach from foundation
         */
        gyroDrive.gyroDriveVariableP(.5,(int)(145.6*1.4),0,.03);
        robot.foundation1.setPosition(1);
        robot.foundation2.setPosition(-1);

        /**
         * Reset grid position
         */
        gridNavigation.setGridPosition(4.87,1.35,0);
        sleep(200);
    }

    private double translateForBlue(double blue) {
        return 6 - blue;
    }

    public double calcTargetX(double[] origPos, double[] newPos, double targetY) {
        int X = 0;
        int Y = 1;


        double deltaX = newPos[X] - origPos[X];
        double deltaY = newPos[Y] - origPos[Y];

        double r2 = newPos[Y] - targetY;

        double r1 = r2 * deltaX / deltaY;

        double targetX = newPos[X] - r1;

//        telemetry.addData("cTX deltaX ", deltaX);
//        telemetry.addData("cTX deltaY ", deltaY);
//        telemetry.addData("cTX r2 ", r2);
//        telemetry.addData("cTX r1 ", r1);
//        telemetry.addData("cTX targetX ", targetX);
//        telemetry.addData("cTX origPos x ", origPos[X]);
//        telemetry.addData("cTX origPos y ", origPos[Y]);
//        telemetry.addData("cTX newPos x ", newPos[X]);
//        telemetry.addData("cTX newPos y ", newPos[Y]);
//        telemetry.update();
//        sleep(3000);


        return targetX;
    }
}