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
//@Disabled
@Autonomous(name = "Red Side Skystone Delivery", group = "Beep")
public class RedSkystoneUsingOpenCV extends LinearOpMode {

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

    LibraryColorSensor stoneColorSensor = new LibraryColorSensor();

    LibraryOpenCV opencv;

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

        // Start position for foundation hooks
        robot.foundation1.setPosition(1);
        robot.foundation2.setPosition(-1);

        robot.claw.setPosition(0);

        opencv = new LibraryOpenCV(robot, telemetry);
        opencv.initOpenCV();

        telemetry.addData("OpenCV initialized","");
        telemetry.update();
        sleep(5000);

        while (!isStarted())
        {
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

        double[] SKYSTONE_POS_1 = {1.9, 2.5};
        double[] SKYSTONE_POS_2 = {1.5, 2.5};
        double[] SKYSTONE_POS_3 = {1.1, 2.5};
        double[] BACKING_UP_1 = {1.9, 1.4};
        double[] BACKING_UP_2 = {1.5, 1.4};
        double[] BACKING_UP_3 = {1.1, 1.4};


        double[] SKYSTONE2_POS_1 = {1, 1.4};
        double[] SKYSTONE2_POS_2 = {.5, 1.4};
        double[] SKYSTONE2_POS_3 = {.8, 1.4};
        double[] GRAB_SKYSTONE2_POS_1 = {1, 2.6};
        double[] GRAB_SKYSTONE2_POS_2 = {.5, 2.6};
        double[] GRAB_SKYSTONE2_POS_3 = {.3, 1.7};
        double[] BACKING_UP2_1 = {.9, 1.4};
        double[] BACKING_UP2_2 = {.5, 1.4};
        double[] BACKING_UP2_3 = {.6, 1.4};


        //Same end to each case
        double[] DELIVERING_SKYSTONE = {3.5, 1.4};
        double[] PARKING_POS = {2.9, 1.4};
        double[] STARTING_POS = {1.7, .3645};

        //waitForStart();

        //opencv.openCVIsNotActive();

        gridNavigation.setGridPosition(STARTING_POS[X], STARTING_POS[Y], 270);


        switch (SkystonePosition) {
            /**
             * This first pos is working just need to add second Skystone
             */
            //Right Skystone position
            case "right":

            telemetry.addData("Telemetry", "Skystone Pos = 3/right");
            telemetry.update();

            intakeSkystone();

//            double testX = calcTargetX(STARTING_POS,SKYSTONE_POS_1,1.4);
//            telemetry.addData("Drive to X ", testX);
//            telemetry.addData("Drive to Y ", 1.4);
//            telemetry.update();
//            sleep(1000);

            gridNavigation.driveToPositionBackwards(calcTargetX(STARTING_POS,SKYSTONE_POS_1,1.4), 1.4,.5);
            gridNavigation.driveToPositionBackwards(SKYSTONE_POS_1[X], SKYSTONE_POS_1[Y], .2);
            sleep(500);

            if (stoneColorSensor.readSaturation(robot, "sensor_color_dance") >= .75)
                robot.clawAid.setPosition(1); //move the claw aid up
                robot.clawAid.setPosition(0); //move the claw aid back
                robot.clawAid.setPosition(1); //move the claw aid up again

                robot.claw.setPosition(1); //close claw
                robot.clawAid.setPosition(0); //reset claw aid


            gridNavigation.driveToPosition(calcTargetX(STARTING_POS,SKYSTONE_POS_1,1.4), BACKING_UP_1[Y], .5);

            sleep(500);

            gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y], .5);
            pushOutSkystone();
            sleep(500);

            gridNavigation.driveToPosition(SKYSTONE2_POS_1[X], SKYSTONE2_POS_1[Y], .5);
            intakeSkystone();
            sleep(500);
            gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE2_POS_1[X], GRAB_SKYSTONE2_POS_1[Y], .2);

            gridNavigation.driveToPosition(calcTargetX(SKYSTONE2_POS_1,GRAB_SKYSTONE2_POS_1,1.4), BACKING_UP2_1[Y], .5);
            sleep(500);

            gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y], .5);
            pushOutSkystone();
            sleep(400);

            gridNavigation.driveToPosition(PARKING_POS[X], PARKING_POS[Y], .5);

                break;

            //Middle Skystone position
            case "middle":

                /**
                 * This second pos is working just needs a bit of tuning
                 * We should come at an angle for the second stone in order to save time
                 */

                telemetry.addData("Telemetry", "Skystone Pos = 2/middle");
                telemetry.update();

                intakeSkystone();
                gridNavigation.driveToPositionBackwards(calcTargetX(STARTING_POS,SKYSTONE_POS_2,1.4), 1.4,.5);

                gridNavigation.driveToPositionBackwards(SKYSTONE_POS_2[X], SKYSTONE_POS_2[Y], .2);
                sleep(1000);
                gridNavigation.driveToPosition(calcTargetX(STARTING_POS,SKYSTONE_POS_2,1.4), BACKING_UP_2[Y], .5);
                sleep(500);

                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y], .5);
                pushOutSkystone();
                sleep(500);

                gridNavigation.driveToPosition(SKYSTONE2_POS_2[X], SKYSTONE2_POS_2[Y], .5);
                intakeSkystone();
                gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE2_POS_2[X], GRAB_SKYSTONE2_POS_2[Y], .2);

                gridNavigation.driveToPosition(calcTargetX(SKYSTONE2_POS_2,GRAB_SKYSTONE2_POS_2,1.4), BACKING_UP2_2[Y], .5);
                sleep(500);

                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y], .5);
                pushOutSkystone();
                sleep(1000);

                gridNavigation.driveToPosition(PARKING_POS[X], PARKING_POS[Y], .5);

                break;

            //Position closest to skybridge
            case "left":

                telemetry.addData("Telemetry", "Skystone Pos = 1/left");
                telemetry.update();

                intakeSkystone();
                gridNavigation.driveToPositionBackwards(calcTargetX(STARTING_POS,SKYSTONE_POS_3,1.4), 1.4,.5);

                gridNavigation.driveToPositionBackwards(SKYSTONE_POS_3[X], SKYSTONE_POS_3[Y], .2);
                sleep(1000);
                gridNavigation.driveToPosition(calcTargetX(STARTING_POS,SKYSTONE_POS_3,1.4), BACKING_UP_3[Y], .5);
                sleep(500);
                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y], .5);
                pushOutSkystone();
                sleep(500);

                gridNavigation.driveToPosition(SKYSTONE2_POS_3[X], SKYSTONE2_POS_3[Y], .5);
                intakeSkystone();
                sleep(500);
                gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE2_POS_3[X], GRAB_SKYSTONE2_POS_3[Y], .2);
                sleep(500);
                gridNavigation.driveToPosition(calcTargetX(SKYSTONE2_POS_3,GRAB_SKYSTONE2_POS_3,1.4), BACKING_UP2_3[Y], .5);
                sleep(500);

                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y], .5);
                pushOutSkystone();
                sleep(400);

                gridNavigation.driveToPosition(PARKING_POS[X], PARKING_POS[Y], .5);

                break;

            // should never get to this case but in case it can't find the skystone position
            // it goes to this default case
            default:

                telemetry.addData("Unknown Position", "Playing Pos 3");
                telemetry.update();

                intakeSkystone();
                gridNavigation.driveToPositionBackwards(calcTargetX(STARTING_POS,SKYSTONE_POS_3,1.4), 1.4,.5);

                gridNavigation.driveToPositionBackwards(SKYSTONE_POS_3[X], SKYSTONE_POS_3[Y], .2);
                sleep(1000);
                gridNavigation.driveToPosition(calcTargetX(STARTING_POS,SKYSTONE_POS_3,1.4), BACKING_UP_3[Y], .5);

                sleep(500);

                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y], .5);
                pushOutSkystone();
                sleep(500);

                gridNavigation.driveToPosition(SKYSTONE2_POS_3[X], SKYSTONE2_POS_3[Y], .5);
                intakeSkystone();
                sleep(500);
                gridNavigation.driveToPositionBackwards(GRAB_SKYSTONE2_POS_3[X], GRAB_SKYSTONE2_POS_3[Y], .2);
                sleep(500);
                gridNavigation.driveToPosition(calcTargetX(SKYSTONE2_POS_3,GRAB_SKYSTONE2_POS_3,1.4), BACKING_UP2_3[Y], .5);
                sleep(500);

                gridNavigation.driveToPositionBackwards(DELIVERING_SKYSTONE[X], DELIVERING_SKYSTONE[Y], .5);
                pushOutSkystone();
                sleep(400);

                gridNavigation.driveToPosition(PARKING_POS[X], PARKING_POS[Y], .5);
            break;
        }

    }

        public void intakeSkystone (){
            robot.rightIntake.setPower(.85);
            robot.leftIntake.setPower(-.85);
    }

        public void pushOutSkystone (){
            robot.rightIntake.setPower(-.8);
            robot.leftIntake.setPower(.8);
        }
        public double calcTargetX(double[] origPos, double[] newPos, double targetY){
        int X = 0;
        int Y = 1;


        double deltaX = newPos[X] - origPos[X];
        double deltaY = newPos[Y]  - origPos[Y];

        double r2 = newPos[Y] - targetY;

        double r1 = r2 * deltaX/deltaY;

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