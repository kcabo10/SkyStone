package org.firstinspires.ftc.teamcode;

public class MichaelGyroTurnTest_Test {

    /**
     * If you get a NullPointerException originating in the runOpMode() method like this:
     * <p>
     * java.lang.NullPointerException
     * at org.firstinspires.ftc.teamcode.MichaelGyroTurnTest2.runOpMode(MichaelGyroTurnTest2.java:34)
     * <p>
     * then you need to initialize the robot hardware in the class by
     * (1) adding/instantiating the HardwarePushBot class to your class and
     * (2) calling the init method on the HardwarePushBot class like this:
     * <p>
     * HardwarePushbot robot   = new HardwarePushbot();
     * robot.init(hardwareMap);
     * <p>
     * (3) Add the robot hardware configuration to the phone (left_drive, right_drive, etc.).
     * <p>
     * <p>
     * <p>
     * For the following 4 methods, if you get a NullPointerException like the one below:
     * <p>
     * java.lang.NullPointerException
     * at org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot.init(HardwarePushbot.java:81)
     * <p>
     * This is correct.
     * <p>
     * The NullPointerException in this case is due to the hardware on the robot not
     * connected to this test class.  When you connect the robot to the phone and configure the
     * hardware (motors) on the phone, your program should run correctly.
     * <p>
     * <p>
     * Sean
     */


    LibraryGridNavigation testGridNav = new LibraryGridNavigation();

    public static void main(String[] args) {

        try {
            MichaelGyroTurnTest_Test obj = new MichaelGyroTurnTest_Test();
            obj.run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    private void run() throws Exception {


        int X = 0;
        int Y = 1;
        double[] DEPOT_POS = {.5, .4}; /* END_ANGLE = 0 */
        double[] SKYSTONE_POS_1 = {.5, 1.5};
        double[] GRABSKYSTONE_POS_1 = {.3, 1.8};
        double[] BACKING_UP = {.5, .2};
        double[] DRIVE_FORWARD = {.5, .5};
        double[] DELIVER_SKYSTONE = {4, .5};

        testGridNav.setGridPosition(1.5,.5,270);
        testGridNav.strafeToPositionBackwardsValuesOnly(DEPOT_POS[X], DEPOT_POS[Y], .5, 1);
        testGridNav.driveToPositionBackwardsValuesOnly(SKYSTONE_POS_1[X], SKYSTONE_POS_1[Y],.2);
        //testGridNav.driveToPositionBackwardsValuesOnly(GRABSKYSTONE_POS_1[X], GRABSKYSTONE_POS_1[Y], .2);
        //testGridNav.driveToPositionValuesOnly(BACKING_UP[X], BACKING_UP[Y],.5);
        //testGridNav.strafeToPositionBackwardsValuesOnly(DELIVER_SKYSTONE[X], DELIVER_SKYSTONE[Y], .5,0);

//        testGridNav.setGridPosition(.9, .9, 180);
//        testGridNav.driveToPositionValuesOnly(.1, 2.3, .7);
//
//        testGridNav.setGridPosition(.1, 2.3, 119);
//        testGridNav.driveToPositionValuesOnly(RED_CRATER_MARKER[0], RED_CRATER_MARKER[1], .7);


//        gridNavigation.driveToPosition(RED_CRATER_MARKER[X], RED_CRATER_MARKER[Y], .7);
//
//        robot.marker.setPosition(90);
//        robot.marker.setPosition(0);
//        gridNavigation.driveToPositionBackwards(RED_CRATER_PARKING[X], RED_CRATER_PARKING[Y], .7);


    }

    private void michaelGyroTurnTest() throws Exception {
        run();

    }

//    private void michaelGyroTurnTest2() throws Exception {
//        michaelGyroTurnTest2.runOpMode();
//    }
//
//    private void michaelGyroTurnTest3() throws Exception {
//        michaelGyroTurnTest3.runOpMode();
//    }
//
//    private void michaelRandomTesting() throws Exception {
//        michaelRandomTesting.runOpMode();
//    }

}