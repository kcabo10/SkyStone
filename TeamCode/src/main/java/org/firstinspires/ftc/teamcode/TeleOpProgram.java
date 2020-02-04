package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.sensors.LibraryColorSensor;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our main teleOp program which controls the robot during the driver controlled period.
 */
@TeleOp(name = "TeleOp Program", group = "TankDrive")
public class TeleOpProgram extends OpMode {

    // Calling hardware map.
    private HardwareBeep robot = new HardwareBeep();
    // Setting value to track whether the Y and A buttons are pressed to zero which is not pressed.
    private int buttonYPressed = 0;
    private int buttonAPressed = 0;
    // Setting initial direction to forward.
    private int direction = -1;
    // Setting scaling to full speed.
    private double scaleFactor = 1;
    private double scaleTurningSpeed = 1;
    private ElapsedTime foundationtime = new ElapsedTime();
    public ElapsedTime danceTime = new ElapsedTime();
    public ElapsedTime clawruntime = new ElapsedTime();
    public ElapsedTime capstonetime = new ElapsedTime();
    private int foundation_state = 0;
    private int buttonXPressed = 0;
    private int up_extrusion_state = 0;
    private int clawAid_state = 0;
    private int clawAlone_state = 0;
    private int capstone_pos = 0;
    private int capstone_state = 0;
    private int dance_state = 0;
    private boolean danceDoOver = false;

    LibraryColorSensor stoneColorSensor = new LibraryColorSensor();
    /**
     * This method reverses the direction of the mecanum drive.
     */
    private void reverseDirection() {
        if (direction == 1) {
            direction = -1;
        } else if (direction == -1) {
            direction = 1;
        }
    }

//    /**
//     * This method scales the speed of the robot to .5.
//     */
//    private void scaleFactor() {
//        if (scaleFactor == 0.5) {
//            scaleFactor = 1;
//        } else if (scaleFactor == 1) {
//            scaleFactor = 0.5;
//        }
//    }

    /**
     * This method initializes hardware map.
     */
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
    }

    /**
     * This method sets motor power to zero
     */
    public void init_loop() {
        //robot.leftIntake.setPower(0);
        //robot.rightIntake.setPower(0);
        robot.outExtrusion.setPower(0);
        robot.droidLifterLeft.setPower(0);
        robot.droidLifterRight.setPower(0);
        //robot.leftIntake.setPower(0);
        //robot.rightIntake.setPower(0);

    }

    /**
     * This method is the main body of our code which contains the code for each of the features on our robot used in teleOp
     */
    public void loop() {

        /**
         * Driving Controls
         */
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;

        // When the direction value is reversed this if statement inverts the addition and subtraction for turning.
        // Default mode: The robot starts with the scaleTurningSpeed set to 1, scaleFactor set to 1, and direction set to forward.
        if (direction == 1) {
            final double v1 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v2 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v3 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v4 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;

            robot.leftFront.setPower(v1);
            robot.rightFront.setPower(v2);
            robot.leftBack.setPower(v3);
            robot.rightBack.setPower(v4);
        } else {
            final double v1 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v2 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v3 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v4 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;

            robot.leftFront.setPower(v1);
            robot.rightFront.setPower(v2);
            robot.leftBack.setPower(v3);
            robot.rightBack.setPower(v4);
        }


        /**
         * Just Claw Controls
         */
        // When the y button has been pressed and released the direction is reversed.
        switch (buttonXPressed) {
            case (0):
                if (gamepad2.x) {
                    buttonXPressed = 1;
                    robot.claw.setPosition(0);
                }
                break;
            case (1):
                if (!gamepad2.x) {
                    buttonXPressed = 0;
                }
                break;
        }


        /**
         * Intake Controls
         */

        if (gamepad2.right_bumper) {
            robot.leftIntake.setPower(-1);
            robot.rightIntake.setPower(1);
        } else if (gamepad2.left_bumper) {
            robot.leftIntake.setPower(1);
            robot.rightIntake.setPower(-1);
        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }


        /**
         * Capstone Controls
         */

        switch (capstone_state) {
            case (0):
                if (gamepad2.a) {
                    capstone_state = 1;
                    robot.capstone.setPosition(capstone_pos);
                }
                break;
            case (1):
                if (!gamepad2.a) {
                    capstone_state = 0;
                    if (capstone_pos == 0) {
                        capstone_pos = 1;
                    }
                    else {
                        capstone_pos = 0;
                    }
                }
                break;
        }

        /**
         * Dance Controls
         */

        switch (dance_state) {
            case (0): //opening claw & setting claw turner back to original pos
                if ((gamepad2.b && !gamepad2.start) || (stoneColorSensor.readSaturation(robot, "sensor_color_dance") >= 0.75 || danceDoOver)) {
                    robot.claw.setPosition(0); //open claw
                    robot.clawTurner.setPosition(1); //ensure claw turner is straight
                    dance_state++;
                    danceTime.reset();
                    danceDoOver = false;
                }
                break;
            case (1): //claw aid pushing brick into claw
                if (danceTime.seconds() > 1) { //wait 1 second
                    robot.clawAid.setPosition(1); //move the claw aid up
                   dance_state++;
                   danceTime.reset();
                }
                break;
            case (2): //claw closing
                if (danceTime.seconds() > 1) { //wait 1 second
                    robot.claw.setPosition(1); //close claw
                    robot.clawAid.setPosition(0); //reset claw aid
                    dance_state++;
                    danceTime.reset();
                }
                break;
            case (3): //claw aid moving back to original pos
                if (danceTime.seconds() > 1 && ((stoneColorSensor.readSaturation(robot, "sensor_color_dance") < .2))) {

                    dance_state = 0;
                }

                if (gamepad2.b) // If b is pressed go back to state 0 to re adjust stone in robot
                {
                    dance_state = 0;
                    danceDoOver = true;
                }

                break;
            }

        /**
         * Foundation Hook Controls
         */

        switch (foundation_state) {
            case (0):
                if (gamepad2.y) {
                    robot.foundation1.setPosition(.5);
                    robot.foundation2.setPosition(.5);
                    foundation_state++;
                }
                break;
            case (1):
                if (!gamepad2.y) {
                    foundation_state++;
                }
                break;
            case (2):
                if (gamepad2.y) {
                    robot.foundation1.setPosition(-1);
                    robot.foundation2.setPosition(1);
                    foundation_state++;
                }
                break;
            case (3):
                if (!gamepad2.y) {
                    foundation_state = 0;
                }
                break;
        }


        /**
         * Out Extrusion Controls
         */

        if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            robot.outExtrusion.setPower(1);
            telemetry.addData("down dpad pressed", gamepad2.dpad_down);
            telemetry.update();
        } else if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.outExtrusion.setPower(-1);
            telemetry.addData("up dpad pressed", gamepad2.dpad_up);
            telemetry.update();
        } else {
            robot.outExtrusion.setPower(0);
        }

        /**
         * Claw Turner Controls
         */

        if(gamepad2.dpad_right && !gamepad2.dpad_left) {
            robot.clawTurner.setPosition(1.0);
            telemetry.addData("right dpad pressed", gamepad2.dpad_right);
            telemetry.update();
        } else if (gamepad2.dpad_left && !gamepad2.dpad_right){
                robot.clawTurner.setPosition(0);
                telemetry.addData("left dpad pressed", gamepad2.dpad_left);
                telemetry.update();
        } else {
            }


        /**
         * Claw Turner Controls
         */
        robot.droidLifterLeft.setPower(gamepad2.left_stick_y);
        robot.droidLifterRight.setPower(-gamepad2.left_stick_y);

//        switch (up_extrusion_state) {
//            case 0:
//                if (gamepad2.left_trigger > 0) {
//                    robot.droidLifterRight.setPower(-1);
//                    robot.droidLifterLeft.setPower(1);
//                    up_extrusion_state++;
//
//                } else if (gamepad2.right_trigger > 0) {
//                    robot.droidLifterRight.setPower(.5);
//                    robot.droidLifterLeft.setPower(-.5);
//                } else {
//                    robot.droidLifterRight.setPower(0);
//                    robot.droidLifterLeft.setPower(0);
//                }
//                break;
//            case 1:
//                // This case sets the power to zero once the right bumper has been released and returns to state zero.
//                // This allows the drivers to terminate the movement of the arm to avoid damage to the robot.
//                if (gamepad2.left_trigger <= 0) {
//                    robot.droidLifterRight.setPower(0);
//                    robot.droidLifterLeft.setPower(0);
//                    up_extrusion_state = 0;
//                }
//                break;
//        }

//        if (gamepad2.right_trigger == 1) {
//            robot.droidLifterLeft.setPower(1);
//            robot.droidLifterRight.setPower(-1);
//            telemetry.addData("right dpad pressed", gamepad2.dpad_right);
//            telemetry.update();
//        } else if (gamepad2.right_trigger == 0) {
//            robot.droidLifterLeft.setPower(0);
//            robot.droidLifterRight.setPower(0);
//        }
//
//        if (gamepad2.left_trigger == 1) {
//            robot.droidLifterLeft.setPower(-.25);
//            robot.droidLifterRight.setPower(.25);
//            telemetry.addData("left dpad pressed", gamepad2.dpad_left);
//            telemetry.update();
//        } else if (gamepad2.left_trigger == 0) {
//            robot.droidLifterLeft.setPower(0);
//            robot.droidLifterRight.setPower(0);
//        }

        // Telemetry
        telemetry.addData("Scale Factor", scaleFactor);
        telemetry.addData("Direction", direction);
        telemetry.addData("left front power", robot.leftFront.getPower());
        telemetry.addData("left back power", robot.leftBack.getPower());
        telemetry.addData("right front power", robot.rightFront.getPower());
        telemetry.addData("right back power", robot.rightBack.getPower());
        telemetry.addData("rb encoder ticks", robot.rightBack.getCurrentPosition());
        telemetry.addData("rf encoder ticks", robot.rightFront.getCurrentPosition());
        telemetry.addData("lb encoder ticks", robot.leftBack.getCurrentPosition());
        telemetry.addData("lf encoder ticks", robot.leftFront.getCurrentPosition());
        telemetry.addData("gyro angle", robot.imuActual.getAngularOrientation().firstAngle);
        telemetry.addData("right intake", robot.rightIntake.getPower());
        telemetry.addData("foundation_state", foundation_state);
        telemetry.addData("buttonXPressed", buttonXPressed);
        telemetry.addData("clawAid_state", clawAid_state);
        //telemetry.addData("claw_aid Pos", robot.clawAid.getPosition());
        telemetry.addData("foundation1 position", robot.foundation1.getPosition());
        telemetry.addData("foundation2 position", robot.foundation2.getPosition());
        telemetry.addData("claw position", robot.claw.getPosition());
        telemetry.addData("claw turner position", robot.clawTurner.getPosition());
        telemetry.addData("foundation1 servo pos", robot.foundation1.getPosition());
        telemetry.addData("foundation2 servo pos", robot.foundation2.getPosition());
        telemetry.addData("foundation state", foundation_state);
        telemetry.addData("capstone state", capstone_state);
        telemetry.addData("capstone pos", capstone_pos);
        //telemetry.addData("gamepad2.a", gamepad2.a);
        telemetry.addData("dance state", dance_state);
        telemetry.addData("color sensor dance", stoneColorSensor.readSaturation(robot, "sensor_color_dance"));
        telemetry.addData("droid_left", robot.droidLifterLeft.getPower());
        telemetry.addData("droid_right", robot.droidLifterRight.getPower());



        telemetry.update();
    }

    /**
     * This method sets the buttons to not being pressed, sets the motor power to zero, and terminates the program.
     */
    public void stop() {

        buttonYPressed = 0;
        buttonAPressed = 0;
        robot.leftIntake.setPower(0);
        robot.rightIntake.setPower(0);
        robot.outExtrusion.setPower(0);
        robot.droidLifterLeft.setPower(0);
        robot.droidLifterRight.setPower(0);
        robot.leftIntake.setPower(0);
        robot.rightIntake.setPower(0);


    }
}