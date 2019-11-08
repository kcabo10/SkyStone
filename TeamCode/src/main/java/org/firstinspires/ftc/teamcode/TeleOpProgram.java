package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public ElapsedTime clawruntime = new ElapsedTime();
    private int foundation_state = 0;
    private int claw_state = 0;
    private int intake_state = 0;



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

    /**
     * This method scales the speed of the robot to .5.
     */
    private void scaleFactor() {
        if (scaleFactor == 0.5) {
            scaleFactor = 1;
        } else if (scaleFactor == 1) {
            scaleFactor = 0.5;
        }
    }

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
        robot.leftIntake.setPower(0);
        robot.rightIntake.setPower(0);
        robot.outExtrusion1.setPower(0);
        robot.outExtrusion2.setPower(0);
        robot.droidLifterLeft.setPower(0);
        robot.droidLifterRight.setPower(0);


    }

    /**
     * This method is the main body of our code which contains the code for each of the features on our robot used in teleOp
     */
    public void loop() {

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

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

        // When the y button has been pressed and released the direction is reversed.
        switch (buttonYPressed) {
            case (0):
                if (gamepad1.y) {
                    buttonYPressed = 1;
                }
                break;
            case (1):
                if (!gamepad1.y) {
                    reverseDirection();
                    buttonYPressed = 0;
                }
                break;
        }

        // When the a button has been pressed and released the speed is scaled to .5 power.
        switch (buttonAPressed) {
            case (0):
                if (gamepad1.a) {
                    buttonAPressed = 1;
                }
                break;
            case (1):
                if (!gamepad1.a) {
                    buttonAPressed = 0;
                    scaleFactor();
                }
                break;
        }

        // When the button below the right joystick is pressed JUST the turning speed power is set to .5.
        if (gamepad1.right_stick_button) {
            scaleTurningSpeed = 0.5;
        } else {
            scaleTurningSpeed = 1;
        }

        // When the game pad 2 a button is pressed set the basket position to 0
        // When the game pad 2 x button is pressed set the basket position to .5.
        // When the game pad 2 b button is pressed set the basket position to .9.

        if (gamepad2.right_bumper) {
            robot.leftIntake.setPower(.5);
            robot.rightIntake.setPower(.5);
        } else if (gamepad2.left_bumper) {
            robot.leftIntake.setPower(-.5);
            robot.rightIntake.setPower(-.5);
        } else if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }

//        if(gamepad2.x){
//            resetStartTime();
//            while(time < 1){
//                robot.claw.setPower(1);
//            }
//            robot.claw.setPower(0);
//        }

        switch (claw_state) {
            case 0:
                if (gamepad2.x) {
                    clawruntime.reset();
                    robot.claw.setPower(-.5);
                    while (clawruntime.seconds() < .2) {
                        telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", clawruntime.seconds());
                        telemetry.update();
                    }
                    claw_state++;

                }
                break;
            case 1:
                if (gamepad2.x) {
                    clawruntime.reset();
                    robot.claw.setPower(.5);
                    while (clawruntime.seconds() < .2) {
                        telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", clawruntime.seconds());
                        telemetry.update();
                    }
                    claw_state++;

                }
                break;
            case 2:
                robot.claw.setPower(0);
                claw_state = 0;
        }

        switch (foundation_state) {
            case (0):
                if (!gamepad2.y) {
                    robot.foundation1.setPosition(0);
                    robot.foundation2.setPosition(0);
                    foundation_state = 0;
                }
                break;
            case (1):
                if (gamepad2.y) {
                    robot.foundation1.setPosition(0.25);
                    robot.foundation2.setPosition(0.25);
                    foundation_state = 1;
                }
                break;
        }


        if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            robot.outExtrusion1.setPower(-1);
            robot.outExtrusion2.setPower(1);
            telemetry.addData("down dpad pressed", gamepad2.dpad_down);
            telemetry.update();
        } else if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.outExtrusion1.setPower(1);
            robot.outExtrusion2.setPower(-1);
            telemetry.addData("up dpad pressed", gamepad2.dpad_up);
            telemetry.update();
        } else {
            robot.outExtrusion1.setPower(0);
            robot.outExtrusion2.setPower(0);
        }

        if(gamepad2.dpad_right && !gamepad2.dpad_left) {
            robot.clawTurner.setPosition(1.0);
            telemetry.addData("right dpad pressed", gamepad2.dpad_right);
            telemetry.update();
        } else if (gamepad2.dpad_left && !gamepad2.dpad_right)    {
                robot.clawTurner.setPosition(-1.0);
                telemetry.addData("left dpad pressed", gamepad2.dpad_left);
                telemetry.update();
        } else {
            }

        if (gamepad2.right_trigger == 1) {
            robot.droidLifterLeft.setPower(1);
            robot.droidLifterRight.setPower(-1);
            telemetry.addData("right dpad pressed", gamepad2.dpad_right);
            telemetry.update();
        } else if (gamepad2.right_trigger == 0) {
            robot.droidLifterLeft.setPower(0);
            robot.droidLifterRight.setPower(0);
        }

        if (gamepad2.left_trigger == 1) {
            robot.droidLifterLeft.setPower(-.25);
            robot.droidLifterRight.setPower(.25);
            telemetry.addData("left dpad pressed", gamepad2.dpad_left);
            telemetry.update();
        } else if (gamepad2.left_trigger == 0) {
            robot.droidLifterLeft.setPower(0);
            robot.droidLifterRight.setPower(0);
        }

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
        telemetry.addData("right intake", robot.rightIntake.getPower());
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
        robot.outExtrusion1.setPower(0);
        robot.outExtrusion2.setPower(0);
        robot.droidLifterLeft.setPower(0);
        robot.droidLifterRight.setPower(0);
        robot.claw.setPower(0);


    }
}