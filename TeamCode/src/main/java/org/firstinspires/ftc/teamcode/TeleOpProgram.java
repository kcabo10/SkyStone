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
    private HardwareBeepTest robot = new HardwareBeepTest();
    // Setting value to track whether the Y and A buttons are pressed to zero which is not pressed.
    private int buttonYPressed = 0;
    private int buttonAPressed = 0;
    // Setting initial direction to forward.
    private int direction = -1;
    // Setting scaling to full speed.
    private double scaleFactor = 0.5;
    private double scaleTurningSpeed = 1;
    private ElapsedTime foundationtime = new ElapsedTime();
    public ElapsedTime clawaidruntime = new ElapsedTime();
    private int foundation_state = 0;
    private int claw_state = 0;
    private int up_extrusion_state = 0;
    private int clawAid_state = 0;


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
        robot.outExtrusion.setPower(0);
        robot.droidLifterLeft.setPower(0);
        robot.droidLifterRight.setPower(0);
        robot.leftIntake.setPower(0);
        robot.rightIntake.setPower(0);

    }

    /**
     * This method is the main body of our code which contains the code for each of the features on our robot used in teleOp
     */
    public void loop() {

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
            robot.leftIntake.setPower(1);
            robot.rightIntake.setPower(1);
        } else if (gamepad2.left_bumper) {
            robot.leftIntake.setPower(-1);
            robot.rightIntake.setPower(-1);
        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }

        if (gamepad2.b) {
            robot.claw.setPosition(0);
            clawaidruntime.reset();
            while (clawaidruntime.milliseconds() > 250){
                //sleep
            }
            robot.clawAid.setPosition(1);
            clawaidruntime.reset();
            while (clawaidruntime.milliseconds() > 250){
                //sleep
            }
            robot.claw.setPosition(1);
            clawaidruntime.reset();
            while (clawaidruntime.milliseconds() > 250){
                //sleep
            }
            robot.clawAid.setPosition(0);
            clawAid_state++;
        }

        switch (claw_state) {
            case (0):
                if (gamepad2.x) {
                    robot.claw.setPosition(0);
                    robot.clawAid.setPosition(1);
                    claw_state++;
                }
                break;
            case (1):
                if (robot.claw.getPosition() <= 0.1 && !gamepad2.x) {
                    claw_state++;
                }
                break;
            case (2):
                if (gamepad2.x) {
                    robot.claw.setPosition(1);
                    claw_state++;
                }
                break;
            case (3):
                if (robot.claw.getPosition() >= .9 && !gamepad2.x) {
                    claw_state = 0;
                }
                break;
        }

        switch (clawAid_state) {
            case (0):
                if (gamepad2.b) {
                    robot.claw.setPosition(1);
                    //sleep
                    robot.clawAid.setPosition(0);
                    //sleep
                    robot.claw.setPosition(0);
                    //sleep
                    robot.clawAid.setPosition(1);
                    clawAid_state++;
                }
                break;
            case (1):
                if (robot.clawAid.getPosition() <= 0.1 && !gamepad2.b) {
                    clawAid_state++;
                }
                break;
            case (2):
                if (gamepad2.b) {
                    robot.clawAid.setPosition(1);
                    clawAid_state++;
                }
                break;
            case (3):
                if (robot.clawAid.getPosition() >= .9 && !gamepad2.b) {
                    clawAid_state = 0;
                }
                break;
        }

        switch (foundation_state) {
            case (0):
                if (gamepad2.y) {
                    robot.foundation1.setPosition(0);
                    robot.foundation2.setPosition(1);
                    foundation_state++;
                }
                break;
            case (1):
                if (robot.foundation1.getPosition() <= 0.1 && robot.foundation2.getPosition() >= .9 && !gamepad2.y) {
                    foundation_state++;
                }
                break;
            case (2):
                if (gamepad2.y) {
                    robot.foundation1.setPosition(1);
                    robot.foundation2.setPosition(0);
                    foundation_state++;
                }
                break;
            case (3):
                if (robot.foundation1.getPosition() >= .9 && robot.foundation2.getPosition() <= 0.1 && !gamepad2.y) {
                    foundation_state = 0;
                }
                break;
        }


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

        switch (up_extrusion_state) {
            case 0:
                // when the right bumper is being pressed and the touch sensor is not being set the armExtrusion power is set to 1 and the basket position is set to .5.
                // Once those conditions are met the state advances to the next case.
                // When the right trigger is being pressed the armExtrusion power is set to -1 and the basket position is set to .4.
                // Otherwise the power is set to 0.
                if (gamepad2.left_trigger > 0) {
                    robot.droidLifterRight.setPower(1);
                    robot.droidLifterLeft.setPower(-1);
                    up_extrusion_state++;

                } else if (gamepad2.right_trigger > 0) {
                    robot.droidLifterRight.setPower(-.5);
                    robot.droidLifterLeft.setPower(.5);
                } else {
                    robot.droidLifterRight.setPower(0);
                    robot.droidLifterLeft.setPower(0);
                }
                break;
            case 1:
                // This case sets the power to zero once the right bumper has been released and returns to state zero.
                // This allows the drivers to terminate the movement of the arm to avoid damage to the robot.
                if (gamepad2.left_trigger <= 0) {
                    robot.droidLifterRight.setPower(0);
                    robot.droidLifterLeft.setPower(0);
                    up_extrusion_state = 0;
                }
                break;
        }

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
        telemetry.addData("right intake", robot.rightIntake.getPower());
        telemetry.addData("foundation_state", foundation_state);
        telemetry.addData("claw_state", claw_state);
        telemetry.addData("foundation1 position", robot.foundation1.getPosition());
        telemetry.addData("foundation2 position", robot.foundation2.getPosition());
        telemetry.addData("claw position", robot.claw.getPosition());
        telemetry.addData("claw turner position", robot.clawTurner.getPosition());
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