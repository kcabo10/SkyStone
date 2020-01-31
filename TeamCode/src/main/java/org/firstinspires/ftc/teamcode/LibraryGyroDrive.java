package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * This is our custom library that we use to drive forward. The point of using a gyro sensor is to
 * drive straight using it reaches a certain position. If the robot runs over an object or hits
 * something it will take the error of the robot and correct it to get back on track and continue
 * driving forward
 */
public class LibraryGyroDrive {

    // This value is to read the robots current heading
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    // This P Coefficient is the value that corrects the robot in relation to the error of the robots
    // current heading

    // Kc = .05
    // Pc = 1
    // Dt = 0.01

    // Kp = .03
    // Ki = .0006
    // Kd = .01
    static double PCoeff = 0.03;     // Larger is more responsive, but also less stable
    static double ICoeff = .0006;
    static double DCoeff = .01;
    // a constant speed
    public double speed = .6;
    // Calls the Hardware map
    HardwareBeep robot = new HardwareBeep();
    // Calls Library gyro to access the gyro sensor
    LibraryGyro gyro = new LibraryGyro();
    Telemetry telemetry;
    DcMotor motor;

    Boolean PRINT_TELEMETRY = true;

    /**
     * The hardware class needs to be initialized before this method is called
     */
    public void init(HardwareBeep myRobot, Telemetry myTelemetry, DcMotor myMotor) {
        robot = myRobot;
        telemetry = myTelemetry;
        gyro.init(robot, telemetry);
        motor = myMotor;

    }
    public void gyroStrafeRight (double speed,
                                 int encoderTicks,
                                 double angle) {

        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;
        double error;
        double steer;
        double leftFrontSpeed;
        double leftBackSpeed;
        double rightFrontSpeed;
        double rightBackSpeed;
        double integral = 0;
        double derivative = 0;
        double lastError = 0;

        if (PRINT_TELEMETRY){
            telemetry.addData("In gyroStrafeRight method", "");
            telemetry.update();
        }

        // In order to use the encoders you need to follow a specific pattern. The first step is to
        // stop and reset the encoders
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // The next step is to set the encoders to run
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = encoderTicks;
        newLeftBackTarget = -encoderTicks;
        newRightFrontTarget = -encoderTicks;
        newRightBackTarget = encoderTicks;

        // Set Target and Turn On RUN_TO_POSITION
        robot.leftFront.setTargetPosition(newLeftFrontTarget);
        robot.leftBack.setTargetPosition(newLeftBackTarget);
        robot.rightFront.setTargetPosition(newRightFrontTarget);
        robot.rightBack.setTargetPosition(newRightBackTarget);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set a range clip for the speed to ensure that the robot doesn't drive faster or slower
        // than the clip
        speed = Range.clip(speed, -1, 1);
        // set all the motors to the .6 power we declared in the beginning of the program
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);

        double remainingTicksAvg;

        // keep looping while motors are still active, and BOTH motors are running.
        do
        {
            remainingTicksAvg =
                    ((Math.abs(robot.leftFront.getTargetPosition())-
                            Math.abs(robot.leftFront.getCurrentPosition()))+
                            (Math.abs(robot.leftBack.getTargetPosition())-
                                    Math.abs(robot.leftBack.getCurrentPosition()))+
                            (Math.abs(robot.rightBack.getTargetPosition())-
                                    Math.abs(robot.rightBack.getCurrentPosition()))+
                            (Math.abs(robot.rightFront.getTargetPosition())-
                                    Math.abs(robot.rightFront.getCurrentPosition())))/4;
            // adjust relative speed based on heading error.
            error = getError(angle);
            integral += error;
            derivative = error - lastError;
            lastError = error;

            steer = getSteer(error, PCoeff, integral, ICoeff, derivative, DCoeff);

            // read the steer to determine the speed for the motors on both sides of the robot
            leftFrontSpeed = speed - steer;
            leftBackSpeed = speed + steer;
            rightFrontSpeed = speed + steer;
            rightBackSpeed = speed - steer;

            // set speed clip for the motors on both sides of the robot
//            leftFrontSpeed = Range.clip(leftFrontSpeed, -.5, .5);
//            leftBackSpeed = Range.clip(leftBackSpeed, -.5, .5);
//            rightFrontSpeed = Range.clip(rightFrontSpeed, -.5, .5);
//            rightBackSpeed = Range.clip(rightBackSpeed, -.5, .5);

            // setting speeds
            robot.leftFront.setPower(leftFrontSpeed);
            robot.leftBack.setPower(leftBackSpeed);
            robot.rightFront.setPower(rightFrontSpeed);
            robot.rightBack.setPower(rightBackSpeed);

            //telemetry
            telemetry.addData("gSR: Error", error);
            telemetry.addData("Steer", steer);
            telemetry.addData("L front Speed", leftFrontSpeed);
            telemetry.addData("L back Speed", leftBackSpeed);
            telemetry.addData("R front Speed", rightFrontSpeed);
            telemetry.addData("R back Speed", rightBackSpeed);
            telemetry.addData("PCoeff", PCoeff);
            telemetry.addData("ICoeff", ICoeff);
            telemetry.addData("DCoeff", DCoeff);
            telemetry.update();

        }
        while ((robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy()
                && robot.rightBack.isBusy()) &&
                remainingTicksAvg > 10);

        // Stop all motion;
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void gyroStrafeLeft (double speed,
                                int encoderTicks,
                                double angle) {

        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;
        double error;
        double steer;
        double leftFrontSpeed;
        double leftBackSpeed;
        double rightFrontSpeed;
        double rightBackSpeed;
        double integral = 0;
        double derivative = 0;
        double lastError = 0;

        telemetry.addData("In gyroStrafeLeft method", "");
        telemetry.update();

        // In order to use the encoders you need to follow a specific pattern. The first step is to
        // stop and reset the encoders
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // The next step is to set the encoders to run
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = -encoderTicks;
        newLeftBackTarget = encoderTicks;
        newRightFrontTarget = encoderTicks;
        newRightBackTarget = -encoderTicks;

        // Set Target and Turn On RUN_TO_POSITION
        robot.leftFront.setTargetPosition(newLeftFrontTarget);
        robot.leftBack.setTargetPosition(newLeftBackTarget);
        robot.rightFront.setTargetPosition(newRightFrontTarget);
        robot.rightBack.setTargetPosition(newRightBackTarget);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set a range clip for the speed to ensure that the robot doesn't drive faster or slower
        // than the clip
        speed = Range.clip(speed, -1, 1);
        // set all the motors to the .6 power we declared in the beginning of the program
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);

        double remainingTicksAvg;

        // keep looping while motors are still active, and BOTH motors are running.
        do
        {
            remainingTicksAvg =
                    ((Math.abs(robot.leftFront.getTargetPosition())-
                            Math.abs(robot.leftFront.getCurrentPosition()))+
                            (Math.abs(robot.leftBack.getTargetPosition())-
                                    Math.abs(robot.leftBack.getCurrentPosition()))+
                            (Math.abs(robot.rightBack.getTargetPosition())-
                                    Math.abs(robot.rightBack.getCurrentPosition()))+
                            (Math.abs(robot.rightFront.getTargetPosition())-
                                    Math.abs(robot.rightFront.getCurrentPosition())))/4;

            // adjust relative speed based on heading error.
            error = getError(angle);
            integral += error;
            derivative = error - lastError;
            lastError = error;

            steer = getSteer(error, PCoeff, integral, ICoeff, derivative, DCoeff);

            // read the steer to determine the speed for the motors on both sides of the robot
            leftFrontSpeed = speed + steer;
            leftBackSpeed = speed - steer;
            rightFrontSpeed = speed - steer;
            rightBackSpeed = speed + steer;

            // set speed clip for the motors on both sides of the robot
//            leftFrontSpeed = Range.clip(leftFrontSpeed, -.5, .5);
//            leftBackSpeed = Range.clip(leftBackSpeed, -.5, .5);
//            rightFrontSpeed = Range.clip(rightFrontSpeed, -.5, .5);
//            rightBackSpeed = Range.clip(rightBackSpeed, -.5, .5);

            // setting speeds
            robot.leftFront.setPower(leftFrontSpeed);
            robot.leftBack.setPower(leftBackSpeed);
            robot.rightFront.setPower(rightFrontSpeed);
            robot.rightBack.setPower(rightBackSpeed);

            //telemetry
            telemetry.addData("gSL: Error", error);
            telemetry.addData("Steer", steer);
            telemetry.addData("L front Speed", leftFrontSpeed);
            telemetry.addData("L back Speed", leftBackSpeed);
            telemetry.addData("R front Speed", rightFrontSpeed);
            telemetry.addData("R back Speed", rightBackSpeed);
            telemetry.addData("PCoeff", PCoeff);
            telemetry.addData("ICoeff", ICoeff);
            telemetry.addData("DCoeff", DCoeff);
            telemetry.update();

        }
        while ((robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy()
                && robot.rightBack.isBusy()) &&
                remainingTicksAvg > 10);
        // Stop all motion;
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void gyroDriveVariableP(double speed,
                                   int encoderTicks,
                                   double angle, double PCoeff) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double leftSpeedRamp = 0;
        double rightSpeedRamp = 0;
        double INTERVAL = 0.07;
        double integral = 0;
        double derivative = 0;
        double lastError = 0;

        telemetry.addData("In Gyro Drive method", "");
        telemetry.update();

        gyro.resetAngle();


        // In order to use the encoders you need to follow a specific pattern. The first step is to
        // stop and reset the encoders
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Then we set the encoders to run
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position, and pass to motor controller
        newLeftTarget = encoderTicks;
        newLeftTarget = encoderTicks;
        newRightTarget = encoderTicks;
        newRightTarget = encoderTicks;

        // Set Target and Turn On RUN_TO_POSITION
        robot.leftFront.setTargetPosition(newLeftTarget);
        robot.leftBack.setTargetPosition(newLeftTarget);
        robot.rightFront.setTargetPosition(newRightTarget);
        robot.rightBack.setTargetPosition(newRightTarget);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double remainingTicksAvg;
        boolean rampReached = false;
        boolean rampDnReached = false;

        // keep looping while motors are still active, and BOTH motors are running.
        do {

            remainingTicksAvg =
                    ((Math.abs(robot.leftFront.getTargetPosition())-
                            Math.abs(robot.leftFront.getCurrentPosition()))+
                            (Math.abs(robot.leftBack.getTargetPosition())-
                                    Math.abs(robot.leftBack.getCurrentPosition()))+
                            (Math.abs(robot.rightBack.getTargetPosition())-
                                    Math.abs(robot.rightBack.getCurrentPosition()))+
                            (Math.abs(robot.rightFront.getTargetPosition())-
                                    Math.abs(robot.rightFront.getCurrentPosition())))/4;

            telemetry.addData("libgyrodrVP: target angle", angle);
            telemetry.addData("libgyrodrVP: gyro angle", gyro.getAngle());
            // adjust relative speed based on heading error.
            error = getError(angle);
            integral += error;
            derivative = error - lastError;
            lastError = error;

            steer = getSteer(error, PCoeff, integral, ICoeff, derivative, DCoeff);

            // if driving in reverse, the motor correction also needs to be reversed
            if (encoderTicks < 0)
                steer *= -1.0;


            //INVERTED THIS 1/31 12:20am
            leftSpeed = speed + steer;
            rightSpeed = speed - steer;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

//            // Ramup up the speed until speed is reached;
//            if (!rampReached) {
//
//                if (leftSpeedRamp < leftSpeed) {
//                    leftSpeedRamp = leftSpeedRamp + INTERVAL;
//                    telemetry.addData("LeftRampSpeed", leftSpeedRamp);
//                }
//                else
//                    rampReached = true;
//
//                if (rightSpeedRamp < rightSpeed) {
//                    rightSpeedRamp += INTERVAL;
//                    telemetry.addData("RightRampSpeed", rightSpeedRamp);
//                }
//                else
//                    rampReached = true;
//
//                leftSpeed = leftSpeedRamp;
//                rightSpeed = rightSpeedRamp;
//            }
//
//            if ((remainingTicksAvg < (newLeftTarget*.1)) && !rampDnReached && rampReached)
//            {
//                if (leftSpeedRamp < .3) {
//                    leftSpeedRamp -= INTERVAL;
//                    telemetry.addData("LeftRampDnSpeed", leftSpeedRamp);
//                }
//                else
//                    rampDnReached = true;
//
//                if (rightSpeedRamp < .3) {
//                    rightSpeedRamp -= INTERVAL;
//                    telemetry.addData("RightRampDnSpeed", rightSpeedRamp);
//                }
//                else
//                    rampDnReached = true;
//
//                leftSpeed = leftSpeedRamp;
//                rightSpeed = rightSpeedRamp;
//            }

            robot.leftFront.setPower(leftSpeed);
            robot.leftBack.setPower(leftSpeed);
            robot.rightFront.setPower(rightSpeed);
            robot.rightBack.setPower(rightSpeed);

            // Display drive status for the driver.
            telemetry.addData("Error", error);
            telemetry.addData("Steer", steer);
            telemetry.addData("L Speed", leftSpeed);
            telemetry.addData("R Speed", rightSpeed);
            telemetry.addData("Remaining Ticks Avg", remainingTicksAvg);
            telemetry.addData("rampReached", rampReached);
            telemetry.addData("PCoeff", PCoeff);
            telemetry.addData("ICoeff", ICoeff);
            telemetry.addData("DCoeff", DCoeff);
            telemetry.update();
        }
        while ((robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy()
                && robot.rightBack.isBusy()) &&
                remainingTicksAvg > 10);

        // Stop all motion;
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    /**
     * This method calculates the error and then determines the speed and direction necessary for the robot to correct in order to drive straight
     *
     * @param speed  Input your desired speed.
     * @param angle  This is the angle the method is fed by the readings to determine the error
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;
        double integral = 0;
        double derivative = 0;
        double lastError = 0;

        // determine turn power based on +/- error
        error = getError(angle);

        // if the error is less than or equal to the target heading don't change the motor speeds
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
            // else if the error is greater then the target heading than we get the steer by taking
            // the error and P Coefficient
        } else {
            steer = getSteer(error, PCoeff, integral, ICoeff, derivative, DCoeff);
            // we multiply the speed by the steer and set the left speed to be the opposite of that
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFront.setPower(leftSpeed);
        robot.leftBack.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.rightBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;


    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getAngle();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;

    }

    public boolean setPIDCoeff(double _PCoeff, double _ICoeff, double _DCoeff) {
        PCoeff = _PCoeff;
        ICoeff = _ICoeff;
        DCoeff = _DCoeff;

        return true;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff, double integral, double ICoeff, double derivative, double DCoeff) {

        return Range.clip(((error * PCoeff) + (integral * ICoeff) + (derivative * DCoeff)), -1, 1);
    }

    /**
     * This is the method we use to call gyro drive in the Grid Navigation library when we want to integrate the p coefficient
     *
     * @param speed        When you call the method input the desired speed
     * @param encoderTicks Input the distance you want to drive in encoder ticks
     * @param angle        What angle you would like to drive
     * @param PCoeff       Proportional Gain Coefficient
     */


}