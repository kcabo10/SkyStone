package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Locale;

/*
 * Thanks to EasyOpenCV for the great API (and most of the example)
 *
 * Original Work Copright(c) 2019 OpenFTC Team
 * Derived Work Copyright(c) 2019 DogeDevs
 */
public class LibraryOpenCV {
    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;
    HardwareBeep robot;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    ElapsedTime timer;

    public LibraryOpenCV(HardwareBeep newHardwareBeep, Telemetry newTelemetry) {

        robot = newHardwareBeep;
        telemetry = newTelemetry;
        hardwareMap = robot.hwMap;

    }

    /**
     * This method starts up the phone light and reads the skystone position.
     *
     * @return This return function sends back the skystone position
     */
    public String findSkystone() {

        initOpenCV();

        long startTime = 0;
        String previousPosition = "";
        String SkystonePosition = "";

        // sets start time to read in milliseconds
        startTime = System.currentTimeMillis();

        // sets the OpenCV to read the mineral for at least 3 seconds to verify that it is the
        // correct skystone
        while (System.currentTimeMillis() < (startTime + 2000)) { /**DEBUG CHANGED TO 30000*/

            // sets skystone position values to the read skystone function
            SkystonePosition = readSkystonePos();

            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
            telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();
        }
        phoneCam.stopStreaming();
        return SkystonePosition;

    }

    private void initOpenCV() {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);
        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public String readSkystonePos() {
        String currentPos = "";
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // while mineral position is not found and the timer counts 6 seconds
        while (currentPos == "" && timer.seconds() < 2) { /**DEBUG CHANGED TO 600 */

            // If the skystone is positioned more toward the right of the phones camera view
            // then the x position will increase

            // If the skystone is at a position greater than 100 than we assume the skystone
            // is in position
            if (skyStoneDetector.getScreenPosition().y <= 100) {
                telemetry.addData("Skystone Position", "Pos 1");
                telemetry.update();
                currentPos = "Pos 1";
                //if the skystone is not greater than the stone than
                // it sets the current skystone position as CENTER
            } else if (skyStoneDetector.getScreenPosition().y > 100 && skyStoneDetector.getScreenPosition().x <= 180) {
                telemetry.addData("Skystone Position", "Pos 2");
                currentPos = "Pos 2";
            } else if (skyStoneDetector.getScreenPosition().x > 180 && skyStoneDetector.getScreenPosition().x <= 270) {
                telemetry.addData("Skystone Position", "Pos 3");
                telemetry.update();
                currentPos = "Pos 3";
            }
        }
        // returns the current skystone position
        return currentPos;
    }
}
