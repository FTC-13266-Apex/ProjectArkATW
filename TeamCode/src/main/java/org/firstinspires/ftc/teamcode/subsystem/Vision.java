package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Vision extends Subsystem {
    public AprilTagDetection tagOfInterest = null;

    private final OpenCvCamera camera;
    private final AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    private static final double fx = 578.272;
    private static final double fy = 578.272;
    private static final double cx = 402.145;
    private static final double cy = 221.506;

    // UNITS ARE METERS
    private static final double tagSize = 0.166;

    // Tag ID 1.2.3 from the 36h11 family
    private static final int LEFT = 1;
    private static final int MIDDLE = 2;
    private static final int RIGHT = 3;



    private boolean tagFound = false;

    public static class Constants {
        public static Hardware hardware = new Hardware();
        public static Value value = new Value();

        public static class Hardware {
            public boolean REVERSED = false;

        }
        public static class Value {

        }
    }

    public Vision(@NonNull OpMode opMode) {
        super(opMode);

        // Obtain camera id to allow for camera preview
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        // Obtain webcam name
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

        // Initialize OpenCvWebcam with live preview
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                opMode.telemetry.addData("OpenCV ran into an error", errorCode);
            }
        });
    }

    @Override
    protected void manualControl() {
        opMode.telemetry.addLine("Um");
        printTagData();
    }

    public void updateTagOfInterest() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        tagFound = false;
        if (currentDetections.size() == 0) return;

        for(AprilTagDetection tag : currentDetections) {
            if(tag.id == LEFT || tag.id == MIDDLE || tag.id ==RIGHT) {
                tagFound = true;
                tagOfInterest = tag;
            }
        }
    }

    public AprilTagDetection getTagOfInterest() {
        return tagOfInterest;
    }



    public void printTagData() {
        if (tagOfInterest == null) {
            opMode.telemetry.addLine("Tag not found");
            return;
        }

        if (tagFound) opMode.telemetry.addLine("Tag in sight");
        else opMode.telemetry.addLine("Tag seen before but not in sight");

        opMode.telemetry.addLine();

        opMode.telemetry.addData("Detected tag ID", tagOfInterest.id);
        opMode.telemetry.addData("Translation X in meters", tagOfInterest.pose.x);
        opMode.telemetry.addData("Translation Y in meters", tagOfInterest.pose.y);
        opMode.telemetry.addData("Translation Z in meters", tagOfInterest.pose.z);
        opMode.telemetry.addData("Rotation Yaw in degrees", Math.toDegrees(tagOfInterest.pose.yaw));
        opMode.telemetry.addData("Rotation Pitch degrees", Math.toDegrees(tagOfInterest.pose.pitch));
        opMode.telemetry.addData("Rotation Roll degrees", Math.toDegrees(tagOfInterest.pose.roll));
    }


}
