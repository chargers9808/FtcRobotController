package org.firstinspires.ftc.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class VisionBase {

    public double lateralOffset=0;
    public double forwardDistanceToTag=0;
    public boolean targetFound=false;
    public String propLocation;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // The variable to store our instance of the AprilTag processor.
    public AprilTagProcessor aprilTag;

    // The variable to store our instance of the TensorFlow Object Detection processor.
    public TfodProcessor tfod;

    //** The variable to store our instance of the vision portal.
    public VisionPortal myVisionPortal;

    OpMode callingOpMode;
    HardwareMap hwMap = null;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/ChargerCustomModelCubes.tflite";
    private static final String[] LABELS = {
            "blue prop",
            "red prop"
    };

    public int numberOfDetections=0;
    private AprilTagDetection desiredTag = null;

    public void initDoubleVision(HardwareMap ahwMap, OpMode _callingOpMode) {
        hwMap = ahwMap;
        callingOpMode = _callingOpMode;

        // --------AprilTag Configuration-----------------------------------------------------------
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // --------TFOD Configuration---------------------------------------------------------------
        tfod = new TfodProcessor.Builder()
                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();
         tfod.setMinResultConfidence(0.87f);       

        // --------Camera Configuration-------------------------------------------------------------
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(callingOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(tfod, aprilTag)
                .build();
    } // end initDoubleVision()

    public void getDistancesToAprilTag(int tagID) {
        desiredTag  = null;
        //targetFound     = false;    // Set to true when an AprilTag target is detected
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        callingOpMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if (detection.id == tagID) {
                    // this is the tag we want to line up on
                    targetFound = true;
                    desiredTag = detection;
                    forwardDistanceToTag=detection.ftcPose.y;
                    lateralOffset=detection.ftcPose.x;
                    break;  // don't look any further.
                }
            }


            // callingOpMode. telemetry.update();
        }   // end for() loop

    }   // end method getDistancesToAprilTag()
    public void locateOurPropStartingRightEdge() {
        double x = 0;
        double y = 0;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        callingOpMode.telemetry.addData("# Objects Detected", currentRecognitions.size());
        // Step through the list of recognitions and display info for each one.
        if(currentRecognitions.size()==0){propLocation="Not Found";propLocation = "Left";}
        else {
            for (Recognition recognition : currentRecognitions) {
                x = (recognition.getLeft() + recognition.getRight()) / 2;
                y = (recognition.getTop() + recognition.getBottom()) / 2;
                if (x > 250) {propLocation = "Right";}
                else if (x > 5 && x < 250) {propLocation = "Center";}
                else {propLocation = "Left";}
                callingOpMode.telemetry.addData("", " ");
                callingOpMode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                callingOpMode.telemetry.addData("- Position", "%.0f / %.0f", x, y);
                callingOpMode.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }// end of the for() loop
        }
        callingOpMode.telemetry.addData("- Prop Location", propLocation);
        callingOpMode.telemetry.addLine("-----------------------------");

    }

}

