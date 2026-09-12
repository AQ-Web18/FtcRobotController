package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class Webcam {
    private AprilTagProcessor atp;
    private VisionPortal vp;


    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry tele;

    public void init(HardwareMap hwMap , Telemetry tele) {
        this.tele = tele;

        atp = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hwMap.get(WebcamName.class, "Webcam"));
            builder.setCameraResolution(new Size(1280,720));
            builder.addProcessor(atp);

            vp = builder.build();
    }

    public void update(){
        detectedTags = atp.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedid){
        if (detectedid == null) {return;}
        if (detectedid.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedid.id, detectedid.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedid.ftcPose.x, detectedid.ftcPose.y, detectedid.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedid.ftcPose.pitch, detectedid.ftcPose.roll, detectedid.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedid.ftcPose.range, detectedid.ftcPose.bearing, detectedid.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedid.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedid.center.x, detectedid.center.y));
        }
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }


        public void stop(){
            if (vp != null){
                vp.close();
            }
        }


}


