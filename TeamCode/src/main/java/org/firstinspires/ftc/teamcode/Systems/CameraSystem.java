package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;


public class CameraSystem{

    private AprilTagProcessor m_AprilTag;
    private final OpMode m_OpMode;
    private VisionPortal m_Camera;

    public enum AprilTagLocation{
        LEFT, MIDDLE, RIGHT
    }

    public CameraSystem(OpMode opMode){
        m_OpMode = opMode;

        m_AprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(m_OpMode.hardwareMap.get(WebcamName.class, "webcam"));
        builder.addProcessor(m_AprilTag);

        m_Camera =  builder.build();
    }

    public List<AprilTagDetection> DetectAprilTags(){
        return m_AprilTag.getDetections();
    }

    public Dictionary<AprilTagDetection,AprilTagLocation> GetAprilTagLocation(List<AprilTagDetection> detections){

        Dictionary<AprilTagDetection, AprilTagLocation> detectionToLocation =
                new Hashtable<AprilTagDetection, AprilTagLocation>();

        Dictionary<Double, AprilTagDetection> doubleToDetection = new Hashtable<Double, AprilTagDetection>();
        double[] posDouble = new double[]{0,0,0}; //the right is the smallest, the left is the highest
        AprilTagDetection[] detects = new AprilTagDetection[3];

        for (int i = 0; i<detections.size(); i++){
            posDouble[i] = detections.get(i).ftcPose.x;
            doubleToDetection.put(detections.get(i).ftcPose.x, detections.get(i));
        }

        posDouble = Arrays.stream(posDouble).sorted().toArray();

        detectionToLocation.put(doubleToDetection.get(posDouble[0]), AprilTagLocation.LEFT);
        detectionToLocation.put(doubleToDetection.get(posDouble[1]), AprilTagLocation.MIDDLE);
        detectionToLocation.put(doubleToDetection.get(posDouble[2]), AprilTagLocation.RIGHT);

        return detectionToLocation;
    }

    public Dictionary<AprilTagDetection,AprilTagLocation> GetAprilTagLocation() {
        return GetAprilTagLocation(DetectAprilTags());
    }

}
