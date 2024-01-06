package org.EverglowLibrary.Systems;

import android.app.TaskInfo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;


public class CameraSystem {
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
        if(detections.size() == 0)
            return new Hashtable<AprilTagDetection,AprilTagLocation>();

        Dictionary<AprilTagDetection, AprilTagLocation> detectionToLocation =
                new Hashtable<AprilTagDetection, AprilTagLocation>();

        Dictionary<Double, AprilTagDetection> doubleToDetection = new Hashtable<Double, AprilTagDetection>();

        for (int i = 0; i<detections.size(); i++){
            doubleToDetection.put(detections.get(i).ftcPose.x, detections.get(i));
        }

        double[] posDouble = new double[doubleToDetection.size()]; //the right is the smallest, the left is the highest
        Enumeration<Double> allValues = doubleToDetection.keys();
        int count =0;
        while (allValues.hasMoreElements()){
            posDouble[count] = allValues.nextElement();
            count++;
        }

        try {
            posDouble = Arrays.stream(posDouble).sorted().toArray();

        }catch (NullPointerException nullEX){
            return null;
        }

        if(posDouble.length == 1) {
            detectionToLocation.put(doubleToDetection.get(posDouble[0]), AprilTagLocation.MIDDLE);
            return detectionToLocation;
        }
        for (int i =0; i<doubleToDetection.size(); i++){
            switch (i){
                case 0: {
                    detectionToLocation.put(doubleToDetection.get(posDouble[0]), AprilTagLocation.LEFT);
                    break;
                }
                case 1:{
                    detectionToLocation.put(doubleToDetection.get(posDouble[1]), AprilTagLocation.MIDDLE);
                    break;
                }
                case 2:{
                    detectionToLocation.put(doubleToDetection.get(posDouble[2]), AprilTagLocation.RIGHT);
                }
            }
        }

        return detectionToLocation;
    }

    public Dictionary<AprilTagDetection,AprilTagLocation> GetAprilTagLocation() {
        return GetAprilTagLocation(DetectAprilTags());
    }

    public void CloseCamera() { m_Camera.close();}
}
