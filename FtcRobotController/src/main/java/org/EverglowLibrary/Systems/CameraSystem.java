package org.EverglowLibrary.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Arrays;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;


public class CameraSystem {

    private AprilTagProcessor m_AprilTag;
    private final OpMode m_OpMode;
    private VisionPortal m_Camera;
    private TfodProcessor m_Prop;
    private List<Recognition> LastRecognitions;

    public enum AprilTagLocation{
        LEFT, MIDDLE, RIGHT
    }

    public CameraSystem(OpMode opMode){
        m_OpMode = opMode;

        m_Prop = new TfodProcessor.Builder().build();
        m_AprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(m_OpMode.hardwareMap.get(WebcamName.class, "webcam"));

        builder.addProcessor(m_AprilTag);
        builder.addProcessor(m_Prop);

        m_Camera =  builder.build();
    }

    public List<AprilTagDetection> DetectAprilTags(){
        return m_AprilTag.getDetections();
    }

    public Dictionary<AprilTagLocation,AprilTagDetection> GetAprilTagLocation(List<AprilTagDetection> detections){
        if(detections.size() == 0)
            return new Hashtable<AprilTagLocation, AprilTagDetection>();

        Dictionary<AprilTagLocation,AprilTagDetection> detectionToLocation =
                new Hashtable<>();

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
            detectionToLocation.put(AprilTagLocation.MIDDLE, doubleToDetection.get(posDouble[0]));
            return detectionToLocation;
        }
        for (int i =0; i<doubleToDetection.size(); i++){
            switch (i){
                case 0: {
                    detectionToLocation.put(AprilTagLocation.LEFT, doubleToDetection.get(posDouble[0]));
                    break;
                }
                case 1:{
                    detectionToLocation.put(AprilTagLocation.MIDDLE, doubleToDetection.get(posDouble[1]));
                    break;
                }
                case 2:{
                    detectionToLocation.put(AprilTagLocation.RIGHT, doubleToDetection.get(posDouble[2]));
                }
            }
        }
        return detectionToLocation;
    }

    public Dictionary<AprilTagLocation, AprilTagDetection> GetAprilTagLocation() {
        return GetAprilTagLocation(DetectAprilTags());
    }

    public double recognitionToDistance(Recognition rec, boolean isX){
        if(isX)
            return (rec.getLeft() + rec.getRight()) / 2 ;
        else
            return (rec.getTop()  + rec.getBottom()) / 2 ;
    }

    public double[] getRecognitionsDistance(boolean isX){
        double[] allDistances = new double[LastRecognitions.size()];
        for (int i = 0; i < LastRecognitions.size(); i++){
            allDistances[i] = recognitionToDistance(LastRecognitions.get(i), isX);
        }
        return allDistances;
    }
    public List<Recognition> detectAllPixels(){
        LastRecognitions = m_Prop.getRecognitions();
        return LastRecognitions;
    }

    public void CloseCamera() { m_Camera.close();}
}
