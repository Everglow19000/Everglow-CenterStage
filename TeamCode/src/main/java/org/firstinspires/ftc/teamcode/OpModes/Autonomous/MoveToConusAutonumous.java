package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.EverglowLibrary.Systems.CameraSystem;
import org.firstinspires.ftc.teamcode.Systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.Systems.GWheelSystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Basic Autunumous Red")
public class MoveToConusAutonumous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
       AutonumousGeneral(true, this);
    }

    public static void AutonumousGeneral(boolean isRed, LinearOpMode opMode){
        GWheelSystem gWheelSystem = new GWheelSystem(opMode);
        CameraSystem cameraSystem = new CameraSystem(opMode, isRed);
        FourBarSystem fourBarSystem = new FourBarSystem(opMode);
        SampleMecanumDrive drive = new SampleMecanumDrive(opMode.hardwareMap);
        double xPoint = 110, yPoint = 23;
        Trajectory toDriveStraf = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .strafeRight(yPoint).build();//move 10 cm left
        Trajectory toDriveStright = drive.trajectoryBuilder(new Pose2d(yPoint, 0,0))
                .forward(xPoint)
                .build();//move 70 cm forward
        boolean ifrun = true;

        opMode.waitForStart();


        CameraSystem.DetectionLocation location  = cameraSystem.DetectAndFindPropLocation();
        opMode.telemetry.addData("Target", location);
        if (opMode.isStopRequested()) return;
        double angle = Math.PI/1.8;
        double forwardDis = 15;
        while(opMode.opModeIsActive()){
            if(ifrun) {
                drive.followTrajectory(toDriveStraf);
                if (opMode.isStopRequested()) return;
                drive.followTrajectory(toDriveStright);
                switch (location){
                    case LEFT:
                        drive.turn(angle);
                        Trajectory trajL = drive.trajectoryBuilder(new Pose2d(yPoint, xPoint,angle))
                                .forward(forwardDis)
                                .build();//move 70 cm forwar)
                        drive.followTrajectory(trajL);
                        break;
                    case RIGHT:
                        angle = -1*angle;
                        drive.turn(angle);
                        Trajectory trajR = drive.trajectoryBuilder(new Pose2d(yPoint, xPoint,angle))
                                .forward(forwardDis)
                                .build();//move 70 cm forwar)
                        drive.followTrajectory(trajR);
                        break;
                    case MIDDLE:
                        Trajectory trajectoryMiddle = drive.trajectoryBuilder(new Pose2d(yPoint, xPoint,angle))
                                .forward(-forwardDis)
                                .build();
                        drive.followTrajectory(trajectoryMiddle);
                        break;

                }


                gWheelSystem.setPower(0.4);
                if (opMode.isStopRequested()) return;
                long time = System.currentTimeMillis();
                while(!opMode.isStopRequested()){
                    if(System.currentTimeMillis()-time > 5000)
                        break;
                }
                gWheelSystem.setPower(0);

//                Trajectory Park = drive.trajectoryBuilder(new Pose2d(yPoint, xPoint, angle))
//                        .splineTo(new Vector2d(yPoint,150), 0 )
//                        .build();//move 70 cm forward
//                drive.followTrajectory(Park);
            }
            ifrun = false;
        }
    }
}