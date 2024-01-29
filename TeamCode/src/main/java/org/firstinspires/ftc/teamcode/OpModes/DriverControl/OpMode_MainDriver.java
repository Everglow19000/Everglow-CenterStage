package org.firstinspires.ftc.teamcode.OpModes.DriverControl;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class OpMode_MainDriver extends LinearOpMode{
    Servo FlipServo, ClawR, ClawL;
    DcMotorEx SlideL, SlideR, FourBar, GagazMot;
    boolean SlideUp = false;
    boolean ClawClosed = false;
    boolean ClawExtended = false;
    boolean A_toggle = false;
    boolean circle_toggle = false;
    boolean right_bumper_toggle = false;
    boolean square_toggle = false;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // flip servo
        FlipServo = hardwareMap.get(Servo.class, "FlipServo");
        FlipServo.setPosition(0);

        // claw
        ClawR = hardwareMap.get(Servo.class, "ClawR");
        ClawL = hardwareMap.get(Servo.class, "ClawL");
        ClawR.setPosition(1);
        ClawL.setPosition(0);

        // slide
        SlideL = hardwareMap.get(DcMotorEx.class, "SlideL");
        SlideR = hardwareMap.get(DcMotorEx.class, "SlideR");
        SlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideR.setDirection( DcMotorSimple.Direction.REVERSE);
        SlideR.setTargetPosition(0);
        SlideL.setTargetPosition(0);
        SlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // four bar
        FourBar = hardwareMap.get(DcMotorEx.class, "4Bar");
        FourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FourBar.setDirection( DcMotorSimple.Direction.REVERSE);
        FourBar.setPower(0.3);
        FourBar.setTargetPosition(-10);
        FourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FourBar.setTargetPositionTolerance(10);

        // galal azikonim
        GagazMot = hardwareMap.get(DcMotorEx.class, "GagazMot");

        waitForStart();

        while (!isStopRequested()) {
            if(gamepad1.circle && !circle_toggle){
                if(SlideUp){ //get elevator up
                    SlideR.setPower(0.6);
                    SlideL.setPower(0.6);
                    SlideR.setTargetPosition(720);
                    SlideL.setTargetPosition(720);
                    SlideUp = !SlideUp;
                } else { //get elevator down
                    SlideR.setPower(0.4);
                    SlideL.setPower(0.4);
                    SlideR.setTargetPosition(0);
                    SlideL.setTargetPosition(0);
                    SlideUp = !SlideUp;
                }
            }
            circle_toggle = gamepad1.circle; //control when the elevator will work,
            // lock circle until the movement has done

            if(gamepad1.right_bumper && !right_bumper_toggle){
                if(ClawClosed){
                    ClawL.setPosition(0.7);
                    ClawR.setPosition(0.4);
                    ClawClosed = !ClawClosed;
                } else {
                    ClawL.setPosition(0);
                    ClawR.setPosition(1);
                    ClawClosed = !ClawClosed;
                }
            }
            right_bumper_toggle = gamepad1.right_bumper;

            if(gamepad1.square && !square_toggle){
                if(ClawExtended){
                    FlipServo.setPosition(0.5);
                    FourBar.setTargetPosition(270);
                    ClawExtended = !ClawExtended;
                } else {
                    FourBar.setTargetPosition(-10);
                    sleep(500);
                    FlipServo.setPosition(0);
                    ClawExtended = !ClawExtended;
                }
            }
            square_toggle = gamepad1.square;

            if(!A_toggle && gamepad1.a) {
                if (GagazMot.getPower() == 0) {
                    GagazMot.setPower(-1);
                }else {
                    GagazMot.setPower(0);
                }
            }
            A_toggle = gamepad1.a;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            //Pose2d poseEstimate = drive.getPoseEstimate();
            //telemetry.addData("x", gamepad1.a);
            //telemetry.update();
        }
    }
}
