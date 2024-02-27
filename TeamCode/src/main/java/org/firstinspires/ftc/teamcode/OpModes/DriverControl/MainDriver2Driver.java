/*
package org.firstinspires.ftc.teamcode.OpModes.DriverControl;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.EverglowLibrary.Systems.ClawSystem;
//import org.firstinspires.ftc.teamcode.Systems.DrivingSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "MainDriver2Driver", group = "drive")
public class MainDriver2Driver extends LinearOpMode {
    public static final double SQUARE_SIZE = 60.5;

    private SampleMecanumDrive drive;
    private ElevatorSystem elevatorSystem;
    private ClawSystem clawSystem;
    private FourBarSystem fourBarSystem;
    private GWheelSystem gWheelSystem;
    private boolean isPressed = false;
    private boolean FourBarUp = false;

    private DcMotorEx SlideL, SlideR;
    private boolean SlideUp = false;
    private boolean circle_toggle = false;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        clawSystem = new ClawSystem(this);
        fourBarSystem = new FourBarSystem(this);
        gWheelSystem = new GWheelSystem(this);

        SlideL = hardwareMap.get(DcMotorEx.class, "SlideL");
        SlideR = hardwareMap.get(DcMotorEx.class, "SlideR");
        SlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideL.setDirection( DcMotorSimple.Direction.REVERSE);
        SlideR.setTargetPosition(0);
        SlideL.setTargetPosition(0);
        SlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideR.setPower(0.4);
        SlideL.setPower(0.4);


        waitForStart();
        while (opModeIsActive()) {


            if(gamepad2.dpad_up) { gWheelSystem.toggle(true); }
            else if(gamepad2.dpad_down) { gWheelSystem.toggle(false); }

            if(gamepad2.right_bumper) { clawSystem.toggle(); }

            //if(gamepad2.dpad_down) { fourBarSystem.set4BarPositionByLevel(FourBarSystem.Level.START); }
            if(gamepad2.square && !isPressed) {
                if(!FourBarUp) {

                    fourBarSystem.set4BarPositionByLevel(FourBarSystem.Level.PICKUP);
                    sleep(500);
                    fourBarSystem.setServoPositionByLevel(FourBarSystem.ServoAngel.PICKUP);
                    FourBarUp = true;
                }
                else {
                    fourBarSystem.setServoPositionByLevel(FourBarSystem.ServoAngel.DROP);
                    fourBarSystem.set4BarPositionByLevel(FourBarSystem.Level.DROP);
                    FourBarUp = false;
                }
            }

            isPressed = gamepad2.square;
            //if(gamepad2.dpad_right) { fourBarSystem.set4BarPositionByLevel(FourBarSystem.Level.REST); }


            if(gamepad2.circle && !circle_toggle){
                if(!SlideUp){ //get elevator up
                    SlideR.setPower(0.4);
                    SlideL.setPower(0.4);
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

            circle_toggle = !circle_toggle;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
            //fourBarSystem.updateP();
        }


    }
}
 */