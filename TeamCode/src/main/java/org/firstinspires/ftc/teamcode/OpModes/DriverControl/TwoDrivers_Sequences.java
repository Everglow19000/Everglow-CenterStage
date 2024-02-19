package org.firstinspires.ftc.teamcode.OpModes.DriverControl;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;
import org.EverglowLibrary.ThreadHandleLib.SequenceInSequence;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TwoDrivers_Sequences", group = "drive")
public class TwoDrivers_Sequences extends LinearOpMode {

    private boolean seq1_toggle = false;
    private boolean seq2_toggle = false;
    private boolean seq3_toggle = false;
    private boolean seq4_toggle = false;
    private boolean gwheel_toggle = false;
    private boolean claw_toggle = false;
    private boolean elevator_toggle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FourBarSystem fourBarSystem = new FourBarSystem(this);
        ClawSystem clawSystem = new ClawSystem(this);
        ElevatorSystem elevatorSystem = new ElevatorSystem(this);
        GWheelSystem gWheelSystem = new GWheelSystem(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SequenceControl sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
        Servo planeServo = hardwareMap.get(Servo.class, "PlaneServo");
        planeServo.setPosition(planeServo.getPosition());
        double servoPos = planeServo.getPosition()+0.5;
        int pos = elevatorSystem.getCurrentPos().state;

        Sequence getReadyToDropSeq = sequenceControl.GetReadyToDropSeq();
        SequenceInSequence setUpAndUnderBlockSeq = sequenceControl.SetUpAndUnderBlockSeq();
        Sequence dropAndRetreatSeq = sequenceControl.DropAndRetreatSeq();
        Sequence getUpSeq = sequenceControl.GetUpAndReadyToDrop();

        sequenceControl = null; // no more use for that

        waitForStart();

        while (opModeIsActive()){
            try {
                if(gamepad1.triangle || gamepad1.dpad_up){
                    getReadyToDropSeq.interruptSequence();
                    dropAndRetreatSeq.interruptSequence();
                    setUpAndUnderBlockSeq.stopAll();
                    getUpSeq.interruptSequence();
                }

                if(gamepad2.square && !seq1_toggle){
                    getReadyToDropSeq.startSequence();
                    pos = elevatorSystem.getCurrentPos().state;
                }
                seq1_toggle = gamepad2.square;

                if(gamepad2.cross && !seq2_toggle){
                    dropAndRetreatSeq.startSequence();
                    pos = elevatorSystem.getCurrentPos().state;
                }
                seq2_toggle = gamepad2.cross;

                if(gamepad2.circle && !seq3_toggle){
                    setUpAndUnderBlockSeq.RunAll();
                    pos = elevatorSystem.getCurrentPos().state;
                }
                seq3_toggle = gamepad2.circle;

                if(gamepad2.triangle && !seq4_toggle){
                    getUpSeq.startSequence();
                    pos = elevatorSystem.getCurrentPos().state;
                }
                seq4_toggle = gamepad2.triangle;
            }catch (Exception e){
                telemetry.addData("exeption", e);
                telemetry.update();
            }

            //if(gamepad1.circle){

              //  elevatorSystem.goTo(pos);
            //}
            //pos += gamepad2.left_stick_y/10;

            if(gamepad2.right_bumper && !claw_toggle){
                clawSystem.toggle();
            }
            claw_toggle = gamepad2.right_bumper;

            if(gamepad1.right_bumper && !gwheel_toggle){
                gWheelSystem.toggle(true);
            }

            if(gamepad2.dpad_up){
                planeServo.setPosition(servoPos);
            }

            if (gamepad1.left_bumper && !gwheel_toggle){
                gWheelSystem.toggle(false);
            }
            gwheel_toggle = gamepad1.right_bumper || gamepad1.left_bumper;

            if(gamepad1.square && !elevator_toggle){
                elevatorSystem.toggleMax();
            }
            elevator_toggle = gamepad1.square;



            //if(gamepad2.)
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            //telemetry.addData("elevator pos", pos);
            drive.update();
            fourBarSystem.updateP(0.35);
        }
    }
}
