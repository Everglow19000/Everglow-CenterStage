package org.firstinspires.ftc.teamcode.OpModes.DriverControl;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TwoDrivers_Sequences", group = "drive")
public class TwoDrivers_Sequences extends LinearOpMode {

    private boolean seq1_toggle = false;
    private boolean seq2_toggle = false;
    private boolean seq3_toggle = false;
    private boolean seq4_toggle = false;
    private boolean gwheel_toggle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FourBarSystem fourBarSystem = new FourBarSystem(this);
        ClawSystem clawSystem = new ClawSystem(this);
        ElevatorSystem elevatorSystem = new ElevatorSystem(this);
        GWheelSystem gWheelSystem = new GWheelSystem(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SequenceControl sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);

        Sequence getReadyToDropSeq = sequenceControl.GetReadyToDropSeq();
        Sequence setUpAndUnderBlockSeq = sequenceControl.SetUpAndUnderBlockSeq();
        Sequence dropAndRetreatSeq = sequenceControl.DropAndRetreatSeq();
        Sequence getUpSeq = sequenceControl.GetUpAndReadyToDrop();

        sequenceControl = null; // no more use for that

        waitForStart();

        while (opModeIsActive()){
            try {

                if(gamepad1.triangle){
                    getReadyToDropSeq.interruptSequence();
                    dropAndRetreatSeq.interruptSequence();
                    setUpAndUnderBlockSeq.interruptSequence();
                    getUpSeq.interruptSequence();
                }

                if(gamepad2.square && !seq1_toggle){
                    getReadyToDropSeq.startSequence();
                }
                seq1_toggle = gamepad2.square;

                if(gamepad2.cross && !seq2_toggle){
                    dropAndRetreatSeq.startSequence();
                }
                seq2_toggle = gamepad2.cross;

                if(gamepad2.circle && !seq3_toggle){
                    setUpAndUnderBlockSeq.startSequence();
                }
                seq3_toggle = gamepad2.circle;

                if(gamepad2.triangle && !seq4_toggle){
                    getUpSeq.startSequence();
                }
                seq4_toggle = gamepad2.triangle;

            }catch (Exception e){
                telemetry.addData("exeption", e);
                telemetry.update();
            }

            if(gamepad1.right_bumper && !gwheel_toggle){
                gWheelSystem.toggle(true);
            }

            if (gamepad1.left_bumper && !gwheel_toggle){
                gWheelSystem.toggle(false);
            }
            gwheel_toggle = gamepad1.right_bumper || gamepad1.left_bumper;

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
