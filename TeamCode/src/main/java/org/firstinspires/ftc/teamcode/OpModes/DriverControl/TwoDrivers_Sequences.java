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
import org.EverglowLibrary.ThreadHandleLib.SequenceControl;
import org.EverglowLibrary.ThreadHandleLib.SequenceInSequence;
import org.EverglowLibrary.ThreadHandleLib.SequenceRunner;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TwoDrivers_Sequences", group = "main-drive")
public class TwoDrivers_Sequences extends LinearOpMode {

    private boolean seq1_toggle = false;
    private boolean seq2_toggle = false;
    private boolean seq3_toggle = false;
    private boolean seq4_toggle = false;
    private boolean gwheel_toggle = false;
    private boolean claw_toggle = false;
    private boolean elevator_toggle = false;
    private boolean left_Claw = false;
    private boolean right_claw = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FourBarSystem fourBarSystem = new FourBarSystem(this);
        ClawSystem clawSystem = new ClawSystem(this);
        ElevatorSystem elevatorSystem = new ElevatorSystem(this);
        GWheelSystem gWheelSystem = new GWheelSystem(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SequenceControl sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
        Servo planeServo = hardwareMap.get(Servo.class, "PlaneServo");
        planeServo.setPosition(0); //close servo mode
        double servoPos = 0.15; //open servo mode

        Sequence getReadyToDropSeq = sequenceControl.GetReadyToDropSeq();
        Sequence setUpAndUnderBlockSeq = sequenceControl.SetUpAndUnderBlockSeq();
        Sequence dropAndRetreatSeq = sequenceControl.DropAndRetreatSeq();
        Sequence MiddleDrop = sequenceControl.GetMiddleDrop();
        SequenceRunner sequenceRunner = new SequenceRunner();

        boolean isRest;

        sequenceControl = null; // no more use for that

        waitForStart();

        fourBarSystem.setMotorPower(0.85);

        while (opModeIsActive()){
            isRest = elevatorSystem.getCurrentPos() == ElevatorSystem.Level.DOWN
                    && fourBarSystem.getTargetLevel() == FourBarSystem.Level.PICKUP;
            try {
                if(gamepad2.square && !seq1_toggle){
                    if(!isRest && sequenceRunner.IsSequenceDone()){
                        elevatorSystem.goTo(ElevatorSystem.Level.UP.state);
                        seq1_toggle = gamepad2.square;
                        continue;
                    }
                    sequenceRunner.RunSequence(getReadyToDropSeq);
                }
                seq1_toggle = gamepad2.square;

                if(gamepad2.cross && !seq2_toggle && !isRest){
                    sequenceRunner.RunSequence(dropAndRetreatSeq);
                }
                seq2_toggle = gamepad2.cross;

                if(gamepad2.circle && !seq3_toggle){
                    if(!isRest && sequenceRunner.IsSequenceDone()){
                        elevatorSystem.goTo(ElevatorSystem.Level.DOWN.state);
                        seq3_toggle = gamepad2.circle;
                        continue;
                    }
                    sequenceRunner.RunSequence(setUpAndUnderBlockSeq);
                }
                seq3_toggle = gamepad2.circle;

                if(gamepad2.triangle && !seq4_toggle){
                    if(!isRest && sequenceRunner.IsSequenceDone()){
                        elevatorSystem.goTo(ElevatorSystem.Level.MED.state);
                        seq4_toggle = gamepad2.triangle;
                        continue;
                    }
                    sequenceRunner.RunSequence(MiddleDrop);
                }
                seq4_toggle = gamepad2.triangle;

            }catch (Exception e){
                telemetry.addData("exeption", e);
            }

            if(gamepad2.left_bumper && !claw_toggle){
                clawSystem.toggle();
            }
            claw_toggle = gamepad2.left_bumper;

            if(gamepad1.right_bumper && !gwheel_toggle){
                //clawSystem.ChangePos(true);
                gWheelSystem.toggle(true);
            }

            if(gamepad2.dpad_up){
                planeServo.setPosition(servoPos);
            }

            if (gamepad1.left_bumper && !gwheel_toggle){
                //clawSystem.ChangePos(true);
                gWheelSystem.toggle(false);
            }
            gwheel_toggle = gamepad1.right_bumper || gamepad1.left_bumper;

            if(gamepad1.square && !elevator_toggle){
                elevatorSystem.toggleMax();
            }
            elevator_toggle = gamepad1.square;

            if(gamepad2.left_trigger > 0.8 && !left_Claw){
                clawSystem.MoveOneClaw(false);
            }
            left_Claw = gamepad2.left_trigger > 0.8;

            if(gamepad2.right_trigger > 0.8 && !right_claw){
                clawSystem.MoveOneClaw(true);
            }
            right_claw = gamepad2.right_trigger > 0.8;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
            sequenceRunner.Update();
            if(fourBarSystem.getTargetPosition() == FourBarSystem.Level.DROP)
                fourBarSystem.set4BarPositionByLevel(fourBarSystem.getTargetPosition());

            telemetry.addData("is finished?",
                    fourBarSystem.isFinish(fourBarSystem.getTargetPosition()));
            //fourBarSystem.updateP(0.8);
            telemetry.update();
        }
        sequenceRunner.Interapt();
        sleep(1000);
    }
}
