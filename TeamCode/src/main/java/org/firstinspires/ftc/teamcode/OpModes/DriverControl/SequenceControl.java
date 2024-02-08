package org.firstinspires.ftc.teamcode.OpModes.DriverControl;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.Executor;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "SequenceControl", group = "OpMode")
public class SequenceControl extends LinearOpMode {

    private boolean seq1_toggle = false;
    private boolean seq2_toggle = false;
    private boolean seq3_toggle = false;
    private boolean seq4_toggle = false;
    private boolean upAndDown_toggle = false;

    private final Sequence getReadyToDropSeq;
    private final Sequence setUpAndUnderBlockSeq;
    private final Sequence dropAndRetreatSeq;
    private final Sequence getUpAndReadyToDrop;

    public SequenceControl(ClawSystem clawSystem, FourBarSystem fourBarSystem
            , ElevatorSystem elevatorSystem){
        getReadyToDropSeq = new Sequence(false, clawSystem.getExecutor(false)
                ,elevatorSystem.getExecutor(ElevatorSystem.Level.UP)
                , fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP));

        setUpAndUnderBlockSeq = new Sequence(false, clawSystem.getExecutor(false),
                elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.REST, FourBarSystem.ServoAngel.DROP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN));

        dropAndRetreatSeq = new Sequence(false, clawSystem.getExecutor(true),
                fourBarSystem.getExecutor(FourBarSystem.Level.PICKUP, FourBarSystem.ServoAngel.PICKUP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN));

        getUpAndReadyToDrop = new Sequence(true, elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        ClawSystem clawSystem = new ClawSystem(this);
        FourBarSystem fourBarSystem = new FourBarSystem(this);
        ElevatorSystem elevatorSystem = new ElevatorSystem(this);
        GWheelSystem gWheelSystem = new GWheelSystem(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Sequence upAndDown = new Sequence(false, elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN));
        Sequence getReadyToDropSeq = new Sequence(false, clawSystem.getExecutor(false)
            ,elevatorSystem.getExecutor(ElevatorSystem.Level.UP)
                , fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP));

        Sequence setUpAndUnderBlockSeq = new Sequence(false, clawSystem.getExecutor(false),
                elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                Executor.sleep(1000),
                fourBarSystem.getExecutor(FourBarSystem.Level.REST, FourBarSystem.ServoAngel.DROP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN));

        Sequence dropAndRetreatSeq = new Sequence(false, clawSystem.getExecutor(true),
                Executor.sleep(1000),
                fourBarSystem.getExecutor(FourBarSystem.Level.PICKUP, FourBarSystem.ServoAngel.PICKUP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN));

        Sequence getUpAndReadyToDrop = new Sequence(true, elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP));

        waitForStart();

        while (opModeIsActive()){
            try {
                if (gamepad1.dpad_up) {
                    getReadyToDropSeq.interruptSequence();
                    setUpAndUnderBlockSeq.interruptSequence();
                    dropAndRetreatSeq.interruptSequence();
                    getUpAndReadyToDrop.interruptSequence();
                    break;
                }

                if (gamepad1.square && !seq1_toggle) {
                    getReadyToDropSeq.startSequence();
                }
                seq1_toggle = gamepad1.square;

                if(gamepad1.circle && !seq2_toggle){
                    setUpAndUnderBlockSeq.startSequence();
                }
                seq2_toggle = gamepad1.circle;

                if (gamepad1.cross && !seq3_toggle) {
                    dropAndRetreatSeq.startSequence();
                }
                seq3_toggle = gamepad1.cross;

                if(gamepad1.triangle && !seq4_toggle){
                    getUpAndReadyToDrop.startSequence();
                }
                seq4_toggle = gamepad1.triangle;

                if (gamepad1.dpad_down && !upAndDown_toggle){
                    upAndDown.startSequence();
                }
                upAndDown_toggle = gamepad1.dpad_down;

            }catch (Exception e){
                telemetry.addLine(e.toString());
                telemetry.update();
                break;
            }
        }
    }

    public Sequence GetReadyToDropSeq(){
        return getReadyToDropSeq;
    }

    public Sequence SetUpAndUnderBlockSeq(){
        return setUpAndUnderBlockSeq;
    }

    public Sequence DropAndRetreatSeq(){
        return dropAndRetreatSeq;
    }

    public Sequence GetUpAndReadyToDrop(){
        return getUpAndReadyToDrop;
    }
}
