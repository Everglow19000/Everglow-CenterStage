package org.firstinspires.ftc.teamcode.OpModes.Tests;

import android.os.strictmode.SqliteObjectLeakedViolation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.Executor;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;
import org.EverglowLibrary.utils.ExecutorSleep;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.ExecutorUtils.ExecutorTrajectories;

@TeleOp(name = "sequence test", group = "test")
public class SequenceTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElevatorSystem elevatorSystem = new ElevatorSystem(this);
        FourBarSystem fourBarSystem = new FourBarSystem(this);
        GWheelSystem gWheelSystem = new GWheelSystem(this);
        ClawSystem clawSystem = new ClawSystem(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(0,0))
                .forward(50)
                .build();
        ExecutorTrajectories executorTrajectories = new ExecutorTrajectories(drive, trajectory,true);
        Sequence moveStraightDownUP = new Sequence(false, executorTrajectories
                ,gWheelSystem.getExecutor(true)
                , Executor.sleep(2000)
                ,gWheelSystem.getExecutor(true));

        Sequence downUP = new Sequence(false, elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN)
                , elevatorSystem.getExecutor(ElevatorSystem.Level.UP));

        Sequence switchElevators = new Sequence(false,elevatorSystem.getExecutor(ElevatorSystem.Level.UP));

        waitForStart();

        boolean isSlide = false;

        while (opModeIsActive()){
            try {
                if(gamepad1.dpad_up){
                    moveStraightDownUP.startSequence();
                }

                if(gamepad1.dpad_down){
                    switchElevators.startSequence();
                }
                if (gamepad1.x){
                    downUP.startSequence();
                }

                if(!isSlide && gamepad1.circle){
                    elevatorSystem.toggle();
                    telemetry.update();
                }
                isSlide = gamepad1.circle;
            } catch (Exception e){
             //does not matter for now
                telemetry.addLine("error");
                telemetry.update();
            }
        }
    }
}
