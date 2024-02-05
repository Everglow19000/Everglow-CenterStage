package org.firstinspires.ftc.teamcode.OpModes.Tests;

import android.os.strictmode.SqliteObjectLeakedViolation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.Executor;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;

@TeleOp(name = "sequence test", group = "test")
public class SequenceTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElevatorSystem elevatorSystem = new ElevatorSystem(this);
        FourBarSystem fourBarSystem = new FourBarSystem(this);
        GWheelSystem gWheelSystem = new GWheelSystem(this);
        ClawSystem clawSystem = new ClawSystem(this);

        Sequence toDrop = new Sequence(false, gWheelSystem.getExecutor(false)
        , Executor.sleep(3000), gWheelSystem.getExecutor(), clawSystem.getExecutor()
                , elevatorSystem.getExecutor(), fourBarSystem.getExecutor(), clawSystem.getExecutor());

        Sequence toTheBegging = new Sequence(false, fourBarSystem.getExecutor(),elevatorSystem.getExecutor());

        waitForStart();

        while (opModeIsActive()){
            try {
                if(gamepad1.dpad_up){
                    toDrop.startSequence();
                }

                if(gamepad1.dpad_down){
                    toTheBegging.startSequence();
                }
            } catch (Exception e){
             //does not matter for now
            }
        }
    }
}
