package org.firstinspires.ftc.teamcode.OpModes.Autonomous;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Utils.MoveToConusAutonumous;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "MTCABackRight", group = "Main")
public class MTCABackRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MoveToConusAutonumous.AutonumousGeneral(this, MoveToConusAutonumous.StartPosition.BACKRIGHT
                ,new SampleMecanumDrive(hardwareMap));

    }
}
