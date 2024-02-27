
package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Utils.MoveToConusAutonumous;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "MTCABackLeft", group = "Main")
public class MTCABackLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //drop the first pixel//
        MoveToConusAutonumous.AutonumousGeneral(this, MoveToConusAutonumous.StartPosition.BACKLEFT
                ,new SampleMecanumDrive(hardwareMap));
    }
}
