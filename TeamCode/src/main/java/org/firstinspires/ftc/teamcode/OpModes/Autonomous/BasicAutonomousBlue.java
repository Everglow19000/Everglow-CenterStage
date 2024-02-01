package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonumous Blue")
public class BasicAutonomousBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MoveToConusAutonumous.AutonumousGeneral(false, this);
    }
}
