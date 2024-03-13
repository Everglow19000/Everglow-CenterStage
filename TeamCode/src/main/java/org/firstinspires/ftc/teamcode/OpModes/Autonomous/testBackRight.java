package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "test")
public class testBackRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FourtyFivePoints fourtyFivePoints = new FourtyFivePoints();
        fourtyFivePoints.firstCall(this, FourtyFivePoints.StartPosition.BACK_RIGHT);
        fourtyFivePoints.SecondCall();
    }
}
