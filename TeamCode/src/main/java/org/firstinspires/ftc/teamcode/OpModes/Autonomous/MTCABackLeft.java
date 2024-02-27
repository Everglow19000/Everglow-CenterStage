
package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.EverglowLibrary.ThreadHandleLib.SequenceControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "MTCABackLeft")
public class MTCABackLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FourtyFivePoints autonumous = new FourtyFivePoints();
        autonumous.firstCall(this, FourtyFivePoints.StartPosition.BACK_LEFT);


    }
}
