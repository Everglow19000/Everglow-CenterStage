package org.EverglowLibrary.OpModeTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetectionEasy;

@TeleOp(name = "concept detection prop")
public class CameraPropConceptOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ConceptTensorFlowObjectDetectionEasy conceptTF =
                new ConceptTensorFlowObjectDetectionEasy();

        conceptTF.runOpMode();
    }
}
