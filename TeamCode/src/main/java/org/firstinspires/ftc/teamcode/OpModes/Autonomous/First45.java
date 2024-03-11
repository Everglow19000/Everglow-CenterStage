/*
package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.EverglowLibrary.ThreadHandleLib.SequenceRunner;

public class First45 extends  FourtyFivePoints{
    @Override
    public void runAfterInput() {
        // Create all Sequences //

        getReadyToDrop  = sequenceControl.GetReadyToDropSeq();
        returnSystemsToStart = sequenceControl.DropAndRetreatSeq();


        // Trajectory Calculations //
        Pose2d startLocation = PoseInTiles(3.61, 5.667, East);
        if(isBack()) {
            startLocation.minus(PoseInTiles(2, 0, 0));
        }
        if(isRight()) {
            startLocation.minus(PoseInTiles(0.27, 0, 0));
        }
        drive.setPoseEstimate(tryRight(startLocation));



        Pose2d middleDropLocation = PoseInTiles(3.5, 4.5, East);
        splineToPurple = trajToPose(tryRight(startLocation), tryRight(middleDropLocation));

        threeYellowDropTrajectories.setStartPose(tryRight(middleDropLocation));
        threeYellowDropTrajectories.endLocations.poseMiddle = tryRight(PoseInTiles(5, 4.5, South));
        threeYellowDropTrajectories.endLocations.poseLeft =
                threeYellowDropTrajectories.endLocations.poseMiddle.plus(
                        new Pose2d(0, -distanceBetweenTags, 0));
        threeYellowDropTrajectories.endLocations.poseRight
                = threeYellowDropTrajectories.endLocations.poseMiddle.plus(
                new Pose2d(0, distanceBetweenTags, 0));
        threeYellowDropTrajectories.createTrajectories();



        Pose2d parkLocation = PoseInTiles(5.55, 5.6, South);
        threeParkTrajectories.setEndPose(parkLocation);
        threeParkTrajectories.startLocations = threeYellowDropTrajectories.endLocations;
        threeParkTrajectories.createTrajectories();

        //

        opMode.telemetry.addLine("Ready!!!!! ");
        opMode.telemetry.update();

        opMode.waitForStart();

        propPlace = cameraSystem.DetectAndFindPropLocation();
        opMode.telemetry.addData("Detectio ", propPlace);
        opMode.telemetry.update();
        Sleep(2);





        sequenceRunner = new SequenceRunner();

        // set the correct start location

        drive.followTrajectory(splineToPurple);

        opMode.telemetry.addData("Location ", LocationInTiles());
        opMode.telemetry.addData("start ", threeYellowDropTrajectories.startLocations.poseMiddle);
        opMode.telemetry.update();

        Sleep(7);

*/
/*       // GWheel - Drop Purple
        switch (propPlace) {
            case LEFT:
                drive.turn(PI / 2); //todo: if the turn is oposside (-Pi/2) then change it like it
                break;

            case RIGHT:
                drive.turn(-PI / 2); //todo: if the turn is oposside Pi/2 then change it like it
                break;
        }

        gWheelSystem.setPower(0.4);

        long time = System.currentTimeMillis();
        while(!opMode.isStopRequested()){
            if(System.currentTimeMillis()-time > 1000)
                break;
        }

        gWheelSystem.setPower(0); // G Wheel

        if(opMode.isStopRequested())
            return;

//        switch (propPlace) {
//            case LEFT:
//                if(isRight()) {
//                    drive.turn(-PI / 2);
//                }
//                else {
//                    drive.turn(PI / 2);
//                }
//                break;
//
//            case RIGHT:
//                if(isRight()) {
//                    drive.turn(PI / 2);
//                }
//                else {
//                    drive.turn(-PI / 2);
//                }
//                break;
//        }*//*





        opMode.telemetry.addData("end ", threeYellowDropTrajectories.endLocations.poseMiddle);

        opMode.telemetry.update();

        Sleep(3);


        threeYellowDropTrajectories.driveCorrectTrajectory();

        opMode.telemetry.addData("Location ", LocationInTiles());
        opMode.telemetry.addData("end ", threeYellowDropTrajectories.endLocations.poseMiddle);
        opMode.telemetry.update();
        Sleep(10);

    }
    }
}
*/
