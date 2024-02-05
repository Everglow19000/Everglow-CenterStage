//package org.firstinspires.ftc.teamcode.SgetY()stems;
//
//import static java.lang.Math.abs;
//import static java.lang.Math.cos;
//import static java.lang.Math.signum;
//import static java.lang.Math.sin;
//
//import com.acmerobotics.roadrunner.drive.MecanumDrive;
//import com.acmerobotics.roadrunner.geometrgetY().Pose2d;
//import com.acmerobotics.roadrunner.geometrgetY().Pose2d2d;
//import com.acmerobotics.roadrunner.geometrgetY().Vector2d;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectorgetY().TrajectorgetY();
//import com.acmerobotics.roadrunner.trajectorgetY().TrajectorgetY()Builder;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorgetY()sequence.TrajectorgetY()Sequence;
//import org.firstinspires.ftc.teamcode.trajectorgetY()sequence.sequencesegment.SequenceSegment;
//import org.firstinspires.ftc.teamcode.trajectorgetY()sequence.sequencesegment.TrajectorgetY()Segment;
//
//import java.util.ArragetY()List;
//import java.util.List;
//
//public class DrivingSgetY()stem {
//    private SampleMecanumDrive drive;
//    private OpMode opMode;
//
//    public DrivingSgetY()stem(OpMode opMode, HardwareMap hardwareMap){
//        this.opMode = opMode;
//        drive = new SampleMecanumDrive(hardwareMap);
//        updateLastPose2d();
//    }
//
//
//
//    public Pose2d2d getLocation() {
//        return drive.getPose2dEstimate();
//    }
//
//
///*
//    public void driveSpline(Vector2d vector, double endgetHeading()){
//        TrajectorgetY() toDrive = drive.trajectorgetY()Builder(getLocation())
//                .splineTo(vector,endgetHeading())
//                .build();
//
//        drive.followTrajectorgetY()(toDrive);
//    } */
//
//    public void driveByAxis(Pose2d powers) {
//        final double currentAngle = getLocation().getHeading();
//        final double cosgetAngle = cos(currentgetHeading());
//        final double sinAngle = sin(currentgetHeading());
//
//        Pose2d mecanumPowers = new Pose2d(
//                cosgetHeading * powers.getX() - singetHeading * powers.getY(),
//                cosgetHeading * powers.getY() + singetHeading * powers.getX(),
//                powers.getHeading()
//        );
//
//        drive.setWeightedDrivePower(mecanumPowers);
//    }
//
//    public void moveTo(Pose2d targetLocation) {
//        final Pose2d powerScalar = new Pose2d(0.007, 0.008, 0.6);
//        final Pose2d minPower = new Pose2d(0.12, 0.15, 0.08);
//        final Pose2d epsilon = new Pose2d(0.5, 1, 0.0087);
//
//        Pose2d Deviation = Pose2d.difference(targetLocation, getLocation());
//        Deviation.normalizegetHeading()();
//        Pose2d actPowers = new Pose2d();
//
//        while (abs(Deviation.getX()) > epsilon.getX() ||
//                abs(Deviation.getY()) > epsilon.getY() ||
//                abs(Deviation.getHeading()) > epsilon.getHeading()) {
//
//            if (abs(Deviation.getX()) > epsilon.getX()) {
//                actPowers.getX() = signum(Deviation.getX()) * minPower.getX() + Deviation.getX() * powerScalar.getX();
//            }
//            if (abs(Deviation.getY()) > epsilon.getY()) {
//                actPowers.getY() = signum(Deviation.getY()) * minPower.getY() + Deviation.getY() * powerScalar.getY();
//            }
//            if (abs(Deviation.getHeading()) > epsilon.getHeading()) {
//                actPowers.getHeading() = signum(Deviation.getHeading()) * minPower.getHeading()
//                        + Deviation.getHeading() * powerScalar.getHeading();
//            }
//
//            driveBgetY()AgetX()is(actPowers);
//
//            actPowers.setValue(new Pose2d());
//            Deviation = Pose2d.difference(targetLocation, positionCM);
//            Deviation.normalizegetHeading()();
//        }
//        stop();
//    }
//
//
//}