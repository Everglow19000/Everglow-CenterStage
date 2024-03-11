package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class DrivingSystem extends SampleMecanumDrive {
    LinearOpMode opMode;

    public DrivingSystem(LinearOpMode opMode) {
        super(opMode.hardwareMap);
        this.opMode = opMode;
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static double TILE_LENGTH = 60.5;

    /**
     * Creates a new Pose2d but with X and Y values multiplaied by Tile Length
     *
     * @param x The X value of the robot Position according to filed Axis - and in Tile units
     * @param y The Y value of the robot Position according to filed Axis - and in Tile units
     * @param Heading The angle of the Robot in Relation to the filed - 0 is North
     *
     * @return new Pose2d in CM Units
     */
    public Pose2d PoseInTiles(double x, double y, double Heading) {
        return new Pose2d(x * TILE_LENGTH, y * TILE_LENGTH, Heading);
    }


    private double chooseLain(Pose2d robotTileLocation) {
        double tileY = (int)robotTileLocation.getY() + 0.5;
        if((robotTileLocation.getX() > 3.5) && (tileY == 2.5)) tileY = 1.5;
        if((robotTileLocation.getX() > 3.5) && (tileY == 3.5)) tileY = 4.5;
        //opMode.telemetry.addData("Target Lain ", tileY);
        return tileY;
    }

    private double anglePower(double angle) {
        double targetAngle = 0;
        if(angle >= PI/2) {
            targetAngle = PI;
        } else if(angle <= -PI/2){
            targetAngle = -PI;
        }
        if(abs(targetAngle - angle) < 3 / 180.0 * PI) return 0;
        return (targetAngle - angle) * 3;
    }

    private Pose2d InPowers(Pose2d robotTileLocation, Pose2d axisPowers) {
        final double Y = robotTileLocation.getY();
        double deviationY = chooseLain(robotTileLocation) - Y;

        double Py = deviationY * deviationY;
        double Pr = anglePower(robotTileLocation.getHeading());

        return new Pose2d(axisPowers.getX(), Py, Pr);
    }


    private Pose2d PassPowers(Pose2d robotTileLocation, Pose2d axisPowers) {
        final double X = robotTileLocation.getX(), Y = robotTileLocation.getY();
        double deviationX = 0;
        if(X > 3.5) deviationX = X - 3.5;
        if(X < 1.5) deviationX = 1.5 - X;

        double deviationY = chooseLain(robotTileLocation) - Y;

        double Py = deviationY * abs(deviationY / deviationX * axisPowers.getX() * 5);
        double Pr = anglePower(robotTileLocation.getHeading());

        return new Pose2d(axisPowers.getX(), Py, Pr);
        //return new Pose2d(axisPowers.getX(), Py, axisPowers.getHeading());
    }

    Pose2d robotToAxisPowers(Pose2d inputPowers, double heading) {
        double powerX =  inputPowers.getX() * cos(heading) - inputPowers.getY() * sin(heading);
        double powerY =  inputPowers.getX() * sin(heading) + inputPowers.getY() * cos(heading);
        return new Pose2d(powerX, powerY, inputPowers.getHeading());
    }

    Pose2d axisToRobotPowers(Pose2d inputPowers, double heading) {
        double powerX =  inputPowers.getX() * cos(heading) + inputPowers.getY() * sin(heading);
        double powerY =  -inputPowers.getX() * sin(heading) + inputPowers.getY() * cos(heading);
        return new Pose2d(powerX, powerY, inputPowers.getHeading());
    }



    Pose2d controlledDriving(Pose2d robotTileLocation, Pose2d axisPowers) {

        //telemetry.addData("axisPowers ", axisPowers);
        final double X = robotTileLocation.getX(), Y = robotTileLocation.getY();
        double px = axisPowers.getX(), py = axisPowers.getY();

        if(X < 3.5 && X > 1.5) {
            return InPowers(robotTileLocation, axisPowers);
        }
        if(((X > 4 && axisPowers.getX() < 0) || (X > 1 && axisPowers.getX() > 0)) && abs(axisPowers.getY()) < 0.6) {
            return PassPowers(robotTileLocation, axisPowers);
        }


        if(abs(axisPowers.getY()) > 0.6 && (X < 4 && X > 1)) {
            px = 0;
        }


        final double minPower = 0.1, scalerDrop = 0.35, scalerWall = 0.6;

        if(X > 4) {
            double distanceTo = max(5 - X, 0);

            px = min(px, minPower + scalerDrop * distanceTo);
        } else if (X < 1.5) {
            double distanceTo = max(X - 0.5, 0);
            px = max(px, -minPower - scalerWall * distanceTo);
        }

        if(Y > 4.5) {
            double distanceTo = max(Y - 5.5, 0);
            py = min(py, minPower + scalerWall * distanceTo);
        } else if(Y < 1.5) {
            double distanceTo = max(Y - 0.5, 0);
            py = max(py, -minPower - scalerWall * distanceTo);
        }


        return new Pose2d(px, py, axisPowers.getHeading());

    }




    double realAngle(double angle) {
        if(angle > PI) angle -= 2 * PI;
        return angle;
    }

    public Pose2d locationInTiles() {
        Pose2d pos = getPoseEstimate();
        return new Pose2d(pos.getX() / TILE_LENGTH, pos.getY() / TILE_LENGTH, realAngle(pos.getHeading()));
    }


    public void regularDrive(Pose2d powers) {
        this.setWeightedDrivePower(powers);
        update();
    }


    public Pose2d adjustedPowers(Pose2d Powers) {
        return new Pose2d (Powers.getX() * 1.0, Powers.getY() * 1.13, Powers.getX() * -0.06 + Powers.getHeading() * 1.0);
    }


    public void allDrives(Pose2d inputPowers, boolean adjusted, boolean byAxis, boolean controlled) {
        //opMode.telemetry.addData("input Powers ", inputPowers);
        Pose2d robotTileLocation = locationInTiles();

        if(controlled) {
            inputPowers = controlledDriving(robotTileLocation, inputPowers);
            inputPowers = axisToRobotPowers(inputPowers, robotTileLocation.getHeading());
            opMode.telemetry.addLine("Controlled good");
        }
        else if(byAxis) {
            inputPowers = axisToRobotPowers(inputPowers, robotTileLocation.getHeading());
            opMode.telemetry.addLine("Axis good");
        }

        if(adjusted) {
            inputPowers = adjustedPowers(inputPowers);
            opMode.telemetry.addLine("adjust good");
        }

        //inputPowers = axisToRobotPowers(inputPowers, robotTileLocation.getHeading());

        //opMode.telemetry.addData("final Powers ", inputPowers);
        regularDrive(inputPowers);
    }

    public void setLocationInTiles(double x, double y, double Heading) {
        setPoseEstimate(new Pose2d(x * TILE_LENGTH, y * TILE_LENGTH, Heading));
    }

    public void driveMecanum(Pose2d powers) {

        // in order to make the driving the same velocity for the same power in the x and y directions,
        // reduce the y power slightly
        double y = powers.getX();
        double x = powers.getY();
        double angle = powers.getHeading();
        // Determine how much power each motor should receive.
        double frontRightPower = y + x + angle;
        double frontLeftPower = y - x - angle;
        double backRightPower = y - x + angle;
        double backLeftPower = y + x - angle;

        // The method motor.setPower() only accepts numbers between -1 and 1.
        // If any number that we want to give it is greater than 1,
        // we must divide all the numbers equally so the maximum is 1
        // and the proportions are preserved.
        double norm = max(max(abs(frontRightPower), abs(frontLeftPower)), max(abs(backRightPower), abs(backLeftPower)));
        if (norm > 1) {
            frontRightPower /= norm;
            frontLeftPower /= norm;
            backRightPower /= norm;
            backLeftPower /= norm;
        }

        rightFront.setPower(frontRightPower);
        leftFront.setPower(frontLeftPower);
        rightRear.setPower(backRightPower);
        leftRear.setPower(backLeftPower);

    }

}
