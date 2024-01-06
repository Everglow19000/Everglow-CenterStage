package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.round;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import java.util.List;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotParameters;



public class DrivingSystrmModle {

    private LinearOpMode opMode;

    /**
     * Variable used to add multiple add to the telemetry dashboard.
     */

    /**
     * The instance of the robot's IMU.
     */
    private final BNO055IMU imu;
    /**
     * The motors which control the Mecanum Wheels .
     */
    private final DcMotor frontRight, frontLeft, backRight, backLeft;

    private static BNO055IMU initializeImu(LinearOpMode opMode) {
        // Create the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // armadillo and new robot require extra configuration for its IMU.
        // copied from https://ftcforum.firstinspires.org/forum/ftc-technology/53812-mounting-the-revhub-vertically

        byte axisMapConfigByte; //This is what to write to the AXIS_MAP_CONFIG register to swap the needed axis

        byte X_AXIS = 0b0;
        byte Y_AXIS = 0b01;
        byte Z_AXIS = 0b10;

        // IMU configuration explained in: https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf, page 24

        axisMapConfigByte = (byte) (X_AXIS | Z_AXIS << 2 | Y_AXIS << 4); // swap z and y axis

        byte AXIS_MAP_SIGN_BYTE = 0x0; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        // Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal);
        opMode.sleep(100); //Changing modes requires a delay before doing anything else
        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, axisMapConfigByte);
        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE);
        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal);
        opMode.sleep(100); // Changing modes again requires a delay

        ElapsedTime elapsedTime = new ElapsedTime();

        while (!imu.isGyroCalibrated() && elapsedTime.milliseconds() < 2000) {
            // wait for the gyroscope calibration
            opMode.sleep(10);
        }
        return imu;
    }


    public DrivingSystrmModle(LinearOpMode opMode, Pose powers, PointD pointD){
        this.opMode = opMode;
        imu = initializeImu(opMode);
        Telemetry telemetry = new Telemetry();


        // Enable bulk reads


        // Get mecanum wheels interfaces
        frontRight = opMode.hardwareMap.get(DcMotor.class, "front_right");
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "front_left");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "back_left");
        backRight = opMode.hardwareMap.get(DcMotor.class, "back_right");

        // Makes the motors break when their power is set to zero, so they can better stop in place.
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Some motors are wired in reverse, so we must reverse them back.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


    }



    public void driveMecanum(Pose powers){
        // in order to make the driving the same velocity for the same power in the x and y directions,
        // reduce the y power slightly
        double y = powers.y;
        double x = powers.x;
        double angle = powers.angle;
        // Determine how much power each motor should receive.
        double frontRightPower = y + x + angle;
        double frontLeftPower = y - x - angle;
        double backRightPower = y - x + angle;
        double backLeftPower = y + x - angle;

        // The method motor.setPower() only accepts numbers between -1 and 1.
        // If any number that we want to give it is greater than 1,
        // we must divide all the numbers equally so the maximum is 1
        // and the proportions are preserved.
        double norm = max(max(abs(frontRightPower), abs(frontLeftPower)), max(abs(backRightPower), abs(backLeftPower))) / maxDrivePower;
        if (norm > 1) {
            frontRightPower /= norm;
            frontLeftPower /= norm;
            backRightPower /= norm;
            backLeftPower /= norm;
        }

        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);

    }

    public void driveByAxis(Pose powers) {
        final double currentAngle = positionCM.angle;
        final double cosAngle = cos(currentAngle);
        final double sinAngle = sin(currentAngle);

        Pose mecanumPowers = new Pose(
                cosAngle * powers.x - sinAngle * powers.y,
                cosAngle * powers.y + sinAngle * powers.x,
                powers.angle
        );

        driveMecanum(mecanumPowers);
    }

    public void driveWithOuthitting(Pose powers) {
        final double cmPerTile = 60;
        final Pose positionTile = new Pose(positionCM.x / cmPerTile, positionCM.y / cmPerTile, positionCM.angle);
        if (positionTile.y > 5 || positionTile.y < 2) {
            driveByAxis(powers);
            return;
        }

        if ((positionTile.y > 4.5 && powers.y > -0.1) || (positionTile.y < 2.5 && powers.y < 0.1)) {
            driveByAxis(powers);
            return;
        }

        if (positionTile.x < 3.8 && positionTile.x > 2.2) {
            driveByAxis(powers);
            return;
        }

        double targetYTile = 0.5;
        if (positionTile.x > 1) targetYTile = 1.5;
        if (positionTile.x > 3) targetYTile = 4.5;
        if (positionTile.x > 5) targetYTile = 5.5;

        double parameter = 4;

        powers.x = (targetYTile - positionTile.x) * (targetYTile - positionTile.x) * parameter;
        driveByAxis(powers);
    }
}
