package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "StraightMotionOpMode")
public class StraightMotionOpMode extends LinearOpMode {
    DcMotor backLeft, backRight, frontLeft, frontRight;
    final double TicksToCM = 0.0585;

    @Override
    public void runOpMode() {
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
        imu.resetYaw();

        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            double powerSide = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;
            backLeft.setPower(power - powerSide + rotation);
            backRight.setPower(-power + powerSide - rotation);
            frontRight.setPower(power - powerSide - rotation);

            frontLeft.setPower(power + powerSide + rotation);
            double xPosition = (frontLeft.getCurrentPosition()-frontRight.getCurrentPosition()-backLeft.getCurrentPosition()+backRight.getCurrentPosition())/4*TicksToCM;
            double yPosition = (frontLeft.getCurrentPosition()+frontRight.getCurrentPosition()+backLeft.getCurrentPosition()+backRight.getCurrentPosition())/4*TicksToCM;
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double angle = angles.getYaw(AngleUnit.DEGREES);

            telemetry.addData("xPosition: ", xPosition);
            telemetry.addData("yPosition: ", yPosition);
            telemetry.addData("angle: ", angle);
            telemetry.update();


        }
    }

    public void moveForward(double distance){
       double targetTicks = distance/TicksToCM;
       double tickPosition = (frontLeft.getCurrentPosition()+frontRight.getCurrentPosition()+backLeft.getCurrentPosition()+backRight.getCurrentPosition())/4;
    }
}
