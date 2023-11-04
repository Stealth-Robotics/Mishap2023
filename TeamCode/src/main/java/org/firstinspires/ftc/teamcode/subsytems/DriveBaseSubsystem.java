package org.firstinspires.ftc.teamcode.subsytems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveBaseSubsystem extends SubsystemBase {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private double rotX = 0;
    private double rotY = 0;
    private double leftX;

    private boolean robotCentric;

    private double headingOffset;

    BNO055IMU imu;

    Vector2d input;

    public DriveBaseSubsystem(HardwareMap hardwareMap)
    {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Check directions and reverse if needed--
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set default behavior when no power
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        resetHeading();

        headingOffset = -(Math.PI / 2);
    }

    public void resetHeading() {
        headingOffset = imu.getAngularOrientation().firstAngle - (Math.PI / 2);
    }

    public double getAngle() {
        return -imu.getAngularOrientation().firstAngle + headingOffset;
    }

    public void drive(double leftStickY, double leftStickX, double rightStickX)
    {
        // Scale down input
        leftStickX *= 0.7;
        leftStickY *= 0.7;
        rightStickX *= 0.7;

        if (robotCentric) {
            resetHeading();
        }

        leftX = leftStickX;
        double rotation = rightStickX;

        rotX = leftX * Math.cos(getAngle()) - leftStickY * Math.sin(getAngle());
        rotY = leftX * Math.sin(getAngle()) + leftStickY * Math.cos(getAngle());

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1);
        double frontLeftPower = (rotY + rotX + rotation) / denominator;
        double backLeftPower = (rotY - rotX + rotation) / denominator;
        double frontRightPower = (rotY - rotX - rotation) / denominator;
        double backRightPower = (rotY + rotX - rotation) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void moveRight(int clicks) {
        double zero = -frontRight.getCurrentPosition();

        while (zero + frontRight.getCurrentPosition() < clicks) {
            leftX = 1;
        }
    }


    public void moveForward(int clicks) {

        double zero = -frontRight.getCurrentPosition();
        // either 28 or 537.7 per rev
        if (zero + frontRight.getCurrentPosition() < clicks) {
            frontLeft.setPower(1);
            frontRight.setPower(1);
            backLeft.setPower(1);
            backRight.setPower(1);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    public void rotate(double angle) {

    }

    public void toggleRobotCentric() {
        robotCentric = !robotCentric;
    }

    @Override
    public void periodic() {
        telemetry.addData("Robot Heading: ", getAngle());
        telemetry.addData("Heading offset: ", headingOffset);
    }
}
