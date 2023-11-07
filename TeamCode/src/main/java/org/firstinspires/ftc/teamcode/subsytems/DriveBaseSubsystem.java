package org.firstinspires.ftc.teamcode.subsytems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveBaseSubsystem extends SubsystemBase {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    private double rotX = 0;
    private double rotY = 0;
    private boolean robotCentric = false;
    private boolean reverseHeading = false;

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

        if (!robotCentric && reverseHeading) {
            headingOffset += Math.PI;
        }

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
            if (reverseHeading) {
                headingOffset += Math.PI;
            }
        }

        double x = leftStickX;
        double y = -leftStickY;
        double rotation = rightStickX;

        rotX = x * Math.cos(getAngle()) - y * Math.sin(getAngle());
        rotY = x * Math.sin(getAngle()) + y * Math.cos(getAngle());

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1);

        frontLeft.setPower((rotY + rotX + rotation) / denominator);
        backLeft.setPower((rotY - rotX + rotation) / denominator);
        frontRight.setPower((rotY - rotX - rotation) / denominator);
        backRight.setPower((rotY + rotX - rotation) / denominator);
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

    public void toggleRobotCentric() {
        robotCentric = !robotCentric;
    }

    public void flipHeading() {
        reverseHeading = !reverseHeading;
    }

    @Override
    public void periodic() {
        telemetry.addData("Robot Heading: ", getAngle());
        telemetry.addData("Heading offset: ", headingOffset);
        telemetry.addData("Is robot centric: ", robotCentric);
    }
}
