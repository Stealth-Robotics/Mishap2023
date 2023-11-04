package org.firstinspires.ftc.teamcode.subsytems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.DoubleSupplier;

public class ClawSubsystem extends SubsystemBase {

    private final DcMotor clawMainElevation;
    private final Servo clawSecondaryElevation;
    private final Servo clawMainServo;
    private boolean clawOpen = false;
    private final double OPEN_POS = 0.65;
    private final double CLOSE_POS = 0;
    private Double leftY, rightY, leftX, rightX;

    private final double sensMinClamp = 0.1;
    private final double sensMaxClamp = 1;

    private double sensMain = (sensMaxClamp + sensMinClamp) / 2;

    private final double secondaryElevationMin = 0.0;
    private final double secondaryElevationMax = 0.65;

    private final double deadzone = 0.05;

    public ClawSubsystem(HardwareMap hardwareMap) {

        // grab servo motors
        clawSecondaryElevation = hardwareMap.get(Servo.class, "clawSecondaryElevation");
        clawMainServo = hardwareMap.get(Servo.class, "clawMainServo");

        // grab DC motors
        clawMainElevation = hardwareMap.get(DcMotor.class, "clawMainElevation");

        clawMainElevation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawMainServo.setPosition(0);

        setClawSecondaryElevation(0.5);
    }

    public void claw(DoubleSupplier rightY, DoubleSupplier
            leftY, DoubleSupplier rightX, DoubleSupplier leftX) {

        this.leftY = leftY.getAsDouble();
        this.rightY = rightY.getAsDouble();
        this.leftX = leftX.getAsDouble();
        this.rightX = rightX.getAsDouble();

        updateSensitivity();

        updateLocations();
    }

    private void updateLocations() {
        setClawMainElevation(leftY * sensMain);

        double inc = 0.025;
        double val;

        if (rightY > deadzone) {
            val = getClawSecondaryElevation() + inc;
            val = Math.max(secondaryElevationMin, Math.min(secondaryElevationMax, val));
            setClawSecondaryElevation(val);
        } else if (rightY < -deadzone) {
            val = getClawSecondaryElevation() - inc;
            val = Math.max(secondaryElevationMin, Math.min(secondaryElevationMax, val));
            setClawSecondaryElevation(val);
        }
    }

    private void updateSensitivity() {

        double sensInc = 0.05;
        if (leftX > deadzone) {
            sensMain += sensInc;
        } else if (leftX < -deadzone) {
            sensMain -= sensInc;
        }

        sensMain = Math.max(sensMinClamp, Math.min(sensMaxClamp, sensMain));
    }

    public double getClawSecondaryElevation() {
        return clawSecondaryElevation.getPosition();
    }

    public boolean getClawOpen() {
        return clawOpen;
    }

    public void toggleClaw() {
        clawOpen = !clawOpen;
        clawMainServo.setPosition(clawOpen ? OPEN_POS : CLOSE_POS);
    }

    public void setClawSecondaryElevation(double input) {
        clawSecondaryElevation.setPosition(input);
    }


    public void setClawMainElevation(double input) {
        clawMainElevation.setPower(input);
    }

    public void openClaw() {
        clawMainServo.setPosition(OPEN_POS);
    }

    public void closeClaw() {
        clawMainServo.setPosition(CLOSE_POS);
    }

    public void periodic() {
        telemetry.addData("Claw Open: ", clawOpen);
        telemetry.addData("Right Y: ", rightY);
        telemetry.addData("Secondary Elevation: ", getClawSecondaryElevation());
    }
}
