package org.firstinspires.ftc.teamcode.subsytems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class ClawSubsystem extends SubsystemBase {


    // Motors
    private final DcMotor clawMainElevation;

    // Servos
    private final Servo clawSecondaryElevation;
    private final Servo leftClaw;
    private final Servo rightClaw;

    // Servo States
    private boolean leftClawOpen = false;
    private boolean rightClawOpen = false;

    // Servo Constants
    private final double OPEN_POS = 0.65;
    private final double CLOSE_POS = 0;
    private final double secondaryElevationMin = 0.0;
    private final double secondaryElevationMax = 1;

    // Sensitivity Constants
    private final double sensMinClamp = 0.1;
    private final double sensMaxClamp = 1;
    private double sensMain = (sensMaxClamp + sensMinClamp) / 2;

    // Inputs
    private Double leftY, rightY, leftX, rightX;
    private final double deadzone = 0.05;

    public ClawSubsystem(HardwareMap hardwareMap) {

        // grab servo motors
        clawSecondaryElevation = hardwareMap.get(Servo.class, "clawSecondaryElevation");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        // grab DC motors
        clawMainElevation = hardwareMap.get(DcMotor.class, "clawMainElevation");

        // set zero power behavior
        clawMainElevation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public boolean getLeftClawOpen() {
        return leftClawOpen;
    }

    public boolean getRightClawOpen() {
        return rightClawOpen;
    }

    public void toggleLeftClaw() {
        leftClawOpen = !leftClawOpen;
        leftClaw.setPosition(leftClawOpen ? OPEN_POS : CLOSE_POS);
    }

    public void toggleRightClaw() {
        rightClawOpen = !rightClawOpen;
        rightClaw.setPosition(rightClawOpen ? OPEN_POS : CLOSE_POS);
    }

    public void setClawSecondaryElevation(double input) {
        clawSecondaryElevation.setPosition(input);
    }

    public void setClawMainElevation(double input) {
        clawMainElevation.setPower(input);
    }

     public void mainElevationMoveTo(int clicks) {
        int near = 25;

        while (clawMainElevation.getCurrentPosition() > clicks + near ||
                clawMainElevation.getCurrentPosition() < clicks - near) {
            if (clawMainElevation.getCurrentPosition() > clicks) {
                clawMainElevation.setPower(-1);
            } else if (clawMainElevation.getCurrentPosition() < clicks) {
                clawMainElevation.setPower(1);
            } else {
                clawMainElevation.setPower(0);
            }
        }
    }

    public void openLeftClaw() {
        leftClawOpen = true;
        leftClaw.setPosition(OPEN_POS);
    }

    public void openRightClaw() {
        rightClawOpen = true;
        rightClaw.setPosition(OPEN_POS);
    }

    public void closeLeftClaw() {
        leftClawOpen = false;
        leftClaw.setPosition(CLOSE_POS);
    }

    public void closeRightClaw() {
        rightClawOpen = false;
        rightClaw.setPosition(CLOSE_POS);
    }

    public void periodic() {
        //telemetry.addData("Main Elevation position: ", clawMainElevation.getCurrentPosition());
        telemetry.addData("Secondary Elevation position: ", clawSecondaryElevation.getPosition());
        telemetry.addData("Left is open: ", leftClawOpen);
        telemetry.addData("Right is open: ", rightClawOpen);
    }
}
