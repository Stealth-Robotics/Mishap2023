package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    private DcMotor clawMainElevation;
    private Servo clawSecondaryElevation;
    private Servo clawMainServo;
    private boolean clawOpen = false;
    private final double OPEN_POS = 1;
    private final double CLOSE_POS = 0;

    public ClawSubsystem(HardwareMap hardwareMap) {

        // grab servo motors
        clawSecondaryElevation = hardwareMap.get(Servo.class, "clawSecondaryElevation");
        clawMainServo = hardwareMap.get(Servo.class, "clawMainServo");

        // grab DC motors
        clawMainElevation = hardwareMap.get(DcMotor.class, "clawMainElevation");
    }

    /*public double getClawSecondaryElevation() {
        return clawSecondaryElevation.getPosition();
    }*/

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
}
