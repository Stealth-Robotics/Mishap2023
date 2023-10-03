package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {
    private Servo clawRotationServo;
    private Servo clawElevationServo;
    private Servo clawMainServo;
    private boolean clawOpen = false;
    private final double OPEN_POS = 1;
    private final double CLOSE_POS = 0;

    public ClawSubsystem(HardwareMap hardwareMap) {

        // grab servo motors
        clawRotationServo = hardwareMap.get(Servo.class, "clawRotationServo");
        clawElevationServo = hardwareMap.get(Servo.class, "clawElevationServo");
        clawMainServo = hardwareMap.get(Servo.class, "clawMainServo");
    }

    public double getClawElevation() {
        return clawElevationServo.getPosition();
    }

    public double getClawRotation() {
        return clawRotationServo.getPosition();
    }

    public boolean getClawOpen() {
        return clawOpen;
    }

    public void toggleClaw() {
        clawOpen = !clawOpen;
        clawMainServo.setPosition(clawOpen ? OPEN_POS : CLOSE_POS);
    }

    public void setClawRotationServo(double input) {
        clawRotationServo.setPosition(input);
    }

    public void setClawElevationServo(double input) {
        clawElevationServo.setPosition(input);
    }

    public void openClaw() {
        clawMainServo.setPosition(OPEN_POS);
    }

    public void closeClaw() {
        clawMainServo.setPosition(CLOSE_POS);
    }
}
