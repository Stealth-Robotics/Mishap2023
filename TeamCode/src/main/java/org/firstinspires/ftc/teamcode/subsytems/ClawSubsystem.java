package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {
    private Servo clawRotationServo;
    private Servo clawElevationServo;
    private Servo clawMainServo;

    public ClawSubsystem(HardwareMap hardwareMap) {

        // grab servo motors
        clawRotationServo = hardwareMap.get(Servo.class, "clawRotationServo");
        clawElevationServo = hardwareMap.get(Servo.class, "clawElevationServo");
        clawMainServo = hardwareMap.get(Servo.class, "clasMainServo");

        
    }

}
