package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsytems.ClawSubsystem;

import java.util.function.DoubleSupplier;

public class ClawDefaultCommand extends CommandBase {
    final ClawSubsystem clawSubsystem;

    final DoubleSupplier leftX, leftY;

    private double inc = 0.1;
    private double threshhold = 0.05;

    ClawDefaultCommand(DoubleSupplier leftX, DoubleSupplier leftY, ClawSubsystem clawSubsystem) {
        this.leftX = leftX;
        this.leftY = leftY;

        this.clawSubsystem = clawSubsystem;

        addRequirements(clawSubsystem);
    }

    @Override
    public void execute() {

        if (leftX.getAsDouble() < threshhold) {
            clawSubsystem.setClawRotationServo(clawSubsystem.getClawRotation() - inc);
        }

        if (leftX.getAsDouble() > threshhold) {
            clawSubsystem.setClawRotationServo((clawSubsystem.getClawRotation()) + inc);
        }

        if (leftY.getAsDouble() < threshhold) {
            clawSubsystem.setClawElevationServo(clawSubsystem.getClawElevation() - inc);
        }

        if (leftY.getAsDouble() > threshhold) {
            clawSubsystem.setClawElevationServo((clawSubsystem.getClawElevation()) + inc);
        }


    }

}
