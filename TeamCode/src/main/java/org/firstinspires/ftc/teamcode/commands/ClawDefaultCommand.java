package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsytems.ClawSubsystem;

import java.util.function.DoubleSupplier;

public class ClawDefaultCommand extends CommandBase {
    final ClawSubsystem clawSubsystem;
    final DoubleSupplier leftY, rightY;

    private double inc = 0.1;
    private double deadzone = 0.05;

    public ClawDefaultCommand(ClawSubsystem clawSubsystem, DoubleSupplier rightY, DoubleSupplier leftY) {
        this.rightY = rightY;
        this.leftY = leftY;

        this.clawSubsystem = clawSubsystem;

        addRequirements(clawSubsystem);
    }

    @Override
    public void execute() {

       clawSubsystem.setClawMainElevation(leftY.getAsDouble());

       /*
       if (rightY.getAsDouble() > deadzone) {
           clawSubsystem.setClawSecondaryElevation(clawSubsystem.getClawSecondaryElevation() + inc);
       } else if (rightY.getAsDouble() < deadzone) {
           clawSubsystem.setClawSecondaryElevation(clawSubsystem.getClawSecondaryElevation() - inc);
       }
        */
    }
}
