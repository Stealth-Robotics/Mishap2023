package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsytems.ClawSubsystem;

import java.util.function.DoubleSupplier;

public class ClawDefaultCommand extends CommandBase {
    final ClawSubsystem clawSubsystem;
    final DoubleSupplier leftY, rightY, leftX, rightX;

    private double inc = 0.1;

    private double sensMain;
    private double sensSecond;

    private double deadzone = 0.05;

    public ClawDefaultCommand(ClawSubsystem clawSubsystem, DoubleSupplier rightY, DoubleSupplier
            leftY, DoubleSupplier rightX, DoubleSupplier leftX) {
        this.rightY = rightY;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;

        this.clawSubsystem = clawSubsystem;

        addRequirements(clawSubsystem);
    }

    @Override
    public void execute() {

       clawSubsystem.setClawMainElevation(leftY.getAsDouble() * sensMain);

       updateSensitivity();


       /*
       if (rightY.getAsDouble() > deadzone) {
           clawSubsystem.setClawSecondaryElevation(clawSubsystem.getClawSecondaryElevation() + inc);
       } else if (rightY.getAsDouble() < deadzone) {
           clawSubsystem.setClawSecondaryElevation(clawSubsystem.getClawSecondaryElevation() - inc);
       }
        */
    }

    private void updateSensitivity() {
        if (sensMain < 1 && leftX.getAsDouble() > 0) {
            sensMain = sensMain + inc;
        } else if (sensMain > 0.1 && leftX.getAsDouble() < 0) {
            sensMain = sensMain - inc;
        }

        if (sensSecond < 1 && rightX.getAsDouble() > 0) {
            sensSecond = sensSecond + inc;
        } else if (sensSecond > 0.1 && rightX.getAsDouble() < 0) {
            sensSecond = sensSecond - inc;
        }
    }
}
