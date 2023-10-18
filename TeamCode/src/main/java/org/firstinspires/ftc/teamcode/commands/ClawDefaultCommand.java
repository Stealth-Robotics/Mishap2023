package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsytems.ClawSubsystem;

import java.util.function.DoubleSupplier;

public class ClawDefaultCommand extends CommandBase {
    final ClawSubsystem clawSubsystem;
    final DoubleSupplier leftY, rightY, leftX, rightX;

    private double inc = 0.1;
    private double sensInc = 0.05;
    private double sensMinClamp = 0.05;
    private double sensMaxClamp = 2;

    public double sensMain;
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

        updateSensitivity();

        clawSubsystem.setClawMainElevation(leftY.getAsDouble() * sensMain);

       /*
       if (rightY.getAsDouble() > deadzone) {
           clawSubsystem.setClawSecondaryElevation(clawSubsystem.getClawSecondaryElevation() + inc);
       } else if (rightY.getAsDouble() < deadzone) {
           clawSubsystem.setClawSecondaryElevation(clawSubsystem.getClawSecondaryElevation() - inc);
       }
        */
    }

    private void updateSensitivity() {
        if (sensMain < sensMaxClamp - inc && leftX.getAsDouble() > 0 + deadzone) {
            sensMain = sensMain + sensInc;
        } else if (sensMain > sensMinClamp + inc && leftX.getAsDouble() < 0) {
            sensMain = sensMain - sensInc;
        }

        if (sensSecond < sensMaxClamp - inc && rightX.getAsDouble() > 0 - deadzone) {
            sensSecond = sensSecond + sensInc;
        } else if (sensSecond > sensMinClamp + inc && rightX.getAsDouble() < 0) {
            sensSecond = sensSecond - sensInc;
        }
    }
}
