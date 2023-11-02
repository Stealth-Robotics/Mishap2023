package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsytems.ClawSubsystem;

import java.util.function.DoubleSupplier;

public class ClawDefaultCommand extends CommandBase {
    final ClawSubsystem clawSubsystem;
    final DoubleSupplier leftY, rightY, leftX, rightX;



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
        clawSubsystem.claw(rightY, leftY, rightX, leftX);
    }


}
