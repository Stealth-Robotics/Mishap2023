package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsytems.DriveBaseSubsystem;

import java.util.function.DoubleSupplier;

public class DriveDefaultCommand extends CommandBase {
    final DriveBaseSubsystem drive;
    final DoubleSupplier leftX, leftY, rightX;

    public DriveDefaultCommand(DriveBaseSubsystem drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX)
    {
        this.drive = drive;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;

        addRequirements(drive);
    }

    public void execute() {
        drive.drive(leftX.getAsDouble(), leftY.getAsDouble(), rightX.getAsDouble());
    }
}
