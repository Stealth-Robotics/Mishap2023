package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.subsytems.DriveBase;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class Teleop extends StealthOpMode {
    DriveBase driveSubsystem;

    GamepadEx driverGamepad;
    GamepadEx opperatorGamepad;


    public void initialize()
    {
        driveSubsystem = new DriveBase(hardwareMap);

        driverGamepad = new GamepadEx(gamepad1);
        opperatorGamepad = new GamepadEx(gamepad2);

        driveSubsystem.setDefaultCommand(
                new DriveDefaultCommand(
                        driveSubsystem,
                        () -> driverGamepad.getLeftX(),
                        () -> driverGamepad.getLeftY(),
                        () -> driverGamepad.getRightX()
                )
        );
    }
}
