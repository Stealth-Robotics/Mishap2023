package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.subsytems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsytems.DriveBaseSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class Teleop extends StealthOpMode {
    DriveBaseSubsystem driveBaseSubsystem;
    ClawSubsystem clawSubsystem;
    GamepadEx driverGamepad;
    GamepadEx opperatorGamepad;


    public void initialize()
    {
        driveBaseSubsystem = new DriveBaseSubsystem(hardwareMap);
        clawSubsystem = new ClawSubsystem(hardwareMap);

        driverGamepad = new GamepadEx(gamepad1);
        opperatorGamepad = new GamepadEx(gamepad2);

        driveBaseSubsystem.setDefaultCommand(
                new DriveDefaultCommand(
                        driveBaseSubsystem,
                        () -> driverGamepad.getLeftX(),
                        () -> driverGamepad.getLeftY(),
                        () -> driverGamepad.getRightX()
                )
        );

        opperatorGamepad.getGamepadButton(GamepadKeys.Button.A).
                whenPressed(new InstantCommand(() -> clawSubsystem.toggleClaw()));

    }
}
