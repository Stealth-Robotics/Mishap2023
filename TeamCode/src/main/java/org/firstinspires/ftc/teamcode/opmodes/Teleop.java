package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ClawDefaultCommand;
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
                        () -> driverGamepad.getLeftY(),
                        () -> driverGamepad.getLeftX(),
                        () -> -driverGamepad.getRightX()
                )
        );

        clawSubsystem.setDefaultCommand(
                new ClawDefaultCommand(
                        clawSubsystem,
                        () -> opperatorGamepad.getRightY(),
                        () -> -opperatorGamepad.getLeftY(),
                        () -> opperatorGamepad.getRightX(),
                        () -> opperatorGamepad.getLeftX()
                )
        );

        opperatorGamepad.getGamepadButton(GamepadKeys.Button.A).
                whenPressed(new InstantCommand(() -> clawSubsystem.toggleClaw()));

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).
                whenPressed(new InstantCommand(() -> driveBaseSubsystem.resetHeading()));

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).//Hi Joey!! I <3 U!
                whenPressed(new InstantCommand(() -> driveBaseSubsystem.toggleRobotCentric()));
    }

    @SuppressWarnings("unused")
    @TeleOp(name = "RED | Telo-Op", group = "Red")
    public static class RedTeleop extends Teleop {
    }

    @SuppressWarnings("unused")
    @TeleOp(name = "BLUE | Telo-Op", group = "Blue")
    public static class BlueTeleop extends Teleop {
    }


}
