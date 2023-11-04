package org.firstinspires.ftc.teamcode.opmodes;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsytems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsytems.DriveBaseSubsystem;
import org.stealthrobotics.library.commands.EndOpModeCommand;
import org.stealthrobotics.library.commands.WaitBeforeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;
@SuppressWarnings("unused")
@Autonomous()
public class FinalAuto extends StealthOpMode {

    private DriveBaseSubsystem drive;
    private ClawSubsystem claw;

    @Override
    public void initialize() {

        drive = new DriveBaseSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);

        register(drive, claw);
    }

    @Override
    public void whileWaitingToStart() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> claw.mainElevationMoveTo(-500)),
                new InstantCommand(() -> drive.moveForward(28)),
                new EndOpModeCommand(this)
        );
    }
}