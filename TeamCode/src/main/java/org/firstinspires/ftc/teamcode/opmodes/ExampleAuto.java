package org.firstinspires.ftc.teamcode.opmodes;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.stealthrobotics.library.commands.EndOpModeCommand;
import org.stealthrobotics.library.commands.WaitBeforeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;
@SuppressWarnings("unused")
@Autonomous()
public abstract class ExampleAuto extends StealthOpMode {


    @Override
    public void initialize() {

        //add all your subsystems to this
        //register(drive, elevator, clawper, intake);
    }

    @Override
    public void whileWaitingToStart() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                //put all commands before this
                //example new InstantCommand(() -> fjdlksafjlds),
                //separate with commas
                new InstantCommand(),
                new InstantCommand(),
                new EndOpModeCommand(this)
        );
    }


}