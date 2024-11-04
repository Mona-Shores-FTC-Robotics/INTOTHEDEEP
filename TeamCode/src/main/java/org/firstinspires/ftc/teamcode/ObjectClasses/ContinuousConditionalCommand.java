package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.Supplier;

public class ContinuousConditionalCommand extends CommandBase {
    private final Supplier<Boolean> condition;
    private final Command trueCommand;
    private final Command falseCommand;
    private Command activeCommand;

    public ContinuousConditionalCommand(Command trueCommand, Command falseCommand, Supplier<Boolean> condition) {
        this.condition = condition;
        this.trueCommand = trueCommand;
        this.falseCommand = falseCommand;
    }

    @Override
    public void initialize() {
        // Start with the command based on the initial condition
        activeCommand = condition.get() ? trueCommand : falseCommand;
        activeCommand.initialize();
    }

    @Override
    public void execute() {
        // Continuously check the condition and switch commands if it changes
        if (condition.get() && activeCommand != trueCommand) {
            // End the current falseCommand and switch to trueCommand
            activeCommand.end(false);
            activeCommand = trueCommand;
            activeCommand.initialize();
        } else if (!condition.get() && activeCommand != falseCommand) {
            // End the current trueCommand and switch to falseCommand
            activeCommand.end(false);
            activeCommand = falseCommand;
            activeCommand.initialize();
        }
        activeCommand.execute();
    }

    @Override
    public boolean isFinished() {
        // Only complete when the condition is true and trueCommand has finished
        return condition.get() && activeCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        activeCommand.end(interrupted);
    }
}

