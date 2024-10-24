package org.firstinspires.ftc.teamcode.ObjectClasses.Deprecated;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

public class RotateShoulderCommand extends CommandBase {
    // The subsystem the command runs on
    private final ShoulderSubsystem shoulderSubsystem;

    //declare target state & position
    private ShoulderSubsystem.ShoulderStates targetState;

    public RotateShoulderCommand(ShoulderSubsystem subsystem, ShoulderSubsystem.ShoulderStates inputState) {
            shoulderSubsystem = subsystem;
            targetState = inputState;

            //require this subsystem
            addRequirements(shoulderSubsystem);
        }

    @Override
    public void initialize() {
        shoulderSubsystem.shoulder.setPosition(targetState.position);
    }

    public void execute() {
    }
    @Override
    public boolean isFinished() {
        //always return true because the command simply sets the servo and we have no way of telling when the servo has finished moving
        return true;
    }
    @Override
    public void end(boolean interrupted) {
        MatchConfig.telemetryPacket.addLine("Shoulder Move COMPLETE From " + shoulderSubsystem.currentState + " to " + targetState);
        //change the current state to the target state
        shoulderSubsystem.setCurrentState(targetState);
    }
}
