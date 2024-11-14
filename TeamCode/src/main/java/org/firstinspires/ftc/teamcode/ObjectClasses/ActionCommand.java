package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.DriveToObservationZone;

import java.util.Set;

public class ActionCommand implements Command {
    private final Action action;
    private final Set<Subsystem> requirements;
    private boolean finished = false;

    public ActionCommand(Action action, Set<Subsystem> requirements) {
        this.action = action;
        this.requirements = requirements;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void execute() {
        action.preview(MatchConfig.telemetryPacket.fieldOverlay());
        finished = action.run(MatchConfig.telemetryPacket); // Set finished to true only when action is complete
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && action instanceof DriveToObservationZone) {
            ((DriveToObservationZone) action).cancelAbruptly();
        }
        finished = false; // Reset finished to allow re-initialization on the next press
    }

}