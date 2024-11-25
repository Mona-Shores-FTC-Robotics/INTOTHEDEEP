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
        finished = !action.run(MatchConfig.telemetryPacket); // Set finished to true only when action is complete
    }

    @Override
    public boolean isFinished() {
        return finished;
    }


    //todo see https://rr.brott.dev/docs/v1-0/guides/ftclib-commands/ we probably need to cover the scenario where BetterPrepareAction is cancelled (due to the driver driving?) to stop the motors -
    //  I'm guessing this has something to do with why we had a couple of times where the robot did strange things while the lift was up

    @Override
    public void end(boolean interrupted) {
        if (interrupted && action instanceof DriveToObservationZone) {
            ((DriveToObservationZone) action).cancelAbruptly();
        }
        finished = false; // Reset finished to allow re-initialization on the next press
    }

}