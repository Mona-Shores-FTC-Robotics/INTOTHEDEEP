package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

public class ReleaseDroneCommand extends CommandBase {
    private final DroneSubsystem droneSubsystem;
    private DroneSubsystem.DroneDeployState targetState;

    public ReleaseDroneCommand(DroneSubsystem subsystem, DroneSubsystem.DroneDeployState inputState) {
        droneSubsystem = subsystem;
        targetState = inputState;

        //require this subsystem
        addRequirements(droneSubsystem);
    }

    @Override
    public void initialize() {droneSubsystem.drone.setPosition(targetState.position);}

    public void execute() {
    }

    @Override
    public boolean isFinished() {
        droneSubsystem.currentState = targetState;
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        //change the current state to the target state
        MatchConfig.telemetryPacket.addLine("Drone move COMPLETE From " + droneSubsystem.currentState + " to " + targetState);
        droneSubsystem.setCurrentState(targetState);
    }

}
