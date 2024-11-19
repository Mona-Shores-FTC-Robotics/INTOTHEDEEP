package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Climber;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

public class ChangeClimberMotorPowerCommand extends CommandBase {

    // The subsystem the command runs on
    private final ClimberSubsystem climberSubsystem;

    //declare target state
    private final ClimberSubsystem.ClimberMotorStates targetState;

    public ChangeClimberMotorPowerCommand(ClimberSubsystem subsystem, ClimberSubsystem.ClimberMotorStates inputState) {
        climberSubsystem = subsystem;
        targetState = inputState;
    }

    @Override
    public void initialize() {

        climberSubsystem.climberMotor.setPower(targetState.getPower());
    }

    //this only needs to run once to change the state of the intake motor so it can just return true
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        MatchConfig.telemetryPacket.addLine("Climber Motor changed from" + climberSubsystem.currentClimberMotorState + " to " + targetState);
        //change the current state to the target state
        climberSubsystem.setCurrentClimberMotorState(targetState);
    }
}