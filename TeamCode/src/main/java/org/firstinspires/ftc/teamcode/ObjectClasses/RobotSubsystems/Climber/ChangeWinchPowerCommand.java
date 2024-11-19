package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Climber;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

public class ChangeWinchPowerCommand extends CommandBase {

    // The subsystem the command runs on
    private final ClimberSubsystem climberSubsystem;

    //declare target state
    private final ClimberSubsystem.WinchMotorStates targetState;

    public ChangeWinchPowerCommand(ClimberSubsystem subsystem, ClimberSubsystem.WinchMotorStates inputState) {
        climberSubsystem = subsystem;
        targetState = inputState;
    }

    @Override
    public void initialize() {

        climberSubsystem.winchMotor.setPower(targetState.getPower());
    }

    //this only needs to run once to change the state of the intake motor so it can just return true
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        MatchConfig.telemetryPacket.addLine("End Game Winch changed from" + climberSubsystem.currentWinchMotorState + " to " + targetState);
        //change the current state to the target state
        climberSubsystem.setCurrentWinchMotorState(targetState);
    }
}