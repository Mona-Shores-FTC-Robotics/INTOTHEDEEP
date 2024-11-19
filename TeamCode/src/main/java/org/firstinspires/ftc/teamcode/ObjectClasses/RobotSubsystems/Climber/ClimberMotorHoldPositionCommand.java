package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Climber;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

public class ClimberMotorHoldPositionCommand extends CommandBase {

    // The subsystem the command runs on
    private final ClimberSubsystem climberSubsystem;

    //declare target state
    private ClimberSubsystem.ClimberMotorStates targetState;

    public ClimberMotorHoldPositionCommand(ClimberSubsystem subsystem) {
        climberSubsystem = subsystem;
    }

    @Override
    public void initialize() {
        int position = climberSubsystem.climberMotor.getCurrentPosition();
        climberSubsystem.climberMotor.setTargetPosition(position);

        climberSubsystem.climberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void execute() {
    }

    //this only needs to run once to change the state of the intake motor so it can just return true
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        MatchConfig.telemetryPacket.addLine("Climber Motor Hold");
    }
}