package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public class ChangeIntakePowerCommand extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem intakeSubsystem;

    //declare target state
    private IntakeSubsystem.IntakeStates targetState1;
    private IntakeSubsystem.IntakeStates targetState2;

    public ChangeIntakePowerCommand(IntakeSubsystem subsystem, IntakeSubsystem.IntakeStates inputState1, IntakeSubsystem.IntakeStates inputState2) {
        intakeSubsystem = subsystem;
        targetState1 = inputState1;
        targetState2 = inputState2;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        intakeSubsystem.intake1.setPower(targetState1.power);
        intakeSubsystem.intake2.setPower(targetState2.power);
    }

    public void execute() {
    }

    //this only needs to run once to change teh state of the intake motor so it can just return true
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        MatchConfig.telemetryPacket.addLine("Intake 1 changed from" + intakeSubsystem.currentIntake1State + " to " + targetState1);
        MatchConfig.telemetryPacket.addLine("Intake 2 changed from" + intakeSubsystem.currentIntake2State + " to " + targetState2);
        //change the current state to the target state
        intakeSubsystem.setCurrentIntake1State(targetState1);
        intakeSubsystem.setCurrentIntake2State(targetState2);
    }
}
