package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift.SampleLiftSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;

public class SampleHandlingStateMachine {

    private final SampleLinearActuatorSubsystem actuatorSubsystem;
    private final SampleIntakeSubsystem intakeSubsystem;
    private final SampleLiftSubsystem liftSubsystem;

    // Constructor
    public SampleHandlingStateMachine(SampleLinearActuatorSubsystem actuatorSubsystem,
                                      SampleIntakeSubsystem intakeSubsystem,
                                      SampleLiftSubsystem liftSubsystem) {
        this.actuatorSubsystem = actuatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.liftSubsystem = liftSubsystem;
    }

    public SampleHandlingStateMachine(SampleLinearActuatorSubsystem actuatorSubsystem,
                                      SampleIntakeSubsystem intakeSubsystem
                                      ) {
        this.actuatorSubsystem = actuatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.liftSubsystem = null;
    }

    // Method to handle button press logic (toggle actuator states)
    public void onIntakeButtonPress() {
        switch (actuatorSubsystem.getCurrentState()) {
            case RETRACT:
                setActuatorState(SampleLinearActuatorSubsystem.SampleActuatorStates.DEPLOY_MID);
                setIntakeState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);
                break;
            case DEPLOY_MID:
                setActuatorState(SampleLinearActuatorSubsystem.SampleActuatorStates.DEPLOY_FULL);
                break;
            case DEPLOY_FULL:
            default:
                setActuatorState(SampleLinearActuatorSubsystem.SampleActuatorStates.RETRACT);
                setIntakeState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
                break;


        }
    }

    // Method to handle piece pickup and expel sequence
    public void onGoodSampleDetected() {
        SequentialCommandGroup retractAndExpelSequence = new SequentialCommandGroup(
                new InstantCommand(() -> setActuatorState(SampleLinearActuatorSubsystem.SampleActuatorStates.RETRACT)),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(this::setIntakeReverse),
                                new WaitCommand(500),
                                new InstantCommand(this::setIntakeOff),
                                new InstantCommand(this::setLiftToLowBasket)
                        ),
                        new WaitCommand(100),
                        this::isActuatorRetracted
                )
        );
        retractAndExpelSequence.schedule();
    }

    public void onBadSampleDetected() {
        SequentialCommandGroup expelPieceSequence = new SequentialCommandGroup(
                new InstantCommand(() -> setIntakeState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_REVERSE)),  // Reverse intake
                new WaitCommand(500),  // Wait for the piece to be expelled
                new InstantCommand(() -> setIntakeState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON)),  // Resume intaking
                new InstantCommand(() -> setActuatorState(SampleLinearActuatorSubsystem.SampleActuatorStates.DEPLOY_MID))  // Set actuator back to mid position
        );
        expelPieceSequence.schedule();
    }


    private boolean isActuatorRetracted() {
        return actuatorSubsystem.isActuatorAtTarget();
    }

    private void setActuatorState(SampleLinearActuatorSubsystem.SampleActuatorStates newState) {
        actuatorSubsystem.setTargetState(newState);
    }

    private void setIntakeState(SampleIntakeSubsystem.SampleIntakeStates newState) {
        intakeSubsystem.setCurrentState(newState);
    }

    // Method to turn the intake on
    public void setIntakeOn() {
        setIntakeState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);
    }

    // Method to turn the intake off
    public void setIntakeOff() {
        setIntakeState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
    }

    // Method to reverse the intake
    public void setIntakeReverse() {
        setIntakeState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_REVERSE);
    }

    private void setLiftState(SampleLiftSubsystem.SampleLiftStates newState) {
        liftSubsystem.setTargetState(newState);
    }

    public void setLiftToHighBasket() {
        // Check if the actuator is fully retracted before moving the lift
        if (actuatorSubsystem.isFullyRetracted()) {  // Assuming this returns true when retracted
            setLiftState(SampleLiftSubsystem.SampleLiftStates.HIGH_BASKET);
        } else {
            System.out.println("Cannot raise lift: Actuator is not fully retracted.");
        }
    }

    public void setLiftToLowBasket() {
        // Check if the actuator is fully retracted before moving the lift
        if (actuatorSubsystem.isFullyRetracted()) {  // Assuming this returns true when retracted
            setLiftState(SampleLiftSubsystem.SampleLiftStates.LOW_BASKET);
        } else {
            System.out.println("Cannot raise lift: Actuator is not fully retracted.");
        }
    }

    public void setLiftToHome() {
        setLiftState(SampleLiftSubsystem.SampleLiftStates.HOME);
    }
}
