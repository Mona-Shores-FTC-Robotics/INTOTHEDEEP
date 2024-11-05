package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift.SampleLiftBucketSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;

public class SampleHandlingStateMachine {

    private final SampleLinearActuatorSubsystem actuatorSubsystem;
    private final SampleIntakeSubsystem intakeSubsystem;
    private final SampleLiftBucketSubsystem liftSubsystem;

    // Constructor
    public SampleHandlingStateMachine(SampleLinearActuatorSubsystem actuatorSubsystem,
                                      SampleIntakeSubsystem intakeSubsystem,
                                      SampleLiftBucketSubsystem liftSubsystem) {
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

            case WITHOUT_ENCODER:
                actuatorSubsystem.enableRunToPositionMode();
                setActuatorTargetState(SampleLinearActuatorSubsystem.SampleActuatorStates.RETRACT);
                setIntakeState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
                break;

            case RETRACT:
                setActuatorTargetState(SampleLinearActuatorSubsystem.SampleActuatorStates.DEPLOY_MID);
                setIntakeState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);
                break;
            case DEPLOY_MID:
                setActuatorTargetState(SampleLinearActuatorSubsystem.SampleActuatorStates.DEPLOY_FULL);
                break;
            case DEPLOY_FULL:
            case MANUAL:
            default:
                setActuatorTargetState(SampleLinearActuatorSubsystem.SampleActuatorStates.RETRACT);
                setIntakeState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
                break;


        }
    }

    // Method to handle piece pickup and expel sequence
    public void onGoodSampleDetected() {
        SequentialCommandGroup retractAndExpelSequence = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(this::setIntakeOff),
                        new InstantCommand(actuatorSubsystem::runWithoutEncodersReverse)
                        ),

                new WaitCommand(350),
                new InstantCommand(actuatorSubsystem::stopActuator),
                new InstantCommand(this::setIntakeReverse),
                new WaitCommand(300),
                new InstantCommand(this::setIntakeOff));

//                new ContinuousConditionalCommand(
//                        new SequentialCommandGroup(
//                                new InstantCommand(this::setIntakeReverse),
//                                new WaitCommand(500),
//                                new InstantCommand(this::setIntakeOff)
////                              new InstantCommand(this::setLiftToLowBasket)
//                        ),
//                        new WaitCommand(100),
//                        this::isActuatorRetracted
//                )
        retractAndExpelSequence.schedule();
    }

    public void onBadSampleDetected() {
        SequentialCommandGroup expelPieceSequence = new SequentialCommandGroup(
                new InstantCommand(this::setIntakeReverse),  // Reverse intake
                new WaitCommand(250),  // Wait for the piece to be expelled
                new InstantCommand(actuatorSubsystem::runWithoutEncodersReverse), // move the actuator back a little bit
                new WaitCommand(250),
                new ParallelCommandGroup(
                    new InstantCommand(this::setIntakeOn), // Resume intaking
                    new InstantCommand(actuatorSubsystem::stopActuator)// move the actuator back a little bit
                )
        );
        expelPieceSequence.schedule();

//        SequentialCommandGroup expelPieceSequence = new SequentialCommandGroup(
//                new InstantCommand(() -> setIntakeState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_REVERSE)),  // Reverse intake
//                new WaitCommand(500),  // Wait for the piece to be expelled
//                new InstantCommand(() -> setIntakeState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON)),  // Resume intaking
//                new InstantCommand(() -> setActuatorTargetState(SampleLinearActuatorSubsystem.SampleActuatorStates.DEPLOY_MID))  // Set actuator back to mid position
//        );
//        expelPieceSequence.schedule();
    }


    private boolean isActuatorRetracted() {
        return actuatorSubsystem.isActuatorAtTarget();
    }

    private void setActuatorTargetState(SampleLinearActuatorSubsystem.SampleActuatorStates newState) {
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

    private void setLiftState(SampleLiftBucketSubsystem.SampleLiftStates newState) {
        liftSubsystem.setTargetLiftState(newState);
    }

    public void setLiftToHighBasket() {
        // Check if the actuator is fully retracted before moving the lift
        if (actuatorSubsystem.isFullyRetracted()) {  // Assuming this returns true when retracted
            setLiftState(SampleLiftBucketSubsystem.SampleLiftStates.HIGH_BASKET);
        } else {
            System.out.println("Cannot raise lift: Actuator is not fully retracted.");
        }
    }

    public void setLiftToLowBasket() {
        // Check if the actuator is fully retracted before moving the lift
        if (actuatorSubsystem.isFullyRetracted()) {  // Assuming this returns true when retracted
            setLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LOW_BASKET);
        } else {
            System.out.println("Cannot raise lift: Actuator is not fully retracted.");
        }
    }

    public void setLiftToHome() {
        setLiftState(SampleLiftBucketSubsystem.SampleLiftStates.HOME);
    }
}
