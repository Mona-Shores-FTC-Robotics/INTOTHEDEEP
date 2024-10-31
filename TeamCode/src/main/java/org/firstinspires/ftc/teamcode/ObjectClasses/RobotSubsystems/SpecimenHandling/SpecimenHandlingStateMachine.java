package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;

public class SpecimenHandlingStateMachine {

    private final SpecimenIntakeSubsystem intakeSubsystem;
    private final SpecimenArmSubsystem armSubsystem;

    // Constructor
    public SpecimenHandlingStateMachine(SpecimenIntakeSubsystem intakeSubsystem,
                                        SpecimenArmSubsystem armSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
    }

    // Method to handle button press logic (toggle actuator states)
    // This button should let us control the pickup and delivery process
    public void onSpecimenHandleButtonPress() {
        switch (armSubsystem.getCurrentState()) {
            //If the arm is in delivery position and operator pushes button, it should move to pickup and turn on the intake
            case SPECIMEN_DELIVERY:
                setArmTargetState(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_PICKUP);
                setIntakeState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_ON);
                break;
                //If the arm is in pickup position and operator pushes button, it should move to staging and turn off the intake
            case SPECIMEN_PICKUP:
                setArmTargetState(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_STAGING);
                setIntakeState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);
                break;
            case SPECIMEN_STAGING:
                setArmTargetState(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_DELIVERY);
                setIntakeState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);
                break;
            default:
                //Could consider some extra states for MOVING_TO_STAGING, MOVING_TO_STAGING, MOVING_TO_PICKUP that would give this button different functionality
                //for example maybe we want to revert to the last known state if we push the button while its moving toward that state?
                break;
        }
    }

    // If we detect a good specimen, shut off the intake and move the arm to staging position
    public void onGoodSpecimenDetected() {
        ParallelCommandGroup retractAndExpelSequence = new ParallelCommandGroup(
                new InstantCommand(this::setIntakeOff),
                new InstantCommand(this::setArmTargetStateToStaging)
        );
        retractAndExpelSequence.schedule();
    }

    //TODO is this really a good idea for onBadSpecimenDetection?
    // What if somehow we detect a yellow Specimen or non-our color specimen
    public void onBadSpecimenDetected() {
        SequentialCommandGroup expelPieceSequence = new SequentialCommandGroup(
                new InstantCommand(this::setArmTargetStateToStaging),
                new WaitCommand(500),  // Wait for the piece to be expelled
                new InstantCommand(this::setIntakeReverse)  // Expel the specimen
        );
        expelPieceSequence.schedule();
    }

    private boolean isArmAtTarget() {
        return armSubsystem.isArmAtTarget();
    }

    private void setIntakeState(SpecimenIntakeSubsystem.SpecimenIntakeStates newState) {
        intakeSubsystem.setCurrentState(newState);
    }

    // Method to turn the intake on
    public void setIntakeOn() {
        setIntakeState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_ON);
    }

    // Method to turn the intake off
    public void setIntakeOff() {
        setIntakeState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);
    }

    // Method to reverse the intake
    public void setIntakeReverse() {
        setIntakeState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_REVERSE);
    }

    private void setArmTargetState(SpecimenArmSubsystem.SpecimenArmStates newState) {
        armSubsystem.setTargetState(newState);
    }

    public void setArmTargetStateToPickup() {
        setArmTargetState(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_PICKUP);
    }

    public void setArmTargetStateToDelivery() {
        setArmTargetState(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_DELIVERY);
    }

    public void setArmTargetStateToStaging() {
        setArmTargetState(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_STAGING);
    }
}
