package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm.SpecimenArmWithMotionProfileSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;

//Right now these are just for Teleop... should they be?
public class SpecimenHandlingStateMachine {

    private final SpecimenIntakeSubsystem intakeSubsystem;
    private final SpecimenArmWithMotionProfileSubsystem armSubsystem;

    // Constructor
    public SpecimenHandlingStateMachine(SpecimenIntakeSubsystem intakeSubsystem,
                                        SpecimenArmWithMotionProfileSubsystem armSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
    }

    // Method to handle button press logic (toggle actuator states)
    // This button should let us control the pickup and delivery process
    public void onSpecimenHandleButtonPress() {
        switch (armSubsystem.getCurrentState()) {

                //If the arm is in pickup position and operator pushes button, it should move to staging and turn off the intake
            case SPECIMEN_PICKUP:
                setArmTargetState(SpecimenArmWithMotionProfileSubsystem.SpecimenArmStates.CW_ARM_HOME);
                setIntakeState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);
                break;
            case CW_ARM_HOME:
                setArmTargetState(SpecimenArmWithMotionProfileSubsystem.SpecimenArmStates.SPECIMEN_DELIVERY);
                setIntakeState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);
                break;
            case CCW_ARM_HOME:
                setArmTargetState(SpecimenArmWithMotionProfileSubsystem.SpecimenArmStates.SPECIMEN_PICKUP);
                setIntakeState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_ON);
                break;
            default:
            case SPECIMEN_DELIVERY:
                setArmTargetState(SpecimenArmWithMotionProfileSubsystem.SpecimenArmStates.CCW_ARM_HOME);
                setIntakeState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);
                break;
        }
    }

    // If we detect a good specimen, shut off the intake and move the arm to staging position
    public void onGoodSpecimenDetectedCommand() {
        ParallelCommandGroup retractAndExpelSequence = new ParallelCommandGroup(
                new InstantCommand(this::setIntakeOff),
                new InstantCommand(this::setArmTargetStateToStaging)
        );
        retractAndExpelSequence.schedule();
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

    private void setArmTargetState(SpecimenArmWithMotionProfileSubsystem.SpecimenArmStates newState) {
        armSubsystem.setTargetStateWithMotionProfile(newState);
    }

    public void setArmTargetStateToPickup() {
        setArmTargetState(SpecimenArmWithMotionProfileSubsystem.SpecimenArmStates.SPECIMEN_PICKUP);
    }

    public void setArmTargetStateToDelivery() {
        setArmTargetState(SpecimenArmWithMotionProfileSubsystem.SpecimenArmStates.SPECIMEN_DELIVERY);
    }

    public void setArmTargetStateToStaging() {
        setArmTargetState(SpecimenArmWithMotionProfileSubsystem.SpecimenArmStates.CW_ARM_HOME);
    }
}
