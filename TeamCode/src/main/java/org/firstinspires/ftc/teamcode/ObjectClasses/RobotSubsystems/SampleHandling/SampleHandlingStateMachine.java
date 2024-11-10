package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;

import java.util.Objects;

@Config
public class SampleHandlingStateMachine {
    public static class SampleHandlingParams {
        public long DELAY_RETRACT_TO_MID_POSITION_IN_MS = 250;
        public long DELAY_MID_TO_FULL_POSITION_IN_MS = 300;
        public long DELAY_FULL_TO_RETRACT_POSITION_IN_MS = 700;
    }

    public static SampleHandlingParams SAMPLE_HANDLING_PARAMS = new SampleHandlingParams();
    private final SampleLinearActuatorSubsystem actuatorSubsystem;
    private final SampleIntakeSubsystem intakeSubsystem;
    private final SampleLiftBucketSubsystem liftSubsystem;


    public enum SampleHandlingStates {
        INTAKE, LOW_BASKET, HIGH_BASKET, SCORE_COMPLETE, HOME
    }
    private SampleHandlingStates currentState=SampleHandlingStates.HOME;
    private SampleHandlingStates targetState=SampleHandlingStates.HOME;

    // Constructor
    public SampleHandlingStateMachine(SampleLinearActuatorSubsystem actuatorSubsystem,
                                      SampleIntakeSubsystem intakeSubsystem,
                                      SampleLiftBucketSubsystem liftSubsystem) {
        this.actuatorSubsystem = actuatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.liftSubsystem = liftSubsystem;
        currentState=SampleHandlingStates.HOME;
        targetState=SampleHandlingStates.HOME;
    }

    public SampleHandlingStateMachine(SampleLinearActuatorSubsystem actuatorSubsystem,
                                      SampleIntakeSubsystem intakeSubsystem
                                      ) {
        this.actuatorSubsystem = actuatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.liftSubsystem = null;
    }

    // Method to handle button press logic (toggle actuator states)
    public void onIntakeButtonPressCommand() {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR_WITH_ENCODER)) {
            switch (actuatorSubsystem.getCurrentState()) {
                case RETRACT:
                    Command sampleIntakeMidDeployCommand = new SequentialCommandGroup(
                            new InstantCommand(this::setIntakeOn),
                            new InstantCommand(this::deployActuatorMid)
                    );
                    sampleIntakeMidDeployCommand.schedule();
                    break;
                case DEPLOY_MID:
                    Command sampleIntakeFullDeployCommand = new SequentialCommandGroup(
                            new InstantCommand(this::setIntakeOn),
                            new InstantCommand(this::deployActuatorFull)
                    );
                    sampleIntakeFullDeployCommand.schedule();
                    break;
                case DEPLOY_FULL:
                case MANUAL:
                default:
                    Command sampleIntakeRetractCommand = new SequentialCommandGroup(
                            new InstantCommand(this::setIntakeOff),
                            new InstantCommand(this::retractActuator)
                    );
                    sampleIntakeRetractCommand.schedule();
                    break;
            }
        } else {
            switch (actuatorSubsystem.getCurrentState()) {
                case RETRACT:
                    Command sampleIntakeMidDeployCommand = new SequentialCommandGroup(
                            new InstantCommand(this::setIntakeOn),
                            new InstantCommand(actuatorSubsystem::runWithoutEncodersForward),
                            new WaitCommand(SAMPLE_HANDLING_PARAMS.DELAY_RETRACT_TO_MID_POSITION_IN_MS),
                            new InstantCommand(actuatorSubsystem::stopActuator),
                            new InstantCommand(this::setActuatorStateDeployedMid)
                    );
                    sampleIntakeMidDeployCommand.schedule();
                    break;
                case DEPLOY_MID:
                    Command sampleIntakeFullDeployCommand = new SequentialCommandGroup(
                            new InstantCommand(this::setIntakeOn),
                            new InstantCommand(actuatorSubsystem::runWithoutEncodersForward),
                            new WaitCommand(SAMPLE_HANDLING_PARAMS.DELAY_MID_TO_FULL_POSITION_IN_MS),
                            new InstantCommand(actuatorSubsystem::stopActuator),
                            new InstantCommand(this::setActuatorStateFullyDeployed)

                    );
                    sampleIntakeFullDeployCommand.schedule();
                    break;
                case DEPLOY_FULL:
                case MANUAL:
                default:
                    Command sampleIntakeRetractCommand = new SequentialCommandGroup(
                            new InstantCommand(this::setIntakeOff),
                            new InstantCommand(actuatorSubsystem::runWithoutEncodersReverse),
                            new WaitCommand(SAMPLE_HANDLING_PARAMS.DELAY_FULL_TO_RETRACT_POSITION_IN_MS),
                            new InstantCommand(actuatorSubsystem::stopActuator),
                            new InstantCommand(this::setActuatorStateRetracted)
                    );
                    sampleIntakeRetractCommand.schedule();
                    break;
            }
        }
    }

    private void setActuatorStateDeployedMid() {
        actuatorSubsystem.setCurrentState(SampleLinearActuatorSubsystem.SampleActuatorStates.DEPLOY_MID);
    }

    private void setActuatorStateRetracted() {
        actuatorSubsystem.setCurrentState(SampleLinearActuatorSubsystem.SampleActuatorStates.RETRACT);
    }

    private void setActuatorStateFullyDeployed() {
        actuatorSubsystem.setCurrentState(SampleLinearActuatorSubsystem.SampleActuatorStates.DEPLOY_FULL);
    }

    public void onScoreButtonPress() {
        switch (currentState) {
            case INTAKE:
                if (actuatorSubsystem.isFullyRetracted()) {  // Assuming this returns true when retracted
                    liftSubsystem.setTargetDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_HOME);
                    liftSubsystem.setTargetBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS);
                    liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LOW_BASKET);
                    currentState=SampleHandlingStates.LOW_BASKET;
                }
                break;
            case LOW_BASKET:
                liftSubsystem.setTargetDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_HOME);
                liftSubsystem.setTargetBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SCORE_POS);
                liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.HIGH_BASKET);
                break;
            case HIGH_BASKET:
                liftSubsystem.setTargetDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_DUMP);
                liftSubsystem.setTargetBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SCORE_POS);
                liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.HIGH_BASKET);
                //TODO timer?
                break;
            case SCORE_COMPLETE:
                liftSubsystem.setTargetDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_HOME);
                liftSubsystem.setTargetBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS);
                liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);


        }
    }


    public void onMoveSampleBucketButtonPress() {
        switch (Objects.requireNonNull(liftSubsystem).getCurrentBucketState()) {
            case BUCKET_INTAKE_POS:
                liftSubsystem.setTargetBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SCORE_POS);
                 break;
            case BUCKET_SCORE_POS:
                liftSubsystem.setTargetBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS);
                break;
        }
    }

    public void onMoveSampleDumperButtonPress() {
        switch (Objects.requireNonNull(liftSubsystem).getCurrentDumperState()) {
            case DUMPER_HOME:
                liftSubsystem.setTargetDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_DUMP);
                break;
            case DUMPER_DUMP:
                liftSubsystem.setTargetDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_HOME);
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
                //Todo Once we have a limit switch, maybe we can handle this better
        // Intothedeepera24!
        // ?
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

    private void retractActuator() {
        actuatorSubsystem.setTargetState(SampleLinearActuatorSubsystem.SampleActuatorStates.RETRACT);
    }
    private void deployActuatorMid() {
        actuatorSubsystem.setTargetState(SampleLinearActuatorSubsystem.SampleActuatorStates.DEPLOY_MID);
    }
    private void deployActuatorFull() {
        actuatorSubsystem.setTargetState(SampleLinearActuatorSubsystem.SampleActuatorStates.DEPLOY_FULL);
    }

    // Method to turn the intake on
    public void setIntakeOn() {
        intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);
    }

    // Method to turn the intake off
    public void setIntakeOff() {
        intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
    }

    // Method to reverse the intake
    public void setIntakeReverse() {
        intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_REVERSE);
    }

    public void setLiftToHighBasket() {
        // Check if the actuator is fully retracted before moving the lift
        if (actuatorSubsystem.isFullyRetracted()) {  // Assuming this returns true when retracted
            liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.HIGH_BASKET);
        } else {
            System.out.println("Cannot raise lift: Actuator is not fully retracted.");
        }
    }

    public void setLiftToLowBasket() {
        // Check if the actuator is fully retracted before moving the lift
        if (actuatorSubsystem.isFullyRetracted()) {  // Assuming this returns true when retracted
            liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LOW_BASKET);
        } else {
            System.out.println("Cannot raise lift: Actuator is not fully retracted.");
        }
    }

    public void setLiftToHome() {
         liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);
    }



    public SampleHandlingStates getCurrentSampleHandlingState() { return currentState;}
    public SampleHandlingStates setCurrentSampleHandlingState() { return targetState;}

}
