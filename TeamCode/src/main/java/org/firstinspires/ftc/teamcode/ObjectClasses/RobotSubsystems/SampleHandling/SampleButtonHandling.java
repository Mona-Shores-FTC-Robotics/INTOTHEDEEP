package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;

import java.util.Objects;

@Config
public class SampleButtonHandling {

    private final SampleLinearActuatorSubsystem actuatorSubsystem;
    private final SampleIntakeSubsystem intakeSubsystem;
    private final SampleLiftBucketSubsystem liftSubsystem;

    public enum SampleButtonHandlingStates {
        INTAKE, LOW_BASKET, HIGH_BASKET, SCORE_COMPLETE, HOME
    }
    private SampleButtonHandlingStates currentState= SampleButtonHandlingStates.HOME;

    // Constructor
    public SampleButtonHandling(SampleLinearActuatorSubsystem actuatorSubsystem,
                                SampleIntakeSubsystem intakeSubsystem,
                                SampleLiftBucketSubsystem liftSubsystem) {
        this.actuatorSubsystem = actuatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.liftSubsystem = liftSubsystem;
        currentState= SampleButtonHandlingStates.HOME;
    }

    public SampleButtonHandling(SampleLinearActuatorSubsystem actuatorSubsystem,
                                SampleIntakeSubsystem intakeSubsystem
                                      ) {
        this.actuatorSubsystem = actuatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.liftSubsystem = null;
    }

    // Method to handle button press logic (toggle actuator states)
    public void onIntakeButtonPressCommand() {
            switch (actuatorSubsystem.getCurrentState()) {
                case FULLY_RETRACTED:
                    intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);
                    actuatorSubsystem.deployMid();
                    break;
                case DEPLOYED_MID:
                    intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);
                    actuatorSubsystem.deployFull();
                    break;
                case DEPLOYED_FULLY:
                    intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);
                    actuatorSubsystem.fullyRetract();
                    break;
                case MANUAL:
                case UNKNOWN:
                case RETRACTING:
                case DEPLOYING_TO_MID:
                case DEPLOYING_TO_FULL:
                default:
                    intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
                    actuatorSubsystem.fullyRetract();

            }
    }

    public void onScoreButtonPress() {
        switch (currentState) {
            case INTAKE:
                if (actuatorSubsystem.isFullyRetracted()) {  // Assuming this returns true when retracted
                    liftSubsystem.setCurrentDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_HOME);
                    liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS);
                    liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LOW_BASKET);
                    currentState= SampleButtonHandlingStates.LOW_BASKET;
                }
                break;
            case LOW_BASKET:
                liftSubsystem.setCurrentDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_HOME);
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SCORE_POS);
                liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.HIGH_BASKET);
                currentState= SampleButtonHandlingStates.HIGH_BASKET;
                break;
            case HIGH_BASKET:
                liftSubsystem.setCurrentDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_DUMP);
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SCORE_POS);
                liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.HIGH_BASKET);
                currentState= SampleButtonHandlingStates.SCORE_COMPLETE;
                break;
            case SCORE_COMPLETE:
            case HOME:
                liftSubsystem.setCurrentDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_HOME);
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS);
                liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);
                currentState= SampleButtonHandlingStates.INTAKE;
        }
    }


    public void onMoveSampleBucketButtonPress() {
        switch (Objects.requireNonNull(liftSubsystem).getCurrentBucketState()) {
            case BUCKET_INTAKE_POS:
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SCORE_POS);
                 break;
            case BUCKET_SCORE_POS:
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS);
                break;
        }
    }

    public void onMoveSampleDumperButtonPress() {
        switch (Objects.requireNonNull(liftSubsystem).getCurrentDumperState()) {
            case DUMPER_HOME:
                liftSubsystem.setCurrentDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_DUMP);
                break;
            case DUMPER_DUMP:
                liftSubsystem.setCurrentDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_HOME);
                break;
        }
    }


    // Method to turn the intake off
    public void setIntakeOff() {
        intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
    }

    // Method to reverse the intake
    public void setIntakeReverse() {
        intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_REVERSE);
    }



}
