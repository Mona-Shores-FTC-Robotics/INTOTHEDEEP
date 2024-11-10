package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;

@Config
public class SampleDetectionStateMachine {

    private final SampleLinearActuatorSubsystem actuatorSubsystem;
    private final SampleIntakeSubsystem intakeSubsystem;
    private final SampleLiftBucketSubsystem liftSubsystem;

    public void setGoodSampleDetectedState() {
        currentSampleDetectionState = SampleDetectionStates.ON_GOOD_SAMPLE_DETECTION;
    }

    public enum SampleDetectionStates {
        ON_GOOD_SAMPLE_DETECTION,
        GETTING_READY_FOR_TRANSFER,
        READY_FOR_TRANSFER,
        TRANSFERRING,
        TRANSFERRED
    }
    private SampleDetectionStates currentSampleDetectionState = SampleDetectionStates.ON_GOOD_SAMPLE_DETECTION;

    public SampleDetectionStateMachine(SampleLinearActuatorSubsystem actuatorSubsystem,
                                       SampleIntakeSubsystem intakeSubsystem,
                                       SampleLiftBucketSubsystem liftSubsystem) {
        this.actuatorSubsystem = actuatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.liftSubsystem = liftSubsystem;
        currentSampleDetectionState= SampleDetectionStates.ON_GOOD_SAMPLE_DETECTION;
    }

    public void  updateSameDetectionState() {
        switch (currentSampleDetectionState) {
            case ON_GOOD_SAMPLE_DETECTION:
                currentSampleDetectionState = SampleDetectionStates.GETTING_READY_FOR_TRANSFER;
                intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS);
                liftSubsystem.setCurrentDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_HOME);
                liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);
                actuatorSubsystem.fullyRetract();
                break;
            case GETTING_READY_FOR_TRANSFER:
                if (actuatorSubsystem.getCurrentState() == SampleLinearActuatorSubsystem.SampleActuatorStates.FULLY_RETRACTED) {
                    currentSampleDetectionState = SampleDetectionStates.READY_FOR_TRANSFER;
                }
                break;
            case READY_FOR_TRANSFER:
                currentSampleDetectionState = SampleDetectionStates.TRANSFERRING;
                intakeSubsystem.transferSampleToBucket();
                break;
            case TRANSFERRING:
                if (intakeSubsystem.getCurrentState()== SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF) {
                    currentSampleDetectionState = SampleDetectionStates.TRANSFERRED;;
                }
        }
    }

    public SampleDetectionStates getCurrentSampleDetectionState() {
        return currentSampleDetectionState;
    }

}
