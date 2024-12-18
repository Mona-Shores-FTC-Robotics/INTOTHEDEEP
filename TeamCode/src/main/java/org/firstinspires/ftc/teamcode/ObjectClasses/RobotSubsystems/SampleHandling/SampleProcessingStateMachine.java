package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import com.acmerobotics.dashboard.config.Config;
import com.example.sharedconstants.FieldConstants;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Lighting.LightingSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;

@Config
public class SampleProcessingStateMachine {

    private final SampleLinearActuatorSubsystem actuatorSubsystem;
    private final SampleIntakeSubsystem intakeSubsystem;
    private final SampleLiftBucketSubsystem liftSubsystem;
    private final LightingSubsystem lightingSubsystem;

    public enum SampleDetectionStates {
        ON_GOOD_SAMPLE_DETECTION,
        GETTING_READY_FOR_TRANSFER,
        TRANSFERRING,
        ON_BAD_SAMPLE_DETECTED,
        EJECTING_BAD_SAMPLE,
        LOOKING_FOR_SAMPLE,
        WAITING_FOR_SAMPLE_DETECTION,
    }
    private SampleDetectionStates currentSampleDetectionState;

    public SampleProcessingStateMachine(SampleLinearActuatorSubsystem actuatorSubsystem,
                                        SampleIntakeSubsystem intakeSubsystem,
                                        SampleLiftBucketSubsystem liftSubsystem,
                                        LightingSubsystem lightingSubsystem) {
        this.actuatorSubsystem = actuatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.liftSubsystem = liftSubsystem;
        this.lightingSubsystem = lightingSubsystem;
        currentSampleDetectionState= SampleDetectionStates.WAITING_FOR_SAMPLE_DETECTION;
    }

    public void updateSampleProcessingState() {
        switch (currentSampleDetectionState) {
            case WAITING_FOR_SAMPLE_DETECTION:
                if (intakeSubsystem.getCurrentIntakeDetectState() == SampleIntakeSubsystem.IntakeDetectState.DETECTED_GOOD_SAMPLE) {
                        currentSampleDetectionState = SampleDetectionStates.ON_GOOD_SAMPLE_DETECTION;
                }
                else if (intakeSubsystem.getCurrentIntakeDetectState() == SampleIntakeSubsystem.IntakeDetectState.DETECTED_BAD_SAMPLE)  {
                        currentSampleDetectionState = SampleDetectionStates.ON_BAD_SAMPLE_DETECTED;
                }
                break;
            case ON_GOOD_SAMPLE_DETECTION:
                currentSampleDetectionState = SampleDetectionStates.GETTING_READY_FOR_TRANSFER;
                lightingSubsystem.setGoodSampleIndicator();
                intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
                liftSubsystem.setCurrentDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_HOME);
                liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);
                liftSubsystem.setBucketToIntakePosition();
                actuatorSubsystem.flipSampleIntakeUpAndRetract();
                lightingSubsystem.setGoodSampleIndicator();
                break;

            case GETTING_READY_FOR_TRANSFER:
                if (actuatorSubsystem.getCurrentState() == SampleLinearActuatorSubsystem.SampleActuatorStates.FULLY_RETRACTED) {
                    currentSampleDetectionState = SampleDetectionStates.TRANSFERRING;
                    lightingSubsystem.setLightToSampleColor();
                    intakeSubsystem.transferSampleToBucket();
                }
                break;
            case TRANSFERRING:
                if (intakeSubsystem.getCurrentState() == SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF) {
                    currentSampleDetectionState = SampleDetectionStates.WAITING_FOR_SAMPLE_DETECTION;
                }
                break;
            case ON_BAD_SAMPLE_DETECTED:
                lightingSubsystem.setBadSampleWarningColor();
                currentSampleDetectionState = SampleDetectionStates.EJECTING_BAD_SAMPLE;
                intakeSubsystem.ejectBadSample();
                break;
            case EJECTING_BAD_SAMPLE:
                if (intakeSubsystem.getCurrentState() == SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF) {
                    currentSampleDetectionState = SampleDetectionStates.WAITING_FOR_SAMPLE_DETECTION;
                    lightingSubsystem.setLightBlack();
                    //Presumably we want to grab another sample from the submersible...
                    actuatorSubsystem.partiallyRetractAndIntakeOn();
                }
                break;
        }
    }

    public SampleDetectionStates getCurrentSampleDetectionState() {
        return currentSampleDetectionState;
    }

}
