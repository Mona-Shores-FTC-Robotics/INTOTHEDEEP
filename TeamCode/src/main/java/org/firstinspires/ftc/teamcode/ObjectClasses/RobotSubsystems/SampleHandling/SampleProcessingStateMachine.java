package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Lighting.LightingSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleTwister.SampleTwisterSubsystem;
import org.firstinspires.ftc.teamcode.messages.MonaShoresMessages.SampleProcessingStateMachineMessage;

@Config
public class SampleProcessingStateMachine {

    public static double TWISTER_DELAY_TIME_FOR_AUTO_MS = 100;
    public static double FLIP_UP_DELAY_TIME_FOR_AUTO_MS = 200;

    public static double FLIP_UP_DELAY_TIME_MS = 840;

    private final SampleLinearActuatorSubsystem actuatorSubsystem;
    private final SampleIntakeSubsystem intakeSubsystem;
    private final SampleLiftBucketSubsystem liftSubsystem;
    private final LightingSubsystem lightingSubsystem;
    private final SampleTwisterSubsystem sampleTwisterSubsystem;
    private ElapsedTime flipAllTheWayUpTimer;

    private boolean haveTwisted=false;

    public enum SampleDetectionStates {
        ON_GOOD_SAMPLE_DETECTION,
        GETTING_READY_FOR_TRANSFER,
        TRANSFERRING,
        ON_BAD_SAMPLE_DETECTED,
        EJECTING_BAD_SAMPLE,
        LOOKING_FOR_SAMPLE,
        WAITING_FOR_SAMPLE_DETECTION, FLIPPING_ALL_THE_WAY_UP,
    }
    private SampleDetectionStates currentSampleDetectionState;

    public SampleProcessingStateMachine(SampleLinearActuatorSubsystem actuatorSubsystem,
                                        SampleIntakeSubsystem intakeSubsystem,
                                        SampleLiftBucketSubsystem liftSubsystem,
                                        SampleTwisterSubsystem sampleTwisterSubsystem,
                                        LightingSubsystem lightingSubsystem) {
        this.actuatorSubsystem = actuatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.liftSubsystem = liftSubsystem;
        this.lightingSubsystem = lightingSubsystem;
        this.sampleTwisterSubsystem = sampleTwisterSubsystem;
        currentSampleDetectionState= SampleDetectionStates.WAITING_FOR_SAMPLE_DETECTION;
        flipAllTheWayUpTimer = new ElapsedTime();
    }

    public void updateSampleProcessingState() {
        FlightRecorder.write("SAMPLE_PROCESSING_STATE_MACHINE", new SampleProcessingStateMachineMessage(currentSampleDetectionState));
        if (Robot.getInstance().isAutoMode()) {
            switch (currentSampleDetectionState) {
                case WAITING_FOR_SAMPLE_DETECTION:
                    if (intakeSubsystem.getCurrentIntakeDetectState() == SampleIntakeSubsystem.SampleIntakeDetectState.DETECTED_GOOD_SAMPLE) {
                        currentSampleDetectionState = SampleDetectionStates.ON_GOOD_SAMPLE_DETECTION;
                    } else if (intakeSubsystem.getCurrentIntakeDetectState() == SampleIntakeSubsystem.SampleIntakeDetectState.DETECTED_BAD_SAMPLE) {
                        currentSampleDetectionState = SampleDetectionStates.ON_BAD_SAMPLE_DETECTED;
                    }
                    break;
                case ON_GOOD_SAMPLE_DETECTION:
                    currentSampleDetectionState = SampleDetectionStates.GETTING_READY_FOR_TRANSFER;
                    intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
                    liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);
                    liftSubsystem.setBucketToIntakePosition();
                    actuatorSubsystem.setFlipperHover();
                    actuatorSubsystem.fullyRetract();
//                    sampleTwisterSubsystem.setTwisterServoFaceInward();
                    lightingSubsystem.setLightToSampleColor();
                    haveTwisted=false;
                    break;

                case GETTING_READY_FOR_TRANSFER:
                    if (actuatorSubsystem.isReadyToTransfer()) {
                        currentSampleDetectionState = SampleDetectionStates.FLIPPING_ALL_THE_WAY_UP;
                        actuatorSubsystem.setFlipperUp();
                        flipAllTheWayUpTimer.reset();
                    }
                    break;
                case FLIPPING_ALL_THE_WAY_UP:
                    if (flipAllTheWayUpTimer.milliseconds()>(FLIP_UP_DELAY_TIME_MS - (FLIP_UP_DELAY_TIME_MS/2)) && !haveTwisted)
                    {
                        haveTwisted=true;
                        sampleTwisterSubsystem.setTwisterServoFaceInward();
                    }
                    if (flipAllTheWayUpTimer.milliseconds() > FLIP_UP_DELAY_TIME_MS) {
                        currentSampleDetectionState = SampleDetectionStates.TRANSFERRING;
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
                    sampleTwisterSubsystem.setTwisterServoDumpToSide();
                    actuatorSubsystem.setFlipperHover();
                    intakeSubsystem.ejectBadSample();
                    break;
                case EJECTING_BAD_SAMPLE:
                    if (intakeSubsystem.getCurrentState() == SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF) {
                        currentSampleDetectionState = SampleDetectionStates.WAITING_FOR_SAMPLE_DETECTION;
                        lightingSubsystem.setLightBlack();
                        //Presumably we want to grab another sample from the submersible...
                        sampleTwisterSubsystem.setTwisterServoFaceOutwards();
                        intakeSubsystem.turnOnIntake();
                    }
                    break;
            }
        } else
        {
            switch (currentSampleDetectionState) {
                case WAITING_FOR_SAMPLE_DETECTION:
                    if (intakeSubsystem.getCurrentIntakeDetectState() == SampleIntakeSubsystem.SampleIntakeDetectState.DETECTED_GOOD_SAMPLE) {
                        currentSampleDetectionState = SampleDetectionStates.ON_GOOD_SAMPLE_DETECTION;
                    } else if (intakeSubsystem.getCurrentIntakeDetectState() == SampleIntakeSubsystem.SampleIntakeDetectState.DETECTED_BAD_SAMPLE) {
                        currentSampleDetectionState = SampleDetectionStates.ON_BAD_SAMPLE_DETECTED;
                    }
                    break;
                case ON_GOOD_SAMPLE_DETECTION:
                    currentSampleDetectionState = SampleDetectionStates.GETTING_READY_FOR_TRANSFER;
                    intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
                    liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);
                    liftSubsystem.setBucketToIntakePosition();
                    actuatorSubsystem.setFlipperHover();
                    actuatorSubsystem.fullyRetract();
                    sampleTwisterSubsystem.setTwisterServoFaceInward();
                    lightingSubsystem.setLightToSampleColor();
                    break;

                case GETTING_READY_FOR_TRANSFER:
                    if (actuatorSubsystem.isReadyToTransfer()) {
                        currentSampleDetectionState = SampleDetectionStates.FLIPPING_ALL_THE_WAY_UP;
                        actuatorSubsystem.setFlipperUp();
                        flipAllTheWayUpTimer.reset();
                    }
                    break;
                case FLIPPING_ALL_THE_WAY_UP:
                    if (flipAllTheWayUpTimer.milliseconds() > FLIP_UP_DELAY_TIME_MS) {
                        currentSampleDetectionState = SampleDetectionStates.TRANSFERRING;
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
                    sampleTwisterSubsystem.setTwisterServoDumpToSide();
                    actuatorSubsystem.setFlipperHover();
                    intakeSubsystem.ejectBadSample();
                    break;
                case EJECTING_BAD_SAMPLE:
                    if (intakeSubsystem.getCurrentState() == SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF) {
                        currentSampleDetectionState = SampleDetectionStates.WAITING_FOR_SAMPLE_DETECTION;
                        lightingSubsystem.setLightBlack();
                        //Presumably we want to grab another sample from the submersible...
                        sampleTwisterSubsystem.setTwisterServoFaceOutwards();
                        intakeSubsystem.turnOnIntake();
                    }
                    break;
            }
        }
    }

    public SampleDetectionStates getCurrentSampleDetectionState() {
        return currentSampleDetectionState;
    }



    public void setWaitingForSampleDetectionState()
    {
        currentSampleDetectionState = SampleDetectionStates.WAITING_FOR_SAMPLE_DETECTION;
    }
}
