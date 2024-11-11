package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;

@Config
public class SampleButtonHandling {

    private final SampleLinearActuatorSubsystem actuatorSubsystem;
    private final SampleIntakeSubsystem intakeSubsystem;
    private final SampleLiftBucketSubsystem liftSubsystem;

    // Constructor
    public SampleButtonHandling(SampleLinearActuatorSubsystem actuatorSubsystem,
                                SampleIntakeSubsystem intakeSubsystem,
                                SampleLiftBucketSubsystem liftSubsystem) {
        this.actuatorSubsystem = actuatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.liftSubsystem = liftSubsystem;
    }

    public void onIntakeButtonPress() {
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
        switch (liftSubsystem.getCurrentLiftState()) {
            case LIFT_HOME:
                if (actuatorSubsystem.isFullyRetracted()) {  // Assuming this returns true when retracted
                    liftSubsystem.hasDumped = false;
                    liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LOW_BASKET);
                }
                break;
            case LOW_BASKET:
                if (!liftSubsystem.hasDumped){
                    liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SCORE_POS);
                    liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.HIGH_BASKET);
                }else {
                    liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);
                    liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS);
                }
                break;
            case HIGH_BASKET:
                if (!liftSubsystem.hasDumped) {
                    liftSubsystem.dumpSampleInBucket();
                }else {
                    liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LOW_BASKET_DOWN);
                    liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS);
                }
                break;
            case LOW_BASKET_DOWN:
                liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);
            case MANUAL:
            default:
                liftSubsystem.setCurrentDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_HOME);
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS);
                liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);
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

    public void onMoveSampleBucketButtonPress() {
        SampleLiftBucketSubsystem.BucketStates currentState = liftSubsystem.getCurrentBucketState();
        if (currentState == SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS) {
            liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SCORE_POS);
        } else {
            liftSubsystem.setBucketTargetPosition(
                    SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS.position, 50);  // Smooth move to intake position
        }
    }
}
