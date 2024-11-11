package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;

import java.util.Objects;

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


    // Method to handle button press logic (toggle actuator states)
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
                    liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);
                    liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS);
                }
                break;
            case MANUAL:
            default:
                liftSubsystem.setCurrentDumperState(SampleLiftBucketSubsystem.DumperStates.DUMPER_HOME);
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS);
                liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);
        }
    }


    public void onMoveSampleBucketButtonPress() {
        switch (liftSubsystem.getCurrentBucketState()) {

            case BUCKET_INTAKE_POS:
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SCORE_POS);
                 break;
            case BUCKET_SCORE_POS:
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SAFE_DESCENT1);
                break;
            case BUCKET_SAFE_DESCENT1:
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SAFE_DESCENT2);
                break;
            case BUCKET_SAFE_DESCENT2:
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SAFE_DESCENT3);
                break;
            case BUCKET_SAFE_DESCENT3:
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SAFE_DESCENT4);
                break;
            case BUCKET_SAFE_DESCENT4:
                liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS);
                break;



        }
    }

    public void onMoveSampleDumperButtonPress() {
        switch (liftSubsystem.getCurrentDumperState()) {
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


//    public void onMoveSampleBucketButtonPress() {
//        SampleLiftBucketSubsystem.BucketStates currentState = liftSubsystem.getCurrentBucketState();
//        // Toggle between intake and score positions
//        double startPosition, endPosition;
//        if (currentState == SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS) {
//            liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SCORE_POS);
//        } else {
//            startPosition = SampleLiftBucketSubsystem.BucketStates.BUCKET_SCORE_POS.position;
//            endPosition = SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS.position;
//            smoothServoTransition(startPosition, endPosition, 50); // 50 incremental steps
//        }
//    }
//
//    private void smoothServoTransition(double startPosition, double endPosition, int numSteps) {
//        double stepSize = (endPosition - startPosition) / numSteps;
//
//        for (int i = 1; i <= numSteps; i++) {
//            double currentPosition = startPosition + (stepSize * i);
//            liftSubsystem.bucket.setPosition(currentPosition); // Set each incremental position
//
//            // Wait for the servo to reach the set position (tune delay as needed)
//            try { Thread.sleep(10); } catch (InterruptedException e) { e.printStackTrace(); }
//        }
//
//        // Final set to end position to ensure it reaches exactly
//        liftSubsystem.setBucketPosition(endPosition);
//    }




}
