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
                    actuatorSubsystem.flipSampleIntakeDown();
                    actuatorSubsystem.partiallyDeploy();
                    break;

                case PARTIALLY_DEPLOYED:
                case MANUAL:
                case UNKNOWN:
                case FULLY_RETRACTING:
                case PARTIALLY_DEPLOYING:
                default:
                    intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
                    actuatorSubsystem.flipSampleIntakeUpAndRetract();
                    break;



            }
    }

    public void onMoveSampleBucketButtonPress() {
        SampleLiftBucketSubsystem.BucketStates currentState = liftSubsystem.getCurrentBucketState();
        if (currentState == SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS) {
            liftSubsystem.setBucketToScorePosition();
        } else {
            liftSubsystem.setBucketToIntakePosition();
        }
    }

}
