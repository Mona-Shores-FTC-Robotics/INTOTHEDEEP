package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleTwister.SampleTwisterSubsystem;

@Config
public class SampleButtonHandling {

    private final SampleLinearActuatorSubsystem actuatorSubsystem;
    private final SampleIntakeSubsystem intakeSubsystem;
    private final SampleLiftBucketSubsystem liftSubsystem;
    private final SampleTwisterSubsystem twisterSubsystem;

    // Constructor
    public SampleButtonHandling(SampleLinearActuatorSubsystem actuatorSubsystem,
                                SampleIntakeSubsystem intakeSubsystem,
                                SampleLiftBucketSubsystem liftSubsystem,
                                SampleTwisterSubsystem twisterSubsystem) {
        this.actuatorSubsystem = actuatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.liftSubsystem = liftSubsystem;
        this.twisterSubsystem = twisterSubsystem;
    }

    public void onIntakeButtonPress() {
            switch (actuatorSubsystem.getCurrentState()) {
                case FULLY_RETRACTED:
                    intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);
                    twisterSubsystem.setTwisterServoFaceOutwards();
                    actuatorSubsystem.setFlipperTargetPositionWithSteps(SampleLinearActuatorSubsystem.ACTUATOR_PARAMS.FLIP_DOWN_POSITION, 20);
                    actuatorSubsystem.partiallyDeploy();
                    break;

                case PARTIALLY_DEPLOYED:
                case MANUAL:
                case UNKNOWN:
                case FULLY_RETRACTING:
                case PARTIALLY_DEPLOYING:
                default:
                    intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
                    twisterSubsystem.setTwisterServoFaceInward();
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
