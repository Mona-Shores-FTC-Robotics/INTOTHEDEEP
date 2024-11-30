package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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
        SampleLinearActuatorSubsystem.SampleActuatorStates currentState = actuatorSubsystem.getCurrentState();
        switch (currentState) {
                case FULLY_RETRACTED:
                    Robot.getInstance().getSampleLinearActuatorSubsystem().setFlipperDown();
                    Robot.getInstance().getSampleLinearActuatorSubsystem().partiallyDeploy();
                    Robot.getInstance().getSampleIntakeSubsystem().turnOnIntake();
                    Robot.getInstance().getSampleTiwsterSubsystem().setTwisterServoFaceOutwards();
                    break;
                case PARTIALLY_DEPLOYED:
                    case PARTIALLY_DEPLOYING:
                        case MANUAL:
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new InstantCommand(Robot.getInstance().getSampleIntakeSubsystem()::turnOffIntake),
                                    new InstantCommand(Robot.getInstance().getSampleTiwsterSubsystem()::setTwisterServoFaceInward),
                                    new InstantCommand(Robot.getInstance().getSampleLinearActuatorSubsystem()::setFlipperUp),
                                    new InstantCommand(Robot.getInstance().getSampleLinearActuatorSubsystem()::fullyRetract)),
                            new WaitCommand(800),
                            new InstantCommand(Robot.getInstance().getSampleIntakeSubsystem()::reverseIntake),
                            new WaitCommand(800),
                            new InstantCommand(Robot.getInstance().getSampleIntakeSubsystem()::turnOffIntake)
                    ).schedule();
                    break;
            case UNKNOWN:
            case FULLY_RETRACTING:
            default:
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
