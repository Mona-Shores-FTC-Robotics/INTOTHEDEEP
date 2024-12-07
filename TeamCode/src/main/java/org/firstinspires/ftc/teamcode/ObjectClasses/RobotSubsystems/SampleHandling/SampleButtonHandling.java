package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
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

    public boolean mightHaveUndetectedSampleFlag;

    // Constructor
    public SampleButtonHandling(SampleLinearActuatorSubsystem actuatorSubsystem,
                                SampleIntakeSubsystem intakeSubsystem,
                                SampleLiftBucketSubsystem liftSubsystem,
                                SampleTwisterSubsystem twisterSubsystem) {
        this.actuatorSubsystem = actuatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.liftSubsystem = liftSubsystem;
        this.twisterSubsystem = twisterSubsystem;
        mightHaveUndetectedSampleFlag =false;
    }

    public void onIntakeButtonPress() {
        Command retract = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(Robot.getInstance().getSampleIntakeSubsystem()::turnOffIntake),
                        new InstantCommand(Robot.getInstance().getSampleTiwsterSubsystem()::setTwisterServoFaceInward),
                        new InstantCommand(Robot.getInstance().getSampleLinearActuatorSubsystem()::fullyRetract),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new  InstantCommand(Robot.getInstance().getSampleLinearActuatorSubsystem()::setFlipperUp)
                        )
                ),
                new WaitCommand(800),
                new InstantCommand(Robot.getInstance().getSampleIntakeSubsystem()::reverseIntake),
                new WaitCommand(800),
                new InstantCommand(Robot.getInstance().getSampleIntakeSubsystem()::turnOffIntake));

        Command deploy = new ParallelCommandGroup(
                new InstantCommand(Robot.getInstance().getSampleLinearActuatorSubsystem()::setFlipperDown),
                new InstantCommand(Robot.getInstance().getSampleLinearActuatorSubsystem()::partiallyDeploy),
                new InstantCommand(Robot.getInstance().getSampleIntakeSubsystem()::turnOnIntake),
                new InstantCommand(Robot.getInstance().getSampleTiwsterSubsystem()::setTwisterServoFaceOutwards)
        );

        Command hover = new ParallelCommandGroup(
                new InstantCommand(Robot.getInstance().getSampleLinearActuatorSubsystem()::setFlipperHover),
                new InstantCommand(Robot.getInstance().getSampleLinearActuatorSubsystem()::partiallyDeploy),
                new InstantCommand(Robot.getInstance().getSampleIntakeSubsystem()::turnOnIntake),
                new InstantCommand(Robot.getInstance().getSampleTiwsterSubsystem()::setTwisterServoFaceOutwards)
        );

        Command hoverWithoutActuator = new ParallelCommandGroup(
                new InstantCommand(Robot.getInstance().getSampleLinearActuatorSubsystem()::setFlipperHover),
                new InstantCommand(Robot.getInstance().getSampleIntakeSubsystem()::turnOnIntake),
                new InstantCommand(Robot.getInstance().getSampleTiwsterSubsystem()::setTwisterServoFaceInward)
        );

        Command deployFromHover = new ParallelCommandGroup(
                new InstantCommand(Robot.getInstance().getSampleLinearActuatorSubsystem()::setFlipperDown),
                new InstantCommand(Robot.getInstance().getSampleIntakeSubsystem()::turnOnIntake)
        );

        SampleLinearActuatorSubsystem.SampleActuatorStates currentState = actuatorSubsystem.getCurrentState();
        switch (currentState) {
            case FULLY_RETRACTED:
                if (Robot.getInstance().getSampleLinearActuatorSubsystem().currentFlipperState== SampleLinearActuatorSubsystem.SampleFlipperStates.FLIPPER_HOVERING) {
                    deploy.schedule();
                } else if (Robot.getInstance().getSampleLinearActuatorSubsystem().currentFlipperState== SampleLinearActuatorSubsystem.SampleFlipperStates.FLIPPER_UP) {
                    hover.schedule();
                }
                break;

            case PARTIALLY_DEPLOYED:
            case PARTIALLY_DEPLOYING:
            case MANUAL:
                if (Robot.getInstance().getSampleLinearActuatorSubsystem().currentFlipperState== SampleLinearActuatorSubsystem.SampleFlipperStates.FLIPPER_HOVERING)
                {
                    if (mightHaveUndetectedSampleFlag)
                    {
                        retract.schedule();
                        mightHaveUndetectedSampleFlag=false;
                    } else deployFromHover.schedule();
                } else if (Robot.getInstance().getSampleLinearActuatorSubsystem().currentFlipperState== SampleLinearActuatorSubsystem.SampleFlipperStates.FLIPPER_DOWN)
                {
                    mightHaveUndetectedSampleFlag =true;
                    //todo this hover should probably not be the same height, should probably be higher so that we dont hit a piece retracting on the submersible.
                    hoverWithoutActuator.schedule();
                }
                break;
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
