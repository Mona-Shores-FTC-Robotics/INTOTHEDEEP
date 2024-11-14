package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem.SAMPLE_LIFT_PARAMS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.ActionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingActions.DriveForwardFromBasket;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;

import java.util.HashSet;
import java.util.Set;

@Config
public class SampleButtonHandling {

    private final SampleLinearActuatorSubsystem actuatorSubsystem;
    private final SampleIntakeSubsystem intakeSubsystem;
    private final SampleLiftBucketSubsystem liftSubsystem;

    private boolean driveAwayTriggered = false; // Flag to track if drive-away action should be triggered
    private ElapsedTime driveAwayTimer = new ElapsedTime(); // Timer for drive-away delay
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
                    actuatorSubsystem.partiallyDeploy();
                    break;
                case PARTIALLY_DEPLOYED:
                    intakeSubsystem.setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);
                    actuatorSubsystem.fullyDeploy();
                    break;
                case MANUAL:
                case UNKNOWN:
                case FULLY_RETRACTING:
                case PARTIALLY_DEPLOYING:
                case FULLY_DEPLOYING:
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
                    liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.HIGH_BASKET);
                    liftSubsystem.setCurrentBucketState(SampleLiftBucketSubsystem.BucketStates.BUCKET_SCORE_POS);
                }
                break;
            case HIGH_BASKET:
                if (!liftSubsystem.hasDumped) {
                    liftSubsystem.dumpSampleInBucket();
                    driveAwayTriggered = false;
                }else if (!driveAwayTriggered)
                {
                    executeDriveAway();
                    driveAwayTimer.reset();
                } else if (driveAwayTimer.seconds() >= 3) {
                    // Start timer for drive-away delay
                    onMoveSampleBucketButtonPress();
                    liftSubsystem.setTargetLiftState(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);
                }
                break;
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
                    SampleLiftBucketSubsystem.BucketStates.BUCKET_INTAKE_POS.position, SAMPLE_LIFT_PARAMS.NUM_STEPS);  // Smooth move to intake position
        }
    }

    public void executeDriveAway() {
        // Implement the logic to drive the robot backward a small distance.
        // Example:
        Set<Subsystem> requirements = new HashSet<>();
        requirements.add(Robot.getInstance().getDriveSubsystem());
        Command driveForwardCommand = new ActionCommand(new DriveForwardFromBasket(10),requirements);
        driveForwardCommand.schedule();
        driveAwayTriggered=true;
    }

}
