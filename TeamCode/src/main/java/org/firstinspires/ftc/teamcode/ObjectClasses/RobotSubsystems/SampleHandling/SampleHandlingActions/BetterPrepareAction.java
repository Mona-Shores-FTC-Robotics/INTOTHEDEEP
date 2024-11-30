package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingActions;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.FieldConstants;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
@Config
public class BetterPrepareAction implements Action {
    public static double DELAY_TO_HIGH_HOLD = .75;
    private boolean started;
    private Action action;// Flag to indicate if the action has been cancelled

    public BetterPrepareAction() {
        this.started = false;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!started) {
            action = new  SequentialAction(
                            new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::moveLiftToHighBasket),
                            new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::moveDumperToPreScore),
                            new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::setBucketToScorePosition),
                            new SleepAction(DELAY_TO_HIGH_HOLD),
                            new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::moveDumperToHighHold));

            action.preview(MatchConfig.telemetryPacket.fieldOverlay()); // Optional: Preview for telemetry
            started = true; // Ensure the action is only initialized once
        }

        // Run the action and update telemetry
        boolean isRunning = action.run(telemetryPacket);

        if (!isRunning) {
            reset(); // Reset the state when the action completes
        }
        return isRunning;
    }

    // Method to reset the state of the action
    private void reset() {
        started = false;
        action = null; // Reset the action reference
    }
}
