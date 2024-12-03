package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import static com.example.sharedconstants.FieldConstants.ANGLE_225_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_45_DEGREES;
import static com.example.sharedconstants.FieldConstants.AllianceColor.BLUE;
import static com.example.sharedconstants.FieldConstants.PoseToVector;
import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.example.sharedconstants.FieldConstants;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingActions.BetterPrepareAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingActions.ScoreSampleAction;

import java.util.Arrays;

public class DriveToNetZone implements Action {
    private final DriveSubsystem driveSubsystem;
    private boolean started;
    private boolean cancelled;
    private Action action;// Flag to indicate if the action has been cancelled

    public DriveToNetZone() {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        this.started = false;
        this.cancelled = false; // Initialize the cancellation flag
    }

    public DriveToNetZone(double inches) {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        this.started = false;
        this.cancelled = false; // Initialize the cancellation flag
    }

    // Velocity and acceleration overrides
    public static final double VELOCITY_OVERRIDE = 45;
    public static final double ACCELERATION_OVERRIDE = 45;
    public static final double ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    // Velocity and acceleration overrides
    public static final double VELOCITY_SLOW_OVERRIDE = 10;
    public static final double ACCELERATION_SLOW_OVERRIDE = 10;

    // Shared constraints for all routes
    public static VelConstraint velConstraint;
    public static AccelConstraint accelConstraint;
    public static VelConstraint velSlowConstraint;
    public static AccelConstraint accelSlowConstraint;


    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (cancelled) {
            // Stop the robot abruptly when cancelled
            driveSubsystem.getMecanumDrive().setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            return false; // Signal that the action is complete
        }

        if (!started) {

            velConstraint = new MinVelConstraint(Arrays.asList(
                    new TranslationalVelConstraint(VELOCITY_OVERRIDE),
                    new AngularVelConstraint(ANGULAR_VELOCITY_OVERRIDE)
            ));
            accelConstraint = new ProfileAccelConstraint(- ACCELERATION_OVERRIDE, ACCELERATION_OVERRIDE);

            velSlowConstraint = new MinVelConstraint(Arrays.asList(
                    new TranslationalVelConstraint(VELOCITY_SLOW_OVERRIDE),
                    new AngularVelConstraint(ANGULAR_VELOCITY_OVERRIDE)
            ));
            accelSlowConstraint = new ProfileAccelConstraint(-ACCELERATION_SLOW_OVERRIDE, ACCELERATION_SLOW_OVERRIDE);
            double maxAngVel = Math.PI*3; // shared with path
            double maxAngAccel = Math.PI*3;
            TurnConstraints turnConstraints = new TurnConstraints(
                    maxAngVel, -maxAngAccel, maxAngAccel);

            RealRobotAdapter robotAdapter = new RealRobotAdapter();
            Pose2d currentPose = driveSubsystem.getMecanumDrive().pose;

            if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
                currentPose = new Pose2d(-currentPose.position.x, -currentPose.position.y, currentPose.heading.log()+PI);
            }

            action = robotAdapter.getActionBuilder(currentPose)
                    .turnTo(Math.toRadians(MatchConfig.finalAllianceColor == BLUE ? 225 : 45)) // Pose map does not effect this...
                    .setTangent(ANGLE_225_DEGREES)
                    .afterDisp(15, new BetterPrepareAction())
                    .strafeToLinearHeading(PoseToVector(FieldConstants.NET_BASKET_DRIVE_TO_NET_APPROACH), ANGLE_45_DEGREES, velConstraint, accelConstraint)
                    .strafeToLinearHeading(PoseToVector(FieldConstants.NET_BASKET_DRIVE_TO_NET_SCORE), ANGLE_45_DEGREES,velSlowConstraint, accelSlowConstraint)
                    .stopAndAdd(new ScoreSampleAction())
                    .build();

            action.preview(MatchConfig.telemetryPacket.fieldOverlay()); // Optional: Preview for telemetry
            started = true; // Ensure the action is only initialized once
        }

        // Run the action and update telemetry
        boolean isRunning = action.run(telemetryPacket);

        if (!isRunning) {
            reset(); // Reset the state when the action completes
        }

        telemetryPacket.put("x", driveSubsystem.getMecanumDrive().pose.position.x);
        telemetryPacket.put("y", driveSubsystem.getMecanumDrive().pose.position.y);
        telemetryPacket.put("heading (deg)", Math.toDegrees(driveSubsystem.getMecanumDrive().pose.heading.log()));
        return isRunning;
    }

    public void cancelAbruptly() {
        cancelled = true; // Set the cancellation flag
    }

    // Method to reset the state of the action
    private void reset() {
        started = false;
        cancelled = false;
        action = null; // Reset the action reference
    }
}
