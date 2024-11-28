package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import static com.example.sharedconstants.FieldConstants.ANGLE_315_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_340_DEGREES;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_NET;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_OBSERVATION;
import static com.example.sharedconstants.FieldConstants.ANGLE_TOWARD_RED;
import static com.example.sharedconstants.FieldConstants.OBS_CORNER_APPROACH_AUDIENCE_WALL;
import static com.example.sharedconstants.FieldConstants.OBS_CORNER_PICKUP_AUDIENCE_WALL;
import static java.lang.Math.PI;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MonaShoresMessages.SpecimenArmStateMessage;

import java.util.Arrays;
import java.util.Set;

public class DriveToObservationZone implements Action {
    private final DriveSubsystem driveSubsystem;
    private boolean started;
    private boolean cancelled;
    private Action action;// Flag to indicate if the action has been cancelled

    // Velocity and acceleration overrides
    public static final double VELOCITY_OVERRIDE = 30;
    public static final double ACCELERATION_OVERRIDE = 30;
    public static final double ANGULAR_VELOCITY_OVERRIDE = Math.toRadians(360);

    // Velocity and acceleration overrides
    public static final double VELOCITY_SLOW_OVERRIDE = 10;
    public static final double ACCELERATION_SLOW_OVERRIDE = 10;

    // Shared constraints for all routes
    public static VelConstraint velConstraint;
    public static AccelConstraint accelConstraint;
    public static VelConstraint velSlowConstraint;
    public static AccelConstraint accelSlowConstraint;

    public DriveToObservationZone() {
        this.driveSubsystem = Robot.getInstance().getDriveSubsystem();
        this.started = false;
        this.cancelled = false; // Initialize the cancellation flag
    }

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

            RealRobotAdapter robotAdapter = new RealRobotAdapter();
            Pose2d currentPose = driveSubsystem.getMecanumDrive().pose;

            if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
                currentPose = new Pose2d(-currentPose.position.x, -currentPose.position.y, currentPose.heading.log()+PI);
            }

            action = robotAdapter.getActionBuilder(currentPose)
                    .turnTo(ANGLE_TOWARD_NET)
                    .afterTime(.8, new InstantAction(Robot.getInstance().getSpecimenArmSubsystem()::gotoPickupAngle))
                    .afterTime(.8, new InstantAction(Robot.getInstance().getSpecimenIntakeSubsystem()::turnOnIntake))
                    .setTangent(ANGLE_340_DEGREES)
                    .splineToLinearHeading(FieldConstants.OBS_CORNER_APPROACH_DRIVE_TO_OBS_AUDIENCE_WALL, ANGLE_315_DEGREES, velConstraint, accelConstraint)
                    .setReversed(true)
                    .splineToLinearHeading(FieldConstants.OBS_CORNER_PICKUP_AUDIENCE_WALL, ANGLE_TOWARD_OBSERVATION, velSlowConstraint, accelSlowConstraint)
                    .build();

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
