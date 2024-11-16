package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveAtFixedDegreeHeadingCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final double fixedHeadingRadians; // Desired fixed heading in radians

    /**
     * Creates a new DriveAtFixedHeadingCommand.
     *
     * @param driveSubsystem  The DriveSubsystem instance
     * @param driveSupplier   Supplier for forward/backward input
     * @param strafeSupplier  Supplier for lateral input
     * @param fixedHeadingDegrees Desired heading in degrees
     */
    public DriveAtFixedDegreeHeadingCommand(DriveSubsystem driveSubsystem,
                                            DoubleSupplier driveSupplier,
                                            DoubleSupplier strafeSupplier,
                                            double fixedHeadingDegrees) {
        this.driveSubsystem = driveSubsystem;
        this.driveSupplier = driveSupplier;
        this.strafeSupplier = strafeSupplier;
        this.fixedHeadingRadians = Math.toRadians(fixedHeadingDegrees); // Convert degrees to radians

        addRequirements(driveSubsystem); // Ensure this command has exclusive access to the drive system
    }

    @Override
    public void execute() {
        // Get inputs for drive and strafe
        double drive = driveSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();

        // Get the current heading from the drive subsystem (assume radians)
        double currentHeading = driveSubsystem.getMecanumDrive().pose.heading.log(); // Replace with actual method for heading

        // Calculate the heading error
        double headingError = fixedHeadingRadians - currentHeading;

        // Normalize the heading error to the range [-PI, PI]
        headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

        // Apply proportional control for heading correction
        double headingCorrection = driveSubsystem.getMecanumDrive().PARAMS.headingGain * headingError;

        // Log data for debugging
        Robot.getInstance().getActiveOpMode().telemetry.addData( "Drive: %.2f", "Strafe: %.2f", "HeadingError: %.2f", "Correction: %.2f",
                drive, strafe, headingError, headingCorrection);

        // Send the drive commands with the heading correction
        driveSubsystem.setDriveStrafeTurnValues(drive, strafe, headingCorrection);
        driveSubsystem.drive(driveSubsystem.drive, driveSubsystem.strafe, driveSubsystem.turn);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        driveSubsystem.getMecanumDrive().setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
        Robot.getInstance().getActiveOpMode().telemetry.addData("FixedHeadingCommand", "Command Ended: Interrupted=%s", interrupted);
    }

    @Override
    public boolean isFinished() {
        // This command runs continuously until explicitly canceled
        return false;
    }
}
