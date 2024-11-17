package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

@Config
public class DriveAtFixedDegreeHeadingCommand extends CommandBase {

    public static class Params_For_Fixed_Orientation {
        public double KP = 0.01; // Proportional gain
        public double KI = 0.0;  // Integral gain
        public double KD = 0.0;  // Derivative gain
        public double KF = 0.0;  // Feedforward (optional)
        public double TOLERANCE_DEGREES = 1.0; // Tolerance in degrees
    }

    public static Params_For_Fixed_Orientation PARAMS_FIXED_ORIENTATION = new Params_For_Fixed_Orientation();

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final PIDFController headingPID;

    public DriveAtFixedDegreeHeadingCommand(DriveSubsystem driveSubsystem ,
                                            DoubleSupplier driveSupplier ,
                                            DoubleSupplier strafeSupplier ,
                                            double fixedHeadingDegrees) {
        this.driveSubsystem = driveSubsystem;
        this.driveSupplier = driveSupplier;
        this.strafeSupplier = strafeSupplier;
        double fixedHeadingRadians = Math.toRadians(fixedHeadingDegrees);

        // Initialize the PIDF Controller using PARAMS_FIXED_ORIENTATION
        headingPID = new PIDFController(
                DriveAtFixedDegreeHeadingCommand.PARAMS_FIXED_ORIENTATION.KP ,
                DriveAtFixedDegreeHeadingCommand.PARAMS_FIXED_ORIENTATION.KI ,
                DriveAtFixedDegreeHeadingCommand.PARAMS_FIXED_ORIENTATION.KD ,
                DriveAtFixedDegreeHeadingCommand.PARAMS_FIXED_ORIENTATION.KF
        );

        // Set the desired heading as the PID controller's setpoint
        headingPID.setSetPoint(fixedHeadingRadians);

        // Use setTolerance to configure error tolerance
        headingPID.setTolerance(Math.toRadians(DriveAtFixedDegreeHeadingCommand.PARAMS_FIXED_ORIENTATION.TOLERANCE_DEGREES));

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // Get drive and strafe inputs from the suppliers
        double drive = driveSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();

        // Get the current heading from the DriveSubsystem
        double currentHeading = driveSubsystem.getMecanumDrive().pose.heading.log();

        // Calculate heading correction using the PID controller
        double headingCorrection = headingPID.calculate(currentHeading);

        // Log telemetry for debugging
        MatchConfig.telemetryPacket.put("Heading" , currentHeading);
        MatchConfig.telemetryPacket.put("Correction" , headingCorrection);

        // Apply the calculated values to the drive system
        driveSubsystem.setDriveStrafeTurnValues(drive , strafe , headingCorrection);
        driveSubsystem.drive(driveSubsystem.drive , driveSubsystem.strafe , driveSubsystem.turn);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        driveSubsystem.getMecanumDrive().setDrivePowers(new PoseVelocity2d(new Vector2d(0 , 0) , 0));
    }

    @Override
    public boolean isFinished() {
        // Command does not finish on its own; only stops when canceled
        return false;
    }
}