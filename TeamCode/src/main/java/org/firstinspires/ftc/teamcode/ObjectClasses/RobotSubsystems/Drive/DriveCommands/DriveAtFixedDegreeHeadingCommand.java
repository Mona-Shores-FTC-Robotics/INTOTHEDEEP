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

    // Store the previously used PID and tolerance values
    private double previousKP = PARAMS_FIXED_ORIENTATION.KP;
    private double previousKI = PARAMS_FIXED_ORIENTATION.KI;
    private double previousKD = PARAMS_FIXED_ORIENTATION.KD;
    private double previousKF = PARAMS_FIXED_ORIENTATION.KF;
    private double previousToleranceDegrees = PARAMS_FIXED_ORIENTATION.TOLERANCE_DEGREES;

    private final double fixedHeadingRadians;

    public DriveAtFixedDegreeHeadingCommand(DriveSubsystem driveSubsystem ,
                                            DoubleSupplier driveSupplier ,
                                            DoubleSupplier strafeSupplier ,
                                            double fixedHeadingDegrees) {
        this.driveSubsystem = driveSubsystem;
        this.driveSupplier = driveSupplier;
        this.strafeSupplier = strafeSupplier;
        fixedHeadingRadians = Math.toRadians(fixedHeadingDegrees);

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
        // Check if PID parameters or tolerance have changed, and update the controller if necessary
        if (previousKP != PARAMS_FIXED_ORIENTATION.KP ||
                previousKI != PARAMS_FIXED_ORIENTATION.KI ||
                previousKD != PARAMS_FIXED_ORIENTATION.KD ||
                previousKF != PARAMS_FIXED_ORIENTATION.KF) {
            headingPID.setPIDF(
                    PARAMS_FIXED_ORIENTATION.KP,
                    PARAMS_FIXED_ORIENTATION.KI,
                    PARAMS_FIXED_ORIENTATION.KD,
                    PARAMS_FIXED_ORIENTATION.KF
            );
            previousKP = PARAMS_FIXED_ORIENTATION.KP;
            previousKI = PARAMS_FIXED_ORIENTATION.KI;
            previousKD = PARAMS_FIXED_ORIENTATION.KD;
            previousKF = PARAMS_FIXED_ORIENTATION.KF;
        }

        if (previousToleranceDegrees != PARAMS_FIXED_ORIENTATION.TOLERANCE_DEGREES) {
            headingPID.setTolerance(Math.toRadians(PARAMS_FIXED_ORIENTATION.TOLERANCE_DEGREES));
            previousToleranceDegrees = PARAMS_FIXED_ORIENTATION.TOLERANCE_DEGREES;
        }

        // Get drive and strafe inputs
        double drive = driveSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();

        // Get the current heading in radians
        double currentHeading = Math.toRadians(driveSubsystem.getMecanumDrive().pose.heading.log());

        // Normalize the current heading and the setpoint to handle wraparound
        double normalizedCurrentHeading = Math.atan2(Math.sin(currentHeading), Math.cos(currentHeading));
        double normalizedSetpoint = Math.atan2(Math.sin(fixedHeadingRadians), Math.cos(fixedHeadingRadians));

        // Update the PID controller's setpoint (if dynamic updates are required)
        headingPID.setSetPoint(normalizedSetpoint);

        // Compute the correction using the PID controller
        double rawCorrection = headingPID.calculate(normalizedCurrentHeading);

        // Clamp the correction to [-1.0, 1.0]
        double maxOutput = 1.0;
        double scaledCorrection = Math.max(-maxOutput, Math.min(maxOutput, rawCorrection));

        // Log telemetry for debugging
        MatchConfig.telemetryPacket.put("FixedAngle/SetPoint", Math.toDegrees(fixedHeadingRadians));
        MatchConfig.telemetryPacket.put("FixedAngle/Heading", Math.toDegrees(currentHeading));
        MatchConfig.telemetryPacket.put("FixedAngle/NormalizedHeading", Math.toDegrees(normalizedCurrentHeading));
        MatchConfig.telemetryPacket.put("FixedAngle/Error", Math.toDegrees(headingPID.getPositionError()));
        MatchConfig.telemetryPacket.put("FixedAngle/Raw Correction", rawCorrection);
        MatchConfig.telemetryPacket.put("FixedAngle/Scaled Correction", scaledCorrection);

        // Apply corrections to the drive system
        driveSubsystem.setDriveStrafeTurnValues(drive, strafe, scaledCorrection);
        driveSubsystem.drive(driveSubsystem.drive, driveSubsystem.strafe, driveSubsystem.turn);
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