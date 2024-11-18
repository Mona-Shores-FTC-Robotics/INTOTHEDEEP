package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.AnglePIDController;
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
    private final AnglePIDController headingPID;

    // Store the previously used PID and tolerance values
    private double previousKP = PARAMS_FIXED_ORIENTATION.KP;
    private double previousKI = PARAMS_FIXED_ORIENTATION.KI;
    private double previousKD = PARAMS_FIXED_ORIENTATION.KD;
    private double previousKF = PARAMS_FIXED_ORIENTATION.KF;
    private double previousToleranceDegrees = PARAMS_FIXED_ORIENTATION.TOLERANCE_DEGREES;

    private final double targetHeadingRadians;

    double maxRawCorrection = PARAMS_FIXED_ORIENTATION.KP * Math.PI; // Assuming error ranges from -π to π


    public DriveAtFixedDegreeHeadingCommand(DriveSubsystem driveSubsystem ,
                                            DoubleSupplier driveSupplier ,
                                            DoubleSupplier strafeSupplier ,
                                            double fixedHeadingDegrees) {
        this.driveSubsystem = driveSubsystem;
        this.driveSupplier = driveSupplier;
        this.strafeSupplier = strafeSupplier;
        targetHeadingRadians = Math.toRadians(fixedHeadingDegrees);

        // Initialize the PIDF Controller using PARAMS_FIXED_ORIENTATION
        headingPID = new AnglePIDController(
                DriveAtFixedDegreeHeadingCommand.PARAMS_FIXED_ORIENTATION.KP ,
                DriveAtFixedDegreeHeadingCommand.PARAMS_FIXED_ORIENTATION.KI ,
                DriveAtFixedDegreeHeadingCommand.PARAMS_FIXED_ORIENTATION.KD ,
                DriveAtFixedDegreeHeadingCommand.PARAMS_FIXED_ORIENTATION.KF
        );

        // Use setTolerance to configure error tolerance
        headingPID.setTolerance(Math.toRadians(DriveAtFixedDegreeHeadingCommand.PARAMS_FIXED_ORIENTATION.TOLERANCE_DEGREES));

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // Update PID coefficients if they have changed
        if (previousKP != PARAMS_FIXED_ORIENTATION.KP ||
                previousKI != PARAMS_FIXED_ORIENTATION.KI ||
                previousKD != PARAMS_FIXED_ORIENTATION.KD ||
                previousKF != PARAMS_FIXED_ORIENTATION.KF) {
            headingPID.setPIDF(
                    PARAMS_FIXED_ORIENTATION.KP ,
                    PARAMS_FIXED_ORIENTATION.KI ,
                    PARAMS_FIXED_ORIENTATION.KD ,
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
        double currentHeading = driveSubsystem.getMecanumDrive().pose.heading.log();

        // Calculate the correction using the AnglePIDController
        double rawCorrection = headingPID.calculate(currentHeading , targetHeadingRadians);
        double normalizedCorrection = rawCorrection / maxRawCorrection;

        // Clamp the correction to the range [-1.0, 1.0]
        double clampedCorrection = Math.max(- 1.0 , Math.min(1.0 , normalizedCorrection));

        // Log telemetry for debugging
        MatchConfig.telemetryPacket.put("FixedAngle/SetPoint" , Math.toDegrees(targetHeadingRadians));
        MatchConfig.telemetryPacket.put("FixedAngle/Current Heading" , Math.toDegrees(currentHeading));
        MatchConfig.telemetryPacket.put("FixedAngle/Error" , Math.toDegrees(headingPID.getPositionError()));
        MatchConfig.telemetryPacket.put("FixedAngle/Raw Correction" , rawCorrection);
        MatchConfig.telemetryPacket.put("FixedAngle/Scaled Correction" , clampedCorrection);

        // Apply corrections to the drive system
        driveSubsystem.setDriveStrafeValues(drive , strafe);
        driveSubsystem.drive(driveSubsystem.drive , driveSubsystem.strafe , clampedCorrection);
    }
}