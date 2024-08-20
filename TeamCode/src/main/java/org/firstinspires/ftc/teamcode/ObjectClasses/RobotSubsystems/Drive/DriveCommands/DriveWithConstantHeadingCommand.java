package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.TurnPIDController;

import java.util.function.DoubleSupplier;
@Config
public class DriveWithConstantHeadingCommand extends CommandBase {

    public static double P_TERM = .1;
    public static double I_TERM;
    public static double D_TERM;
    public static double F_TERM;

    private final DriveSubsystem driveSubsystem;

    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final double lockedHeadingDegrees;

    private TurnPIDController pid;
    private double currentAngle;
    private MecanumDriveMona mecanumDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;

    /**
     * Creates a new command to drive with heading locked.
     */
    public DriveWithConstantHeadingCommand(DriveSubsystem subsystem,
                                           DoubleSupplier driveInput, DoubleSupplier strafeInput, double headingDegrees) {
        driveSubsystem = subsystem;
        driveSupplier = driveInput;
        strafeSupplier = strafeInput;
        lockedHeadingDegrees = headingDegrees;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        pid = new TurnPIDController(lockedHeadingDegrees, P_TERM, I_TERM, D_TERM, F_TERM);
    }

    @Override
    public void execute() {
        currentAngle = Robot.getInstance().getGyroSubsystem().currentRelativeYawDegrees;
        //this sets the drive/strafe/turn values based on the values supplied, while also doing automatic apriltag driving to the backdrop
        driveSubsystem.setDriveStrafeTurnValues(driveSupplier.getAsDouble(), strafeSupplier.getAsDouble(), pid.update(currentAngle));
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(driveSubsystem.drive, driveSubsystem.strafe, driveSubsystem.turn);
    }
}