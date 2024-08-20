package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input *
 */
public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier turnSupplier;

    private MecanumDriveMona mecanumDrive;

    /**
     * Creates a new DefaultDrive.
     */
    public DefaultDriveCommand(DriveSubsystem subsystem,
                                DoubleSupplier driveInput, DoubleSupplier strafeInput, DoubleSupplier turnInput) {
        driveSubsystem = subsystem;
        driveSupplier = driveInput;
        strafeSupplier = strafeInput;
        turnSupplier = turnInput;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //this sets the drive/strafe/turn values based on the values supplied, while also doing automatic apriltag driving to the backdrop
        driveSubsystem.setDriveStrafeTurnValues(driveSupplier.getAsDouble(), strafeSupplier.getAsDouble(), turnSupplier.getAsDouble());
        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(driveSubsystem.drive, driveSubsystem.strafe, driveSubsystem.turn);
    }
}