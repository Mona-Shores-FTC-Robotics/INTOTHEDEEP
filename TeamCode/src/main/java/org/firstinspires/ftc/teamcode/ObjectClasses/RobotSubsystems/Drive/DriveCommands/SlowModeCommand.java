package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input *
 */
@Config
public class SlowModeCommand extends CommandBase {

    public static double SLOW_DRIVE_FACTOR = .35;
    public static double SLOW_TURN_FACTOR = .5;
    public static double SLOW_STRAFE_FACTOR = .5;

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier turnSupplier;

    private double previousDriveFactor;
    private double previousStrafeFactor;
    private double previousTurnFactor;

    /**
     * Creates a new SlowMode Command.
     */
    public SlowModeCommand(DriveSubsystem subsystem,
                           DoubleSupplier driveInput, DoubleSupplier strafeInput, DoubleSupplier turnInput) {
        driveSubsystem = subsystem;
        driveSupplier = driveInput;
        strafeSupplier = strafeInput;
        turnSupplier = turnInput;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        DriveSubsystem.TeleopParams teleopParams = Robot.getInstance().getDriveSubsystem().TELEOP_PARAMS;
        //save the normal speed factors to reset them after the command is finished
        previousDriveFactor = teleopParams.DRIVE_SPEED_FACTOR;
        previousStrafeFactor =  teleopParams.STRAFE_SPEED_FACTOR;
        previousTurnFactor =  teleopParams.TURN_SPEED_FACTOR;

        //set the slow mode speed factors
        teleopParams.DRIVE_SPEED_FACTOR = SLOW_DRIVE_FACTOR;
        teleopParams.STRAFE_SPEED_FACTOR = SLOW_STRAFE_FACTOR;
        teleopParams.TURN_SPEED_FACTOR = SLOW_TURN_FACTOR;
    }

    @Override
    public void execute() {
        //this sets the drive/strafe/turn values based on the values supplied and the FACTORS set above
        driveSubsystem.setDriveStrafeTurnValues(
                driveSupplier.getAsDouble(),
                strafeSupplier.getAsDouble(),
                turnSupplier.getAsDouble()
        );
        driveSubsystem.mecanumDriveSpeedControl(driveSubsystem.drive, driveSubsystem.strafe, driveSubsystem.turn);
    }

    @Override
    public void end(boolean interrupted) {
        DriveSubsystem.TeleopParams teleopParams = Robot.getInstance().getDriveSubsystem().TELEOP_PARAMS;

        //set the speed factors back to normal
        teleopParams.DRIVE_SPEED_FACTOR = previousDriveFactor;
        teleopParams.STRAFE_SPEED_FACTOR = previousStrafeFactor;
        teleopParams.TURN_SPEED_FACTOR = previousTurnFactor;
    }
}