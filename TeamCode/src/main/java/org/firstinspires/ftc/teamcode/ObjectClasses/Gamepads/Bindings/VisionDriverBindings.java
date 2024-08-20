package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor.AllianceColor.RED;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadCommands.IsGamepadActiveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DriveWithConstantHeadingCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActionToCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

public class VisionDriverBindings {
    public static Command defaultDriveCommand;
    public static Command backupFromBackdropCommand;
    public static Command driveAwayFromBackdropWithConstantHeading;

    public VisionDriverBindings(GamepadEx gamepad) {

        //Make the commands to use for the bindings
        MakeCommands(gamepad);

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT STICK / RIGHT STICK - Default Driving           //
        //                                                      //
        //////////////////////////////////////////////////////////
        CommandScheduler.getInstance().setDefaultCommand(Robot.getInstance().getDriveSubsystem(), defaultDriveCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        // RIGHT BUMPER                                         //
        //                                                      //
        //////////////////////////////////////////////////////////

        //While held down, drive normally but hold camera heading toward backdrop

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(driveAwayFromBackdropWithConstantHeading);


        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT BUMPER                                          //
        //                                                      //
        //////////////////////////////////////////////////////////


        //////////////////////////////////////////////////////////
        //                                                      //
        //  Y BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        // move to just outside the correct color wing?
        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(backupFromBackdropCommand);


        //////////////////////////////////////////////////////////
        //                                                      //
        //  X BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    Robot.getInstance().getVisionSubsystem().setDeliverLocation(VisionSubsystem.DeliverLocation.LEFT);
                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    Robot.getInstance().getVisionSubsystem().setDeliverLocation(VisionSubsystem.DeliverLocation.CENTER);
                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    Robot.getInstance().getVisionSubsystem().setDeliverLocation(VisionSubsystem.DeliverLocation.RIGHT);
                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  BACK/OPTIONS BUTTON                                 //
        //                                                      //
        //////////////////////////////////////////////////////////

        //not sure if this should even be an option
        gamepad.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> {
                                Robot.getInstance().getVisionSubsystem().resetHeading=true;
                            }));


        //////////////////////////////////////////////////////////
        //                                                      //
        //  START BUTTON                                        //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .toggleWhenPressed(new InstantCommand(() -> {
                    Robot.getInstance().getActiveOpMode().telemetry.addLine("Field Centric Driving");
                    Robot.getInstance().getDriveSubsystem().fieldOrientedControl = true;
                }), new InstantCommand(() -> {
                    Robot.getInstance().getActiveOpMode().telemetry.addLine("Robot Centric Driving");
                    Robot.getInstance().getDriveSubsystem().fieldOrientedControl = false;
                }));
    }

    private void MakeCommands(GamepadEx gamepad) {
        defaultDriveCommand = new DefaultDriveCommand(Robot.getInstance().getDriveSubsystem(),
                gamepad::getLeftY,
                gamepad::getLeftX,
                gamepad::getRightX
        );


        Command driveWhileAt90Heading = new DriveWithConstantHeadingCommand(Robot.getInstance().getDriveSubsystem(),
                gamepad::getLeftY,
                gamepad::getLeftX,
                Math.toDegrees(Math.toRadians(0)));

        Command driveWhileAt270Heading = new DriveWithConstantHeadingCommand(Robot.getInstance().getDriveSubsystem(),
                gamepad::getLeftY,
                gamepad::getLeftX,
                Math.toDegrees(Math.toRadians(0)));

        if (MatchConfig.finalAllianceColor==RED){
            driveAwayFromBackdropWithConstantHeading=driveWhileAt270Heading;
        } else driveAwayFromBackdropWithConstantHeading= driveWhileAt90Heading;

    }
}

