package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import static com.example.sharedconstants.FieldConstants.END_GAME_TIME;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActionToCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

//TODO can we describe what each button should do? Maybe we should download a PS5 controller map and fill it out to start thinking about how the driver and operator will control the robot?
// -maybe functionally describe what each button should do in this file to get us started?

public class IntoTheDeepDriverBindings {
    public Command defaultDriveCommand;
    public Command slowModeCommand;
    public Command backupFromBackdropCommand;
    public Command slowModeZeroHeadingCommand;

    public IntoTheDeepDriverBindings(GamepadEx gamepad) {
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
        // RIGHT BUMPER - Slow Mode Zero Heading                //
        //                                                      //
        //////////////////////////////////////////////////////////


        //TODO can we get slowmode working?
//        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenHeld(slowModeZeroHeadingCommand);

//        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenHeld(slowModeCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT BUMPER     - fly drone                          //
        //                                                      //
        //////////////////////////////////////////////////////////
//
//        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(new InstantCommand(() -> {
//                    if (MatchConfig.teleOpTimer.seconds() > END_GAME_TIME) {
//                        //new ReleaseDroneCommand(Robot.getInstance().getDroneSubsystem(), DroneSubsystem.DroneDeployState.FLY).schedule();
//                    }
//                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  Y BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        // moves straight back and rotates us toward the wing - can be cancelled to easily grab from the neutral stacks instead
//        gamepad.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(backupFromBackdropCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        //  X BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
//        gamepad.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(new InstantCommand(() -> {
//                    Robot.getInstance().getVisionSubsystem().setDeliverLocation(VisionSubsystem.DeliverLocation.LEFT);
//                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
//        gamepad.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new InstantCommand(() -> {
//                    Robot.getInstance().getVisionSubsystem().setDeliverLocation(VisionSubsystem.DeliverLocation.CENTER);
//                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
//        gamepad.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(new InstantCommand(() -> {
//                    Robot.getInstance().getVisionSubsystem().setDeliverLocation(VisionSubsystem.DeliverLocation.RIGHT);
//                }));

//        gamepad.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(new RoadRunnerActionToCommand.ActionAsCommand(Robot.getInstance().getDriveSubsystem(), new TurnToAction(0)));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  BACK/OPTIONS BUTTON                                 //
        //                                                      //
        //////////////////////////////////////////////////////////

        //This button sets a flag so the next time we are at the backdrop it does a full gyro reset
        //The concept here is if we disconnect, this gives us a way to reset our gyro to a known field position
        //Otherwise, we normally only reset our x/y pose based on the apriltag reading and the heading based on the gyro
        // because the gyro is much more accurate than the april tag based heading
//        gamepad.getGamepadButton(GamepadKeys.Button.BACK)
//                .whenPressed(new InstantCommand(() -> {
//                    Robot.getInstance().getVisionSubsystem().resetHeading=true;
//                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  START BUTTON  - FIELD ORIENTED CONTROL              //
        //                                                      //
        //////////////////////////////////////////////////////////
//        gamepad.getGamepadButton(GamepadKeys.Button.START)
//                .toggleWhenPressed(new InstantCommand(() -> {
//                    Robot.getInstance().getActiveOpMode().telemetry.addLine("Field Centric Driving");
//                    Robot.getInstance().getDriveSubsystem().fieldOrientedControl = true;
//                }), new InstantCommand(() -> {
//                    Robot.getInstance().getActiveOpMode().telemetry.addLine("Robot Centric Driving");
//                    Robot.getInstance().getDriveSubsystem().fieldOrientedControl = false;
//                }));
    }

    private void MakeCommands(GamepadEx gamepad) {
        defaultDriveCommand = new DefaultDriveCommand(Robot.getInstance().getDriveSubsystem(),
                gamepad::getLeftY,
                gamepad::getLeftX,
                gamepad::getRightX
        );

//        slowModeCommand = new SlowModeCommand(Robot.getInstance().getDriveSubsystem(),
//                gamepad::getLeftY,
//                gamepad::getLeftX,
//                gamepad::getRightX
//        );
//
//        slowModeZeroHeadingCommand = new SlowModeZeroHeadingCommand(Robot.getInstance().getDriveSubsystem(),
//                gamepad::getLeftY,
//                gamepad::getLeftX,
//                0
//        );

    }
}

