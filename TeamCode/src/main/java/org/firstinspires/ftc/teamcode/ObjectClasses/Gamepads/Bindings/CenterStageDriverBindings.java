package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.RoadRunnerActionToCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.SlowModeCommand;


public class CenterStageDriverBindings {
    public Command defaultDriveCommand;
    public Command slowModeCommand;

    public CenterStageDriverBindings(GamepadEx gamepad) {

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
        // RIGHT BUMPER - Slow Mode                             //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(slowModeCommand);

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

        //////////////////////////////////////////////////////////
        //                                                      //
        //  X BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        //  BACK/OPTIONS BUTTON                                 //
        //                                                      //
        //////////////////////////////////////////////////////////

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

        slowModeCommand = new SlowModeCommand(Robot.getInstance().getDriveSubsystem(),
                gamepad::getLeftY,
                gamepad::getLeftX,
                gamepad::getRightX
        );

    }
}

