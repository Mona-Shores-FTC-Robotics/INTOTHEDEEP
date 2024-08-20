package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ChangeWinchPowerCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.DroneSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ReleaseDroneCommand;

public class PitModeDriverBindings {



    ClimberSubsystem climberSubsystem = Robot.getInstance().getClimberSubsystem();

    public PitModeDriverBindings(GamepadEx gamepad) {

        //////////////////////////////////////////////////////////
        //                                                      //
        // START TO FLY DRONE                             //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new ReleaseDroneCommand(Robot.getInstance().getDroneSubsystem(), DroneSubsystem.DroneDeployState.FLY));

        //////////////////////////////////////////////////////////
        //                                                      //
        // RIGHT-BUMPER TO ARM DRONE                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ReleaseDroneCommand(Robot.getInstance().getDroneSubsystem(), DroneSubsystem.DroneDeployState.HOLD));

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-UP - ROBOT UP WITH WINCH                        //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new ChangeWinchPowerCommand(climberSubsystem, ClimberSubsystem.WinchMotorStates.ROBOT_UP))
                .whenReleased(new ChangeWinchPowerCommand(climberSubsystem, ClimberSubsystem.WinchMotorStates.OFF));


        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-DOWN - ROBOT DOWN WITH WINCH                    //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ChangeWinchPowerCommand(climberSubsystem, ClimberSubsystem.WinchMotorStates.ROBOT_DOWN))
                .whenReleased(new ChangeWinchPowerCommand(climberSubsystem, ClimberSubsystem.WinchMotorStates.OFF));


    }
}
