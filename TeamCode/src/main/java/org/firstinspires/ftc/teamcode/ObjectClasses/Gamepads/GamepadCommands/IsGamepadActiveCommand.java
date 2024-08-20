package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class IsGamepadActiveCommand extends CommandBase {

    boolean done;
    GamepadEx gamepad;
    public IsGamepadActiveCommand(GamepadEx pad)
    {
        gamepad=pad;
    }

    @Override
    public void initialize() {
        done=false;
    }

    @Override
    public void execute(){
        Robot.getInstance().getDriveSubsystem().periodic();
        done = Robot.getInstance().getDriveSubsystem().driverGamepadIsActive(
                gamepad.getLeftY(),
                gamepad.getLeftX(),
                gamepad.getRightX());
    }

    @Override
    public boolean isFinished(){
       if (done)
       {
           return true;
       } else return false;
    }
}
