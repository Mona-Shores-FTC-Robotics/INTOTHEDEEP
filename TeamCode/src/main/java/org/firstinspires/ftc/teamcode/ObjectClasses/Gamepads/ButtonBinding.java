package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class ButtonBinding {
    private final GamepadType gamepadType;
    private final GamepadKeys.Button button;
    private final Command command;
    private final String description;

    public ButtonBinding(GamepadType gamepadType, GamepadKeys.Button button, Command command, String description) {
        this.gamepadType = gamepadType;
        this.button = button;
        this.command = command;
        this.description = description;
    }

    public GamepadType getGamepadType() {
        return gamepadType;
    }

    public GamepadKeys.Button getButton() {
        return button;
    }

    public Command getCommand() {
        return command;
    }

    public String getDescription() {
        return description;
    }
}
