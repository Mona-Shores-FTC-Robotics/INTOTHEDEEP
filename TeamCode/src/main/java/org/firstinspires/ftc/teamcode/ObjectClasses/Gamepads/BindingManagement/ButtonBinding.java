package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class ButtonBinding implements GamePadBinding {
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

    public ButtonBinding(GamepadType gamepadType, GamepadKeys.Button button, String description) {
        this(gamepadType, button, null, description);
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

    public String getButtonName() {
        return button.toString();
    }

    @Override
    public String getDescription() {
        return description;
    }
}
