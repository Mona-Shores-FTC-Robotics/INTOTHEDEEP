package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement;

import com.arcrobotics.ftclib.command.Command;

import java.util.function.DoubleSupplier;

public class AnalogBinding implements GamePadBinding {
    private final GamepadType gamepadType;
    private final String analogInputName;
    private final String description;

    public AnalogBinding(GamepadType gamepadType, String analogInputName, String description) {
        this.gamepadType = gamepadType;
        this.analogInputName = analogInputName;
        this.description = description;
    }

    public String getAnalogInputName() {
        return analogInputName;
    }

    public GamepadType getGamepadType() {
        return gamepadType;
    }

    @Override
    public String getDescription() {
        return description;
    }
}
