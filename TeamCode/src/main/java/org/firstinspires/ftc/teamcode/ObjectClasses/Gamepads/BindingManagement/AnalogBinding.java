package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement;

import com.arcrobotics.ftclib.command.Command;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;

public class AnalogBinding implements GamePadBinding {
    private final GamepadType gamepadType;
    private final List<String> analogInputNames;
    private final String description;

    // Constructor for multiple analog inputs
    public AnalogBinding(GamepadType gamepadType, List<String> analogInputNames, String description) {
        this.gamepadType = gamepadType;
        this.analogInputNames = analogInputNames;
        this.description = description;
    }

    // Overloaded constructor for a single analog input
    public AnalogBinding(GamepadType gamepadType, String analogInputName, String description) {
        this.gamepadType = gamepadType;
        this.analogInputNames = Collections.singletonList(analogInputName);
        this.description = description;
    }

    public List<String> getAnalogInputNames() {
        return analogInputNames;
    }

    public GamepadType getGamepadType() {
        return gamepadType;
    }

    @Override
    public String getDescription() {
        return description;
    }
}
