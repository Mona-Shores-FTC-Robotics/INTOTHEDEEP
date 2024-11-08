package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class GamePadBindingManager {
    private final List<GamePadBinding> bindings;

    public GamePadBindingManager() {
        bindings = new ArrayList<>();
    }

    // Method to register button bindings
    public void registerBinding(ButtonBinding binding) {
        bindings.add(binding);
    }

    // Overloaded method to register analog stick bindings
    public void registerBinding(AnalogBinding binding) {
        bindings.add(binding);
    }

    public List<GamePadBinding> getBindings() {
        return Collections.unmodifiableList(bindings);
    }

    // Method to clear bindings if needed
    public void clearBindings() {
        bindings.clear();
    }

    // Method to check if bindings exist
    public boolean hasBindings() {
        return !bindings.isEmpty();
    }
}
