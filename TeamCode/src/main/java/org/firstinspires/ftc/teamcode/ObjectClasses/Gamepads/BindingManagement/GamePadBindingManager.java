package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class GamePadBindingManager {
    private static GamePadBindingManager instance;
    private final List<GamePadBinding> bindings;

    private GamePadBindingManager() {
        bindings = new ArrayList<>();
    }

    public static GamePadBindingManager getInstance() {
        if (instance == null) {
            instance = new GamePadBindingManager();
        }
        return instance;
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

}
