package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ButtonBindingManager {
    private static ButtonBindingManager instance;
    private final List<ButtonBinding> bindings;

    private ButtonBindingManager() {
        bindings = new ArrayList<>();
    }

    public static ButtonBindingManager getInstance() {
        if (instance == null) {
            instance = new ButtonBindingManager();
        }
        return instance;
    }

    public void registerBinding(ButtonBinding binding) {
        bindings.add(binding);
    }

    public List<ButtonBinding> getBindings() {
        return Collections.unmodifiableList(bindings);
    }

    // Optionally, add methods to clear bindings, find bindings by button, etc.
}
