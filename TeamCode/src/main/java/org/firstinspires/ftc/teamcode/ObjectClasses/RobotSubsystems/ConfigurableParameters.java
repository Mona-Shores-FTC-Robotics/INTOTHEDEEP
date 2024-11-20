package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public abstract class ConfigurableParameters {
    private boolean parametersCustomized = false;

    // Abstract method for loading defaults (must be implemented by subclasses)
    public abstract void loadDefaultsForRobotType(Robot.RobotType robotType);

    // Shared implementation for tracking customization
    public boolean haveRobotSpecificParametersBeenLoaded() {
        return parametersCustomized;
    }

    public void markRobotSpecificParametersLoaded() {
        parametersCustomized = true;
    }
}
