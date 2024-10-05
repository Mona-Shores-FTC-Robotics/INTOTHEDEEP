package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Params;

import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

public class LocalizerParams {

    // Localizer Params for Chassis Two Dead Wheel Internal IMU
    public static class ChassisTwoDeadWheelInternalIMULocalizerParams extends TwoDeadWheelLocalizer.Params {
        public ChassisTwoDeadWheelInternalIMULocalizerParams() {
            this.parYTicks = -1450.0;
            this.perpXTicks = 800.0;
        }
    }

    // Localizer Params for CenterStage Two Dead Wheel Internal IMU
    public static class CenterStageTwoDeadWheelInternalIMULocalizerParams extends TwoDeadWheelLocalizer.Params {
        public CenterStageTwoDeadWheelInternalIMULocalizerParams() {
            this.parYTicks = -1440.0;
            this.perpXTicks = 820.0;
        }
    }

    // Add more localizer parameter classes as needed
}
