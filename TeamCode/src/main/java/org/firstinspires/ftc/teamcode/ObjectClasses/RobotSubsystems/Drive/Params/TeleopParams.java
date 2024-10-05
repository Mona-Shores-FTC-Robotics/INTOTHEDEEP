package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Params;

public class TeleopParams {

    public enum TeleopMode {
        DEFAULT,
        POWER_TEST,
        HIGH_SPEED
    }

    // Default configuration
    public double DRIVE_SPEED_FACTOR = 0.82;
    public double STRAFE_SPEED_FACTOR = 1.0;
    public double TURN_SPEED_FACTOR = 1.0;
    public double DEAD_ZONE = 0.2;
    public double DRIVE_RAMP = 0.2;
    public double STRAFE_RAMP = 0.22;
    public double TURN_RAMP = 0.4;
    public double RAMP_THRESHOLD = 0.04;
    public double P = 0;
    public double D = 0;
    public double I = 0;
    public double F = 13;

    // Inner class for Power Test configuration
    public static class TeleopPowerTestParams extends TeleopParams {
        public TeleopPowerTestParams() {
            this.DRIVE_SPEED_FACTOR = 1.0;
            this.STRAFE_SPEED_FACTOR = 1.0;
            this.TURN_SPEED_FACTOR = 1.0;
            this.DEAD_ZONE = 0.2;
            this.DRIVE_RAMP = 0;
            this.STRAFE_RAMP = 0;
            this.TURN_RAMP = 0;
            this.RAMP_THRESHOLD = 0;
            this.P = 0;
            this.D = 0;
            this.I = 0;
            this.F = 0;
        }
    }

    // Inner class for High Speed Teleop
    public static class HighSpeedTeleopParams extends TeleopParams {
        public HighSpeedTeleopParams() {
            this.DRIVE_SPEED_FACTOR = 1.2;
            this.STRAFE_SPEED_FACTOR = 1.1;
            this.TURN_SPEED_FACTOR = 1.1;
            this.DEAD_ZONE = 0.15;
            this.DRIVE_RAMP = 0.25;
            this.STRAFE_RAMP = 0.25;
            this.TURN_RAMP = 0.45;
            this.RAMP_THRESHOLD = 0.05;
            this.P = 0;
            this.D = 0;
            this.I = 0;
            this.F = 10;
        }
    }

    // Method to choose which params to use based on the enum
    public static TeleopParams getParamsFor(TeleopMode mode) {
        switch (mode) {
            case POWER_TEST:
                return new TeleopPowerTestParams();
            case HIGH_SPEED:
                return new HighSpeedTeleopParams();
            default:
                return new TeleopParams();  // Return default if no match
        }
    }
}
