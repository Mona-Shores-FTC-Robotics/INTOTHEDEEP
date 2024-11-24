package org.firstinspires.ftc.teamcode.messages.MonaShoresMessages;

public final class SpecimenArmPowerMessage {
    public long timestamp;
    public double pidPower;
    public double feedforwardPower;
    public double clippedPower;

    public SpecimenArmPowerMessage(double pidPower, double feedforwardPower, double clippedPower) {
        this.timestamp = System.nanoTime();
        this.pidPower = pidPower;
        this.feedforwardPower = feedforwardPower;
        this.clippedPower = clippedPower;
    }
}
