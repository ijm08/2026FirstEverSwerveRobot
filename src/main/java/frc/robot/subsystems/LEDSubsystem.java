package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {    
    private DigitalOutput desiredOutput;
    
    public LEDSubsystem(int channel) {
        desiredOutput = new DigitalOutput(channel);
    }

    public void sendSignalToArduino() {
        this.desiredOutput.set(true);
    }

    public void turnOffChannel() {
        this.desiredOutput.set(false);
    }
}
