package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {    
    private DigitalOutput ledChOutput;
    
    public LEDSubsystem(int channel) {
        ledChOutput = new DigitalOutput(channel);
    }

    public void sendSignalToArduino() {
        this.ledChOutput.set(true);
    }

    public void turnOffChannel() {
        this.ledChOutput.set(false);
    }
}
