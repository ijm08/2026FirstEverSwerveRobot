package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.SerialPort;

public class LEDSubsystem extends SubsystemBase {    
    private DigitalOutput arduino;
    
    public LEDSubsystem(int channel) {
        arduino = new DigitalOutput(channel);
    }

    public void sendSignalToArduino() {
        this.arduino.set(false);
    }

    public void turnOffChannel() {
        this.arduino.set(true);
    }
}
