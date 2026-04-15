package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;

public class LEDSubsystem extends SubsystemBase {    
    // private SerialPort arduino;
    
    public LEDSubsystem() {
        // this.arduino = new SerialPort(9600, SerialPort.Port.kOnboard);
    }

    public void sendSignalToArduino(String data) {
        // this.arduino.writeString(data);
    }
}
