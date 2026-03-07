package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LEDSubsystem;

public class Shoot extends Command {
    private final Shooter shooter;
    private final LEDSubsystem shootSignal;

    public Shoot(Shooter subsystem, LEDSubsystem subsystem2) {
        shootSignal = subsystem2;
        shooter = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        shootSignal.sendSignalToArduino();
        shooter.shootFuel();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
    @Override
    public void end(boolean interrupted) {
        shootSignal.turnOffChannel();
    }
}
