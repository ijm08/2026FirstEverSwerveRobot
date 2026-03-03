package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    public Shoot(Shooter subsystem, double speed) {
        addRequirements(subsystem);
    }
}
