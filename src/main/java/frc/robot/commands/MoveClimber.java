package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveClimber extends Command {
    final Climber climber;
    double speed;

    public MoveClimber(Climber subsystem, double desiredDirectionAndSpeed) {
        speed = desiredDirectionAndSpeed;
        climber = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        climber.moveClimberAutomatically(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
    @Override
    public void end(boolean interrupted) {

    }
}
