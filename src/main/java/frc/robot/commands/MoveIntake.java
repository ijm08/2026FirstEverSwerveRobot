package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class MoveIntake extends Command { 
    Intake intake;
    double speed;
    
    public MoveIntake(Intake subsystem, double desiredSpeed) {
        addRequirements(subsystem);
        speed = desiredSpeed;
        intake = subsystem;
    }

    @Override
    public void initialize() {

    }

    @Override 
    public void execute() {
        intake.moveIntakeAutomatically(speed);
    }

    @Override
    public void end(boolean isInterrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
