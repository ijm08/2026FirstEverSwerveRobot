package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveClimber extends Command {
    private Climber climber;
    private double dir;

    public MoveClimber(Climber subsystem, double desiredDirection) {
        dir = desiredDirection;
        climber = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        climber.moveClimberAutomatically(dir);
    }

    @Override
    public boolean isFinished() {
        // Encoder code for the climber (only works if the robot is started with it down all the way):
        // I will keep this code commented for now until I can 
        // obtain accurate encoder counts so that we do not damage parts

        /* if (climber.getEncoderValues() <= Constants.encoderPositions.climberExtended + Constants.encoderPositions.climberEncoderTolerance) {
            climber.stop();
            return true;
        } else if (climber.getEncoderValues() >= Constants.encoderPositions.climberRetracted - Constants.encoderPositions.climberEncoderTolerance) {
            climber.stop();
            return true;
        } */
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {

    }
}
