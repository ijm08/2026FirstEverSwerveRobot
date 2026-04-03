package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants;

public class MoveIntake extends Command { 
    Intake intake;
    double dir;
    private boolean isIntakeDown;
    private double firstEncoderReading;
    private double distMoved;
    LEDSubsystem intakeSignal;
    
    public MoveIntake(Intake subsystem, double desiredDirection, LEDSubsystem desiredIntakeSignal) {
        addRequirements(subsystem);
        dir = desiredDirection;
        isIntakeDown = false;
        intake = subsystem;
        firstEncoderReading = 0;
        distMoved = 0.0;
        intakeSignal = desiredIntakeSignal;
    }

    @Override
    public void initialize() {
        firstEncoderReading = intake.getArmEncoderValues();
    }

    @Override 
    public void execute() {
        intake.moveIntakeAutomatically(dir);
    }

    @Override
    public void end(boolean isInterrupted) {
        if (dir < 0.00) {
          intakeSignal.sendSignalToArduino();
        } else {
          intakeSignal.turnOffChannel();
        }
    }

    @Override
    public boolean isFinished() {
        // Arm must be in the robot for this to work, i think....
        // Commented for now until I can get the encoder counts
        
        /* if (intake.getArmEncoderValues() <= Constants.encoderPositions.intakeExtended + Constants.encoderPositions.intakeTolerance && isIntakeDown == false) {
            // Stop the arm motor once the intake fully extends outnts.encoderPositions.inta
            isIntakeDown = true;
            intake.stopArmMotor();
            return true;
        } else if (intake.getArmEncoderValues() >= Constants.encoderPositions.intakeRetracted - Constants.encoderPositions.intakeTolerance && isIntakeDown == true) {
            // Stop the arm motor once it fully retracts back into the robot
            isIntakeDown = false;
            intake.stopArmMotor();
            return true;
            
        } else {
            return false;
        }   */
        return false;
    }
}
