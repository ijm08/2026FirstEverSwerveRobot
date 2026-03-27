package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;

public class MoveIntake extends Command { 
    Intake intake;
    double dir;
    LEDSubsystem intakeSignal;
    
    public MoveIntake(Intake subsystem, double desiredDirection, LEDSubsystem desiredIntakeSignal) {
        addRequirements(subsystem);
        dir = desiredDirection;
        intake = subsystem;
        intakeSignal = desiredIntakeSignal;
    }

    @Override
    public void initialize() {

    }

    @Override 
    public void execute() {

    }

    @Override
    public void end(boolean isInterrupted) {
        //if (dir < 0.00) {
        //    intakeSignal.sendSignalToArduino();
        //} else {
        //    intakeSignal.turnOffChannel();
        //}
    }

    @Override
    public boolean isFinished() {
        // Arm must be in the robot for this to work, i think....
        // Commented for now until I can get the encoder counts
        
        /* if (intake.getArmEncoderValues() <= Constants.encoderPositions.intakeExtended + Constants.encoderPositions.intakeTolerance) {
            // Stop the arm motor once the intake fully extends out
            intake.stopArmMotor();
            return true;
        } else if (intakeArmMotor.getEncoder().getPosition() >= Constants.encoderPositions.intakeRetracted - Constants.encoderPositions.intakeTolerance) {
            // Stop the arm motor once it fully retracts back into the robot
            intake.stopArmMotor();
            return true;
            
        } else {
            return false;
        } */
        return false;
    }
}
