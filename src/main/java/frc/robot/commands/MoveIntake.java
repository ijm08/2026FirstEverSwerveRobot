package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants;

public class MoveIntake extends Command { 
    Intake intake;
    double dir;
    private static boolean isIntakeDown = false;
    // private double firstEncoderReading;
    // private double distMoved;
    LEDSubsystem intakeSignal;
    
    public MoveIntake(Intake subsystem, double desiredDirection, LEDSubsystem desiredIntakeSignal) {
        addRequirements(subsystem);
        dir = desiredDirection;
        intake = subsystem;
        // firstEncoderReading = 0;
        // distMoved = 0.0;
        intakeSignal = desiredIntakeSignal;
    }

    @Override
    public void initialize() {
        System.out.println(intake.getArmEncoderValues());
        // firstEncoderReading = intake.getArmEncoderValues();
    }

    @Override 
    public void execute() {
        if (dir > 0.00 && isIntakeDown == false) {
           intake.moveIntakeAutomatically(dir);
        } else if (dir < 0.00 && isIntakeDown == true) {
           intake.moveIntakeAutomatically(dir);
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        System.out.println(intake.getArmEncoderValues());
        intake.stopArmMotor();
        if (dir > 0.00) {
          isIntakeDown = true;
          intakeSignal.sendSignalToArduino("intakeDown");
        } else {
          isIntakeDown = false;
          intakeSignal.sendSignalToArduino("intakeUp");
        }
    }

    @Override
    public boolean isFinished() {
        // Arm must be in the robot for this to work, i think....
        // Commented for now until I can get the encoder counts

        // ENCODER STUFF WITH TOLERANCES
        
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

        // ENCODER STUFF WITHOUT TOLERANCES

     /* if (intake.getArmEncoderValues() <= Constants.encoderPositions.intakeExtended && isIntakeDown == false) {
            // Stop the arm motor once the intake fully extends outnts.encoderPositions.inta
            isIntakeDown = true;
            intake.stopArmMotor();
            return true;
        } else if (intake.getArmEncoderValues() >= Constants.encoderPositions.intakeRetracted && isIntakeDown == true) {
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
