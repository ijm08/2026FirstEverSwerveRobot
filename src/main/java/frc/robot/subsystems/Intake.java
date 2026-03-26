package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private SparkMax intakeArmMotor = new SparkMax(Constants.subsystemCanIds.intakeArmMotor, MotorType.kBrushless);
    private SparkFlex intakeMotor = new SparkFlex(Constants.subsystemCanIds.intakeMotor, MotorType.kBrushless);

    SparkMaxConfig intakeArmMotorConfig = new SparkMaxConfig();
    SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();
    public Intake() {
        intakeMotor.clearFaults();
        intakeArmMotor.clearFaults();
        intakeArmMotorConfig.smartCurrentLimit(40);

        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeArmMotor.configure(intakeArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);       
    }

    // IM A TERRRRRRIIIIIIBLLLLLLLEEEEEEE PROGRAMMER!!!!!!

    public void moveIntakeAutomatically(double desiredIntakeDirection) {
        //intakeArmMotor.set(Constants.speeds.intakeArmMotorSpeed * desiredIntakeDirection);
        //if (desiredIntakeDirection > 0.00) {
            // Up I think, make rollers stop
            //intakeMotor.set(Constants.speeds.intakeRollerMotorSpeed);
        //} else {
            // When the intake is down, keep the rollers spinning constantly
            // Until it is retracted, at which point stop them
            //intakeMotor.set(0.0);
        //}
    }

    public void moveIntakeMan() {
        intakeMotor.set(-0.3);
    }

    public void stop() {
        intakeMotor.set(0.0);
    }

    // im just including the following method because whatever, even though everything
    // SHOULD, yes, SHOULD (meaning it probably wont), handle everything automatically
    public void stopArmMotor() {
        // intakeArmMotor.set(0.0);

        intakeMotor.set(0.0);
    }

    public double getArmEncoderValues() {
        return intakeArmMotor.getEncoder().getPosition();
    }
}
