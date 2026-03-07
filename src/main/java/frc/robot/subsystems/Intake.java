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
    SparkMax intakeArmMotor = new SparkMax(Constants.subsystemCanIds.intakeArmMotor, MotorType.kBrushless);
    SparkFlex intakeMotor = new SparkFlex(Constants.subsystemCanIds.intakeMotor, MotorType.kBrushless);

    SparkMaxConfig intakeArmMotorConfig = new SparkMaxConfig();
    SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();
    public Intake() {
        intakeMotor.clearFaults();
        intakeArmMotor.clearFaults();

        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeArmMotor.configure(intakeArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);       
    }

    // IM A TERRRRRRIIIIIIBLLLLLLLEEEEEEE PROGRAMMER!!!!!!

    public void moveIntakeAutomatically(double intakeArmSpeed) {
        intakeArmMotor.set(intakeArmSpeed);
        if (intakeArmSpeed > 0.00) {
            intakeMotor.set(Constants.speeds.intakeRollerMotorSpeed);
        } else {
            intakeMotor.set(0.0);
        }
    }

    public void stop() {
        intakeMotor.set(0.0);
        intakeArmMotor.set(0.0);
    }
}
