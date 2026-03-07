package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkRelativeEncoder;

import frc.robot.Constants.subsystemCanIds;
import frc.robot.Constants.speeds;

public class Climber extends SubsystemBase {
    SparkMax rightMotor = new SparkMax(subsystemCanIds.climberMotorRight, MotorType.kBrushless);
    SparkMax leftMotor = new SparkMax(subsystemCanIds.climberMotorLeft, MotorType.kBrushless);
    
    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    
    public Climber() {
        leftMotorConfig.follow(rightMotor, true);
        leftMotorConfig.apply(leftMotorConfig);
        // Apply the configurations to the motor controllers
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void moveClimberAutomatically(double somethingToRepresentDirectionCuzIdk) {
        rightMotor.set(speeds.climberSpeed * somethingToRepresentDirectionCuzIdk);
    }

    public void stop() {
        rightMotor.set(0.0);
    }
}
