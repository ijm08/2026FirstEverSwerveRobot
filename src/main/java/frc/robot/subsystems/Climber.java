package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.subsystemCanIds;
import frc.robot.Constants.speeds;

public class Climber extends SubsystemBase {
    private SparkMax rightMotor = new SparkMax(subsystemCanIds.climberMotorRight, MotorType.kBrushless);
    private SparkMax leftMotor = new SparkMax(subsystemCanIds.climberMotorLeft, MotorType.kBrushless);
    
    private Spark pushingOutMotor = new Spark(1);
    
    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    
    public Climber() {
        leftMotorConfig.follow(rightMotor, true);
        leftMotorConfig.smartCurrentLimit(40);
        rightMotorConfig.smartCurrentLimit(40);
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

    
    public double getEncoderValues() {
        return rightMotor.getEncoder().getPosition();
    }
}
