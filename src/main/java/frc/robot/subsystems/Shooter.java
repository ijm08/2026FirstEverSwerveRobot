package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private SparkMax rightShooterMotor = new SparkMax(Constants.subsystemCanIds.upperRightShooterMotor, MotorType.kBrushless);
    private SparkMax leftShooterMotor = new SparkMax(Constants.subsystemCanIds.upperLeftShooterMotor, MotorType.kBrushless);
    private SparkMax kickerMotor = new SparkMax(Constants.subsystemCanIds.lowerShooterMotor, MotorType.kBrushless);

    private SparkMaxConfig shooterRightMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig shooterLeftMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig kickerMotorConfig = new SparkMaxConfig();  
    
    public Shooter() {
        shooterLeftMotorConfig.follow(rightShooterMotor, true);
        shooterLeftMotorConfig.apply(shooterLeftMotorConfig);
        shooterRightMotorConfig.apply(shooterRightMotorConfig);

        rightShooterMotor.configure(shooterRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftShooterMotor.configure(shooterLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kickerMotor.configure(kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void shootFuel() {
        rightShooterMotor.set(Constants.speeds.shooterMotorSpeed);
        kickerMotor.set(Constants.speeds.kickerMotorSpeed);
    }

    public void stop() {
        rightShooterMotor.set(0.0);
        kickerMotor.set(0.0);
    }
}
