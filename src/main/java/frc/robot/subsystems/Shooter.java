package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private SparkMax rightShooterMotor = new SparkMax(Constants.subsystemCanIds.lowerRightShooterMotor, MotorType.kBrushless);
    private SparkMax leftShooterMotor = new SparkMax(Constants.subsystemCanIds.lowerLeftShooterMotor, MotorType.kBrushless);
    private SparkMax kickerMotor = new SparkMax(Constants.subsystemCanIds.kickerMotor, MotorType.kBrushless);
    private SparkMax topRightShooterMotor = new SparkMax(Constants.subsystemCanIds.topRightShooterMotor, MotorType.kBrushless);
    private SparkMax topLeftShooterMotor = new SparkMax(Constants.subsystemCanIds.topLeftShooterMotor, MotorType.kBrushless);
    private SparkMax rollerMotor = new SparkMax(Constants.subsystemCanIds.shooterRollerMotor, MotorType.kBrushed);
    
    SparkMaxConfig shooterRightMotorConfig = new SparkMaxConfig();
    SparkMaxConfig shooterLeftMotorConfig = new SparkMaxConfig();
    SparkMaxConfig kickerMotorConfig = new SparkMaxConfig();  
    SparkMaxConfig topRightShooterMotorConfig = new SparkMaxConfig();
    SparkMaxConfig topLeftShooterMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    
    public Shooter() {
        topLeftShooterMotorConfig.follow(topRightShooterMotor, true);
        topRightShooterMotorConfig.apply(topRightShooterMotorConfig);
        topLeftShooterMotorConfig.apply(topLeftShooterMotorConfig);
        shooterLeftMotorConfig.follow(rightShooterMotor, true);
        shooterLeftMotorConfig.apply(shooterLeftMotorConfig);
        shooterRightMotorConfig.apply(shooterRightMotorConfig);
        rollerMotorConfig.apply(rollerMotorConfig);

        rightShooterMotor.configure(shooterRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftShooterMotor.configure(shooterLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kickerMotor.configure(kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        topRightShooterMotor.configure(topRightShooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        topLeftShooterMotor.configure(topLeftShooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void shootFuel(boolean isTracked, double desiredSpeed) {
        // If the camera is tracking the april tags on the hub, use the passed calculated
        // shooter speed in Shoot.java. Otherwise, use the default shooter speed
        if (isTracked == true) {
            rightShooterMotor.set(-desiredSpeed);
            topRightShooterMotor.set(desiredSpeed);

        } else {
            rightShooterMotor.set(-Constants.speeds.defaultShooterMotorSpeed);
            topRightShooterMotor.set(Constants.speeds.defaultShooterMotorSpeed);

        }
        rollerMotor.set(Constants.speeds.shooterRollerMotorSpeed);
        kickerMotor.set(Constants.speeds.kickerMotorSpeed);
    }

    public void stop() {
        rightShooterMotor.set(0.0);
        kickerMotor.set(0.0);
        topRightShooterMotor.set(0.0);
        rollerMotor.set(0.0);
    }
}
