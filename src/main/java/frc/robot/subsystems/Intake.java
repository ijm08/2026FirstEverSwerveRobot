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
    private SparkMax leftIntakeArmMotor = new SparkMax(Constants.subsystemCanIds.leftIntakeArmMotor, MotorType.kBrushless);
    private SparkMax rightIntakeArmMotor = new SparkMax(Constants.subsystemCanIds.rightIntakeArmMotor, MotorType.kBrushless);
    private SparkFlex intakeMotor = new SparkFlex(Constants.subsystemCanIds.intakeMotor, MotorType.kBrushless);

    SparkMaxConfig leftIntakeArmMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightIntakeArmMotorConfig = new SparkMaxConfig();
    SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();

    private boolean rollersOn = false;

    public Intake() {
        intakeMotor.clearFaults();
        leftIntakeArmMotor.clearFaults();
        rightIntakeArmMotor.clearFaults();
        leftIntakeArmMotorConfig.smartCurrentLimit(80);
        rightIntakeArmMotorConfig.smartCurrentLimit(80);

        // rightIntakeArmMotorConfig.follow(leftIntakeArmMotor, true);

        rightIntakeArmMotorConfig.apply(rightIntakeArmMotorConfig);
        leftIntakeArmMotorConfig.apply(leftIntakeArmMotorConfig);

        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftIntakeArmMotor.configure(leftIntakeArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);       
        rightIntakeArmMotor.configure(rightIntakeArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);       

    }

    // IM A TERRRRRRIIIIIIBLLLLLLLEEEEEEE PROGRAMMER!!!!!!
    // -23 start, -11 up

    public void moveIntakeAutomatically(double desiredIntakeDirection) {
        // System.out.println(getArmEncoderValues());
        leftIntakeArmMotor.set(Constants.speeds.intakeArmMotorSpeed * desiredIntakeDirection);
        rightIntakeArmMotor.set(-1 * (Constants.speeds.intakeArmMotorSpeed * desiredIntakeDirection));
/*         if (desiredIntakeDirection > 0.00) {
            // Up I think, make rollers stop
            intakeMotor.set(Constants.speeds.intakeRollerMotorSpeed);
        } else {
            // When the intake is down, keep the rollers spinning constantly
            // Until it is retracted, at which point stop them
            intakeMotor.set(0.0);
        } */
    }

    // CODE FOR TESTING

/*     public void setRollers(boolean on) {
        rollersOn = on;                                                           
        intakeMotor.set(on ? -0.3 : 0.0);
    }

    public void setArmSpeed(double speed) {
        leftIntakeArmMotor.set(speed);
    } 

    public void toggleRollers() {
        rollersOn = !rollersOn;
        setRollers(rollersOn);
    } */

    // im just including the following method because whatever, even though everything
    // SHOULD, yes, SHOULD (meaning it probably wont), handle everything automatically
    public void stopArmMotor() {
        // intakeArmMotor.set(0.0);

        leftIntakeArmMotor.set(0.0);
        rightIntakeArmMotor.set(0.0);
    }

    public double getArmEncoderValues() {
        return leftIntakeArmMotor.getEncoder().getPosition();
    }

    public void setIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }
}
