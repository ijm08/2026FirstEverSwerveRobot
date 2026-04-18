package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants;

public class Shoot extends Command {
    private final Shooter shooter;
    private final LEDSubsystem shootSignal;
    private VisionSubsystem camera = new VisionSubsystem();

    private final double shooterMinSpeed = 0.5; // Needs to be tuned later once testing starts
    private final double shooterMaxSpeed = 1.0;
    private double shooterSpeed = 0.0;

    public Shoot(Shooter subsystem, LEDSubsystem desiredLEDSubsystem, VisionSubsystem processingCamera) {
        shootSignal = desiredLEDSubsystem;
        shooter = subsystem;
        this.camera = processingCamera;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        var result = camera.result;
        // shooter.shootFuel(false, 0.0);
        SmartDashboard.putBoolean("hasTargets:", result.hasTargets());
        
        if (result.hasTargets()) {
            Transform3d transform = result.getBestTarget().getBestCameraToTarget();
            // Translation3d translation = transform.getTranslation();
            double distance = transform.getX();


            SmartDashboard.putNumber("Distance", distance);
            SmartDashboard.putNumber("x Dist:", transform.getX());
            SmartDashboard.putNumber("Y Dist", transform.getY());
            shooterSpeed = calculateShooterSpeed(distance);
            shooter.shootFuel(true, shooterSpeed);
        } else {
            shooter.shootFuel(false, 0.0);
        } 
        shootSignal.sendSignalToArduino();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        shootSignal.turnOffChannel();
    }

    private double calculateShooterSpeed(double distance) {
        distance = Math.max(distance, 0.01);

        // At 6ft, 2m the speed should be 50%
        // At 18ft, 6m, the speed should be 100%
        // y = mx + b
        // m = .5 / 4 = .125
        // b = .25

        double speed = .125 * distance + .25;

        speed = Math.min(speed, shooterMaxSpeed);
        speed = Math.max(speed, shooterMinSpeed);

        return speed;
    } 
}
