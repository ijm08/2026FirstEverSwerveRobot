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
    private boolean isAuto;
    private VisionSubsystem camera = new VisionSubsystem();

    private final double shooterMinSpeed = 0.40; // Needs to be tuned later once testing starts
    private final double shooterMaxSpeed = 1.0;
    private double shooterSpeed = 0.0;

    public Shoot(Shooter subsystem, LEDSubsystem desiredLEDSubsystem, VisionSubsystem processingCamera, boolean auto) {
        shootSignal = desiredLEDSubsystem;
        shooter = subsystem;
        isAuto = auto;
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
            SmartDashboard.putNumber("motor speed", shooterSpeed);
            shooterSpeed = calculateShooterSpeed(distance);
            shooter.shootFuel(true, shooterSpeed);
        } else {
            shooter.shootFuel(false, 0.0);
        } 
        shootSignal.sendSignalToArduino();
    }

    @Override
    public boolean isFinished() {
        if (isAuto == true) {
            return true;
        } else {
            return false;
        }
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

        double speed = .120 * distance + .15;

        speed = Math.min(speed, shooterMaxSpeed);
        speed = Math.max(speed, shooterMinSpeed);

        return speed;
    } 
}
