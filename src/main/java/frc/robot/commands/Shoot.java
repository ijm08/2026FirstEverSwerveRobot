package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Transform3d;

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
        shooter.shootFuel(false, shooterSpeed);

        /*if (result.hasTargets()) {
            Transform3d transform = result.getBestTarget().getBestCameraToTarget();
            double distance = transform.getTranslation().getZ();

            shooterSpeed = calculateShooterSpeed(distance);
            shooter.shootFuel(true, shooterSpeed);
        } else {
            shooter.shootFuel(false, shooterSpeed);
        } */
        shootSignal.sendSignalToArduino();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
    @Override
    public void end(boolean interrupted) {
        shootSignal.turnOffChannel();
    }

    private double calculateShooterSpeed(double area) {
        area = Math.max(area, 0.01);

        double speed = 1.0 / area;

        speed = Math.min(speed, shooterMaxSpeed);
        speed = Math.max(speed, shooterMinSpeed);

        return speed;
    }
}
