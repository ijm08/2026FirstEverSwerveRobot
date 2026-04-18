// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.speeds;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.commands.MoveClimber;
import frc.robot.commands.Shoot;
import frc.robot.commands.MoveIntake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final VisionSubsystem camera = new VisionSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(camera);
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  // Digital outputs to interface with an RGB strip connected
  // to the arduino (if it gets set up), because....
  // LEDS ARE SO COOL

  // private final LEDSubsystem leds = new LEDSubsystem();

  private final LEDSubsystem lockRobotSignal = new LEDSubsystem(0);
  private final LEDSubsystem zeroHeadingSignal = new LEDSubsystem(1);
  private final LEDSubsystem shootSignal = new LEDSubsystem(2);
  private final LEDSubsystem readyToIntakeSignal = new LEDSubsystem(3);
  
  private final SendableChooser<Command> autoChooser;

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("Shoot preloaded", new Shoot(shooter, shootSignal, camera));
    NamedCommands.registerCommand("Shoot Loaded", new Shoot(shooter, shootSignal, camera));
    NamedCommands.registerCommand("WaitThreeSeconds", new WaitCommand(3.0));
    NamedCommands.registerCommand("Stop Shooter", new InstantCommand(shooter::stop));
    NamedCommands.registerCommand("Extend Intake", new MoveIntake(intake, 1.00, readyToIntakeSignal).withTimeout(2.5));
    NamedCommands.registerCommand("Retract Intake", new MoveIntake(intake, -1.00, readyToIntakeSignal).withTimeout(2.5));
    // NamedCommands.registerCommand("Stop Intake", new InstantCommand(intake::stopArmMotor));

    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Choices", autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand( () -> {
            double rot = 0; 
            double yaw;
            double avgYaw;
            double totalYaw = 0;
            int tagID;
            int count = 0;

            if (m_driverController.getTrigger()) {
                var result = camera.gResult();
                var targets = result.getTargets();
                
                if (result.hasTargets()) {
                    if (targets.size() > 2) {
                        for (var target : targets) {
                            if (target.getPoseAmbiguity() < 0.2) {
                                totalYaw += target.getYaw();
                                count++;
                            }
                        }
                        if (count > 0) {
                            avgYaw = totalYaw / count;
                            rot = avgYaw * 0.02;
                            if (Math.abs(avgYaw) < 1.0) {
                                rot = 0;
                            }
                        } else {
                            rot = 0;
                            // yaw = result.getBestTarget().getYaw();
                            // rot = yaw * 0.02;
                        }
                    } else {
                        // Only 2 tags were seen, eventually this will align with the tag 
                        // at the CENTRE OF THE HUB  
                        for (var target : targets) {
                            tagID = target.getFiducialId();
                            if (tagID == 26 || tagID == 10 || tagID == 9) {
                               yaw = target.getYaw();
                               rot = yaw * 0.02;
                               break;
                            }
                        } 
                    }

                } else {
                    rot = 0;
                }
            } else {
                rot = -MathUtil.applyDeadband(m_driverController.getTwist(), OIConstants.kDriveZDeadband);
            }

            m_robotDrive.drive(         
                    -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveXYDeadband),
                    -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveXYDeadband),
                    rot, true);

        }, 
        m_robotDrive)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, OIConstants.lockRobotButton)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.lockRobot(lockRobotSignal),
            m_robotDrive))
        .onFalse(new InstantCommand(lockRobotSignal::turnOffChannel));
    new JoystickButton(m_driverController, OIConstants.zeroHeadingButton)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(zeroHeadingSignal),
            m_robotDrive));
    new JoystickButton(m_driverController, OIConstants.shootButton)
        .whileTrue(new Shoot(shooter, shootSignal, camera))
        .onFalse(new InstantCommand(shooter::stop));
        // EXTEND (arm down + toggle rollers ON)
  /*   new JoystickButton(m_driverController, OIConstants.extendIntakeButton)
        .whileTrue(new RunCommand(
            () -> intake.setArmSpeed(-Constants.speeds.intakeArgmMotorSpeed),
            intake))
        .onFalse(new InstantCommand(() -> {
            intake.setArmSpeed(0.0);
            intake.setRollers(true); // turn rollers ON
        }));

    // RETRACT (arm up + toggle rollers OFF)
    new JoystickButton(m_driverController, OIConstants.retractIntakeButton)
        .whileTrue(new RunCommand(
            () -> intake.setArmSpeed(Constants.speeds.intakeArmMotorSpeed),
            intake))
        .onFalse(new InstantCommand(() -> {
            intake.setArmSpeed(0.0);
            intake.setRollers(false); // turn rollers OFF
        })); */
    new JoystickButton(m_driverController, OIConstants.extendClimberButton)
        .onTrue(new MoveClimber(climber, -1.00))
        .onFalse(new InstantCommand(climber::stop));
    new JoystickButton(m_driverController, OIConstants.retractClimberButton)
        .onTrue(new MoveClimber(climber, 1.00))
        .onFalse(new InstantCommand(climber::stop));
    new JoystickButton(m_driverController, OIConstants.extendIntakeButton)
        .onTrue(new MoveIntake(intake, 1.00, readyToIntakeSignal).withTimeout(2.0))
        .onFalse(new InstantCommand(intake::stopArmMotor));
    // RETRACT (arm up + toggle rollers OFF)
    new JoystickButton(m_driverController, OIConstants.retractIntakeButton)
        .onTrue(new MoveIntake(intake, -1.00, readyToIntakeSignal).withTimeout(2.0))
        .onFalse(new InstantCommand(intake::stopArmMotor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
/*     // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0.01), new Translation2d(2, -0.01), new Translation2d(1, 0.01)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    // m_robotDrive.zeroHeading();
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  */  

    return autoChooser.getSelected();
    }
}