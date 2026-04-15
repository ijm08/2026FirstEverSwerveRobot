// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public static class OperatorConstants {
    // This is the port you must connect your joystick to, I believe
    public static final int kDriverControllerPort = 1;
  }
  public static final class DriveConstants {
    // The position of the camera relative to the centre-floor of the robot
    public static final Transform3d kRobotCameraPosition = new Transform3d(
      new Translation3d(0.361, 0.0, 0.431),
      new Rotation3d(0, Math.toRadians(60), 0)
    );
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAngularSpeed = Math.PI/5 ; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int FrontLeftDrivingCANId = 2;
    public static final int RearLeftDrivingCANId = 7;
    public static final int FrontRightDrivingCANId = 3;
    public static final int RearRightDrivingCANId = 6;

    public static final int FrontLeftTurningCANId = 1;
    public static final int RearLeftTurningCANId = 8;
    public static final int FrontRightTurningCANId = 4;
    public static final int RearRightTurningCANId = 5;

    //PID Gains for the drive train. These are unrelated to Path Planner's path finding
    // as these gains simply control any errors with the wheels maintaining a desired rotation and velocity setpoint
    public static final double kTranslationP = 0.04;
    public static final double kTranslationI = 0.0;
    public static final double kTranslationD = 0.0;

    public static final double kSteeringP = 1.00;
    public static final double kSteeringI = 0.0;
    public static final double kSteeringD = 0.0;

    // -1 is reversed, 1 not
    public static final double kGyroReversed = -1.0;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (42.0 * 20) / (kDrivingMotorPinionTeeth * 16);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
    public static final double kDriveXYDeadband = 0.35;
    public static final double kDriveZDeadband = 0.8;
  
    // Buttons for everything
    public static final int shootButton = 1;
    public static final int lockRobotButton = 2;
    public static final int zeroHeadingButton = 8;
    public static final int extendIntakeButton = 3;
    public static final int retractIntakeButton = 5;
    public static final int extendClimberButton = 6;
    public static final int retractClimberButton = 4;
  }

  public static final class AutoConstants {
    // Constants for the old REV MAX Swerve java template code that was 
    // Originally taken from the internet
    // These are no longer needed as we now use Path Planner which handles velocity/
    // Acceleration constraints in the GUI

    /*  public static final double kMaxSpeedMetersPerSecond = 0.3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI/5;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI/5;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 0.00; 
    
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    */

    //PID Gains for Path Planner NO TOUCHIE TOUCHIE!!!
    public static final double kTranslationProportionalGain = 3.0;
    public static final double kTranslationIntegralGain = 0.0;
    public static final double kTranslationDerivativeGain = 0.025;

    public static final double kTurningProportionalGain = 1.0;
    public static final double kTurningIntegralGain = 0.0;
    public static final double kTurningDerivativeGain = 0.05; 
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class subsystemCanIds {
    // CAN IDs for all motors not part of the Swerve Subsystem
    // Shooter
    public static final int lowerRightShooterMotor = 40;
    public static final int lowerLeftShooterMotor = 41;
    public static final int kickerMotor = 29;
    public static final int topRightShooterMotor = 30;
    public static final int topLeftShooterMotor = 31;
    public static final int shooterRollerMotor = 13;
    // Intake (Arm and intake rollers)
    public static final int intakeMotor = 12;
    public static final int leftIntakeArmMotor = 10;
    public static final int rightIntakeArmMotor = 11;
    // Climber 
    public static final int climberMotorRight = 60;
    public static final int climberMotorLeft = 61;
  }
  public static final class speeds {
    // Im just picking random values for now since our team is quite slow at building stuff
    // Everything is pretty slow for now (should be)
    public static final double topShooterMotorSpeed = 1.0;
    public static final double bottomShooterMotorSpeed = 0.5;
    public static final double shooterRollerMotorSpeed = -1.0;
    public static final double kickerMotorSpeed = 0.5;
    public static final double intakeArmMotorSpeed = 0.2;
    public static final double intakeRollerMotorSpeed = -0.4;
    public static final double climberSpeed = 0.5;
  }
  public static final class encoderPositions {
    // Random numbers for now
    public static final double climberExtended = -0.67;
    public static final double climberRetracted = 0.067;
    public static final double climberEncoderTolerance = 1.5;
    public static final double intakeExtended = -22.0;
    public static final double intakeRetracted = -12.0;
    public static final double intakeTolerance = 0.067; 
  }
}
