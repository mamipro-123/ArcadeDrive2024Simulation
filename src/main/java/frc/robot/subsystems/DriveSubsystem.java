// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax Left_Leader = new CANSparkMax(DriveConstants.kLeftMotor1Port,MotorType.kBrushless);
  private final CANSparkMax Left_Follower = new CANSparkMax(DriveConstants.kLeftMotor2Port,MotorType.kBrushless);

  // The motors on the right side of the drive.
  private final CANSparkMax Right_Leader = new CANSparkMax(DriveConstants.kRightMotor1Port,MotorType.kBrushless);
  private final CANSparkMax Right_Follower = new CANSparkMax(DriveConstants.kRightMotor2Port,MotorType.kBrushless);

  // The robot's drive
  private final DifferentialDrive drive =
      new DifferentialDrive(Left_Leader::set, Right_Leader::set);

  // The left-side drive encoder
  private final Encoder LeftEncoder =
      new Encoder(
          DriveConstants.kLeftEncoderPorts[0],
          DriveConstants.kLeftEncoderPorts[1],
          DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder RightEncoder =
      new Encoder(
          DriveConstants.kRightEncoderPorts[0],
          DriveConstants.kRightEncoderPorts[1],
          DriveConstants.kRightEncoderReversed);

  // The gyro sensor
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry;

  // These classes help us simulate our drivetrain
  public DifferentialDrivetrainSim drivetrainSimulator;
  private final EncoderSim LeftEncoderSim;
  private final EncoderSim RightEncoderSim;
  // The Field2d class shows the field in the sim GUI
  private final Field2d m_fieldSim;
  private final ADXRS450_GyroSim gyroSim;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SendableRegistry.addChild(drive, Left_Leader);
    SendableRegistry.addChild(drive, Right_Leader);


    Left_Follower.follow(Left_Leader);
    Right_Follower.follow(Right_Leader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    Right_Leader.setInverted(true);

    // Sets the distance per pulse for the encoders
    LeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    RightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    odometry =
        new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeading()),
            LeftEncoder.getDistance(),
            RightEncoder.getDistance());

    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      drivetrainSimulator =
          new DifferentialDrivetrainSim(
              DriveConstants.kDrivetrainPlant,
              DriveConstants.kDriveGearbox,
              DriveConstants.kDriveGearing,
              DriveConstants.kTrackwidthMeters,
              DriveConstants.kWheelDiameterMeters / 2.0,
              VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      // The encoder and gyro angle sims let us set simulated sensor readings
      LeftEncoderSim = new EncoderSim(LeftEncoder);
      RightEncoderSim = new EncoderSim(RightEncoder);
      gyroSim = new ADXRS450_GyroSim(gyro);

      // the Field2d class lets us visualize our robot in the simulation GUI.
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);
    } else {
      LeftEncoderSim = null;
      RightEncoderSim = null;
      gyroSim = null;

      m_fieldSim = null;
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        LeftEncoder.getDistance(),
        RightEncoder.getDistance());
    m_fieldSim.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    drivetrainSimulator.setInputs(
        Left_Leader.get() * RobotController.getBatteryVoltage(),
        Right_Leader.get() * RobotController.getBatteryVoltage());
    drivetrainSimulator.update(0.020);

    LeftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters());
    LeftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond());
    RightEncoderSim.setDistance(drivetrainSimulator.getRightPositionMeters());
    RightEncoderSim.setRate(drivetrainSimulator.getRightVelocityMetersPerSecond());
    gyroSim.setAngle(-drivetrainSimulator.getHeading().getDegrees());
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want
   * it to work elsewhere, use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return drivetrainSimulator.getCurrentDrawAmps();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(LeftEncoder.getRate(), RightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    drivetrainSimulator.setPose(pose);
    odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        LeftEncoder.getDistance(),
        RightEncoder.getDistance(),
        pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    Left_Leader.setVoltage(leftVolts);
    Right_Leader.setVoltage(rightVolts);
    drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    LeftEncoder.reset();
    RightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (LeftEncoder.getDistance() + RightEncoder.getDistance()) / 2.0;
  }

  
  public Encoder getLeftEncoder() {
    return LeftEncoder;
  }

  
  public Encoder getRightEncoder() {
    return RightEncoder;
  }

 
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }


  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
