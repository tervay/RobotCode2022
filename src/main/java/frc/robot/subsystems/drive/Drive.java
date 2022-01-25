// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.commands.SysIdCommand.DriveTrainSysIdData;
import frc.robot.subsystems.drive.DriveIO.DriveIOInputs;
import frc.robot.subsystems.vision.Vision.TimestampedTranslation2d;
import frc.robot.util.GeomUtil;
import frc.robot.util.PoseHistory;
import frc.robot.util.TunableNumber;

public class Drive extends SubsystemBase {
  private static final double maxCoastVelocityMetersPerSec = 0.05; // Need to be under this to
                                                                   // switch to coast when disabling
  private static final Transform2d vehicleToCamera = new Transform2d(
      new Translation2d(Units.inchesToMeters(1.875), 0.0), new Rotation2d());
  private static final int poseHistoryCapacity = 500;
  private static final double maxNoVisionLog = 0.1; // How long to wait with no vision data before
                                                    // clearing log visualization
  private static final double visionNominalFramerate = 45;
  private static final double visionShiftPerSec = 0.85; // After one second of vision data, what %
  // of pose average should be vision
  private static final double visionMaxAngularVelocity =
      Units.degreesToRadians(8.0); // Max angular velocity before vision data is rejected

  private final double wheelRadiusMeters;
  private final double maxVelocityMetersPerSec;
  private final double trackWidthMeters;
  private final SimpleMotorFeedforward leftModel, rightModel;
  private final TunableNumber kP = new TunableNumber("Drive/kP");
  private final TunableNumber kD = new TunableNumber("Drive/kD");

  private final DriveIO io;
  private final DriveIOInputs inputs = new DriveIOInputs();

  private Supplier<Boolean> disableOverride = () -> false;
  private Supplier<Boolean> openLoopOverride = () -> false;

  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), new Pose2d());
  private final Timer noVisionTimer = new Timer(); // Time since last vision update
  private PoseHistory poseHistory = new PoseHistory(poseHistoryCapacity);
  private Pose2d lastVisionPose = new Pose2d();
  private double baseDistanceLeftRad = 0.0;
  private double baseDistanceRightRad = 0.0;
  private boolean brakeMode = false;
  private boolean resetOnVision = false;

  /** Creates a new DriveTrain. */
  public Drive(DriveIO io) {
    this.io = io;
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
      case ROBOT_2022P:
        maxVelocityMetersPerSec = Units.inchesToMeters(150.0);
        wheelRadiusMeters = Units.inchesToMeters(2.0);
        trackWidthMeters = 1.5;
        leftModel = new SimpleMotorFeedforward(0.0, 0.2, 0.0);
        rightModel = new SimpleMotorFeedforward(0.0, 0.2, 0.0);
        kP.setDefault(0);
        kD.setDefault(0);
        break;
      case ROBOT_2020:
        maxVelocityMetersPerSec = Units.inchesToMeters(150.0);
        wheelRadiusMeters = Units.inchesToMeters(3.0);
        trackWidthMeters = 1.768748;
        leftModel = new SimpleMotorFeedforward(0.23004, 0.2126, 0.036742);
        rightModel = new SimpleMotorFeedforward(0.22652, 0.21748, 0.03177);
        kP.setDefault(0.00015);
        kD.setDefault(0.0015);
        break;
      case ROBOT_KITBOT:
        maxVelocityMetersPerSec = Units.inchesToMeters(122.0);
        wheelRadiusMeters = Units.inchesToMeters(3.18);
        trackWidthMeters = 0.6928821;
        leftModel = new SimpleMotorFeedforward(0.75379, 0.25162, 0.042941);
        rightModel = new SimpleMotorFeedforward(0.70773, 0.24745, 0.032956);
        kP.setDefault(2);
        kD.setDefault(40);
        break;
      case ROBOT_SIMBOT:
        maxVelocityMetersPerSec = 4.0;
        wheelRadiusMeters = Units.inchesToMeters(3.0);
        trackWidthMeters = 0.354426;
        leftModel = new SimpleMotorFeedforward(0.0, 0.22643, 0.018292);
        rightModel = new SimpleMotorFeedforward(0.0, 0.22643, 0.018292);
        kP.setDefault(0.4);
        kD.setDefault(0.0);
        break;
      case ROBOT_ROMI:
        maxVelocityMetersPerSec = 0.6;
        wheelRadiusMeters = 0.035;
        trackWidthMeters = 0.281092;
        leftModel = new SimpleMotorFeedforward(0.27034, 0.64546, 0.021935);
        rightModel = new SimpleMotorFeedforward(0.48548, 0.37427, 0.07421);
        kP.setDefault(0.25);
        kD.setDefault(0.001);
        break;
      default:
        maxVelocityMetersPerSec = 0;
        wheelRadiusMeters = Double.POSITIVE_INFINITY;
        trackWidthMeters = 1.0;
        leftModel = new SimpleMotorFeedforward(0, 0, 0);
        rightModel = new SimpleMotorFeedforward(0, 0, 0);
        kP.setDefault(0);
        kD.setDefault(0);
        break;
    }

    io.setBrakeMode(false);
    noVisionTimer.start();
  }

  /** Set boolean supplier for the override switches. */
  public void setOverrides(Supplier<Boolean> disableOverride,
      Supplier<Boolean> openLoopOverride) {
    this.disableOverride = disableOverride;
    this.openLoopOverride = openLoopOverride;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);

    // Update odometry
    odometry.update(new Rotation2d(inputs.gyroPositionRad * -1),
        (inputs.leftPositionRad - baseDistanceLeftRad) * wheelRadiusMeters,
        (inputs.rightPositionRad - baseDistanceRightRad) * wheelRadiusMeters);


    // Log robot pose
    Pose2d robotPose = odometry.getPoseMeters();
    poseHistory.insert(Timer.getFPGATimestamp(), robotPose);
    Logger.getInstance().recordOutput("Odometry/Robot",
        new double[] {robotPose.getX(), robotPose.getY(),
            robotPose.getRotation().getRadians()});

    // Log vision pose
    if (noVisionTimer.get() < maxNoVisionLog) {
      Logger.getInstance().recordOutput("Odometry/VisionPose",
          new double[] {lastVisionPose.getX(), lastVisionPose.getY(),
              lastVisionPose.getRotation().getRadians()});
      Logger.getInstance().recordOutput("Odometry/VisionTarget", new double[] {
          FieldConstants.fieldLength / 2.0, FieldConstants.fieldWidth / 2.0});

    }

    // Update brake mode
    if (DriverStation.isEnabled()) {
      if (!brakeMode) {
        brakeMode = true;
        io.setBrakeMode(true);
      }
    } else {
      if (brakeMode
          && Math
              .abs(getLeftVelocityMetersPerSec()) < maxCoastVelocityMetersPerSec
          && Math.abs(
              getRightVelocityMetersPerSec()) < maxCoastVelocityMetersPerSec) {
        brakeMode = false;
        io.setBrakeMode(false);
      }
    }

    // Send tuning constants
    if (kP.hasChanged() | kD.hasChanged()) {
      io.configurePID(kP.get(), 0, kD.get());
    }
  }

  /**
   * Drive at the specified voltage with no other processing. Only use with SysId.
   */
  public void driveVoltage(double leftVolts, double rightVolts) {
    if (disableOverride.get()) {
      io.setVoltage(0, 0);
      return;
    }

    io.setVoltage(leftVolts, rightVolts);
  }

  /**
   * Drive at the specified percentage of max speed.
   */
  public void drivePercent(double leftPercent, double rightPercent) {
    if (disableOverride.get()) {
      io.setVoltage(0, 0);
      return;
    }

    driveVelocity(leftPercent * maxVelocityMetersPerSec,
        rightPercent * maxVelocityMetersPerSec);
  }

  /**
   * Drive at the specified velocity.
   */
  public void driveVelocity(double leftVelocityMetersPerSec,
      double rightVelocityMetersPerSec) {
    if (disableOverride.get()) {
      io.setVoltage(0, 0);
      return;
    }


    double leftVelocityRadPerSec = leftVelocityMetersPerSec / wheelRadiusMeters;
    double rightVelocityRadPerSec =
        rightVelocityMetersPerSec / wheelRadiusMeters;

    double leftFFVolts = leftModel.calculate(leftVelocityRadPerSec);
    double rightFFVolts = rightModel.calculate(rightVelocityRadPerSec);

    if (openLoopOverride.get()) {
      // Use open loop control
      io.setVoltage(leftFFVolts, rightFFVolts);
    } else {
      io.setVelocity(leftVelocityRadPerSec, rightVelocityRadPerSec, leftFFVolts,
          rightFFVolts);
    }
  }

  /**
   * In open loop, goes to neutral. In closed loop, resets velocity setpoint.
   */
  public void stop() {
    drivePercent(0, 0);
  }

  /** Returns the current odometry pose */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Returns the current rotation according to odometry */
  public Rotation2d getRotation() {
    return odometry.getPoseMeters().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose, boolean clearHistory) {
    if (clearHistory) {
      poseHistory = new PoseHistory(poseHistoryCapacity);
    }
    baseDistanceLeftRad = inputs.leftPositionRad;
    baseDistanceRightRad = inputs.rightPositionRad;
    odometry.resetPosition(pose, new Rotation2d(inputs.gyroPositionRad * -1));
  }

  /** Adds a new timestamped vision measurement */
  public void addVisionMeasurement(TimestampedTranslation2d data) {
    Optional<Pose2d> historicalFieldToTarget = poseHistory.get(data.timestamp);
    if (historicalFieldToTarget.isPresent()) {

      // Calculate new robot pose
      Rotation2d robotRotation = historicalFieldToTarget.get().getRotation();
      Rotation2d cameraRotation =
          robotRotation.rotateBy(vehicleToCamera.getRotation());
      Transform2d fieldToTargetRotated =
          new Transform2d(FieldConstants.hubCenter, cameraRotation);
      Transform2d fieldToCamera = fieldToTargetRotated.plus(
          GeomUtil.transformFromTranslation(data.translation.unaryMinus()));
      Pose2d visionFieldToTarget = GeomUtil
          .transformToPose(fieldToCamera.plus(vehicleToCamera.inverse()));

      // Save vision pose for logging
      noVisionTimer.reset();
      lastVisionPose = visionFieldToTarget;

      // Calculate vision percent
      double angularErrorScale =
          Math.abs(inputs.gyroVelocityRadPerSec) / visionMaxAngularVelocity;
      angularErrorScale = MathUtil.clamp(angularErrorScale, 0, 1);
      double visionShift =
          1 - Math.pow(1 - visionShiftPerSec, 1 / visionNominalFramerate);
      visionShift *= 1 - angularErrorScale;

      // Reset pose
      Pose2d currentFieldToTarget = getPose();
      Translation2d fieldToVisionField = new Translation2d(
          visionFieldToTarget.getX() - historicalFieldToTarget.get().getX(),
          visionFieldToTarget.getY() - historicalFieldToTarget.get().getY());
      Pose2d visionLatencyCompFieldToTarget =
          new Pose2d(currentFieldToTarget.getX() + fieldToVisionField.getX(),
              currentFieldToTarget.getY() + fieldToVisionField.getY(),
              currentFieldToTarget.getRotation());

      if (resetOnVision) {
        setPose(new Pose2d(visionFieldToTarget.getX(),
            visionFieldToTarget.getY(), currentFieldToTarget.getRotation()),
            true);
        resetOnVision = false;
      } else {
        setPose(new Pose2d(
            currentFieldToTarget.getX() * (1 - visionShift)
                + visionLatencyCompFieldToTarget.getX() * visionShift,
            currentFieldToTarget.getY() * (1 - visionShift)
                + visionLatencyCompFieldToTarget.getY() * visionShift,
            currentFieldToTarget.getRotation()), false);
      }
    }
  }

  /** Reset odometry on the next vision frame (rather than using averaging). */
  public void resetOnNextVision() {
    resetOnVision = true;
  }

  /** Returns whether the vision reset triggered by "resetOnNextVision()" is complete. */
  public boolean getVisionResetComplete() {
    return !resetOnVision;
  }

  /** Return left velocity in meters per second. */
  public double getLeftVelocityMetersPerSec() {
    return inputs.leftVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Return right velocity in meters per second. */
  public double getRightVelocityMetersPerSec() {
    return inputs.rightVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Return track width in meters. */
  public double getTrackWidthMeters() {
    return trackWidthMeters;
  }

  /** Return average kS. */
  public double getKs() {
    return (leftModel.ks + rightModel.ks) / 2;
  }

  /** Return average kV in (volts * second) / meter. */
  public double getKv() {
    return ((leftModel.kv + rightModel.kv) / 2) / wheelRadiusMeters;
  }

  /** Return average kA in (volts * second^2) / meter. */
  public double getKa() {
    return ((leftModel.ka + rightModel.ka) / 2) / wheelRadiusMeters;
  }

  /**
   * Returns a set of data for SysId
   */
  public DriveTrainSysIdData getSysIdData() {
    return new DriveTrainSysIdData(inputs.leftPositionRad,
        inputs.rightPositionRad, inputs.leftVelocityRadPerSec,
        inputs.rightVelocityRadPerSec, inputs.gyroPositionRad,
        inputs.gyroVelocityRadPerSec);
  }
}
