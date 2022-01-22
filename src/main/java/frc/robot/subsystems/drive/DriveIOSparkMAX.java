package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class DriveIOSparkMAX implements DriveIO {
  private final double afterEncoderReduction;

  private final CANSparkMax leftLeader;
  private final CANSparkMax leftFollower;
  private final CANSparkMax rightLeader;
  private final CANSparkMax rightFollower;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final SparkMaxPIDController leftPID;
  private final SparkMaxPIDController rightPID;

  private final AHRS gyro = new AHRS(SPI.Port.kMXP); // SPI currently broken on 2022

  public DriveIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2020:
        afterEncoderReduction = 1.0 / ((9.0 / 62.0) * (18.0 / 30.0));
        leftLeader = new CANSparkMax(3, MotorType.kBrushless);
        leftFollower = new CANSparkMax(12, MotorType.kBrushless);
        rightLeader = new CANSparkMax(16, MotorType.kBrushless);
        rightFollower = new CANSparkMax(15, MotorType.kBrushless);
        break;
      default:
        throw new RuntimeException("Invalid robot for DriveIOSparkMax!");
    }

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();
    leftPID = leftLeader.getPIDController();
    rightPID = rightLeader.getPIDController();

    if (Constants.burnMotorControllerFlash) {
      leftLeader.restoreFactoryDefaults();
      leftFollower.restoreFactoryDefaults();
      rightLeader.restoreFactoryDefaults();
      rightFollower.restoreFactoryDefaults();
    }

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    rightLeader.setInverted(false);
    leftLeader.setInverted(true);

    leftLeader.enableVoltageCompensation(12.0);
    rightLeader.enableVoltageCompensation(12.0);

    leftLeader.setSmartCurrentLimit(80);
    leftFollower.setSmartCurrentLimit(80);
    rightLeader.setSmartCurrentLimit(80);
    rightFollower.setSmartCurrentLimit(80);

    if (Constants.burnMotorControllerFlash) {
      leftLeader.burnFlash();
      leftFollower.burnFlash();
      rightLeader.burnFlash();
      rightFollower.burnFlash();
    }

    gyro.zeroYaw();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionRad =
        leftEncoder.getPosition() * (2.0 * Math.PI) / afterEncoderReduction;
    inputs.rightPositionRad =
        rightEncoder.getPosition() * (2.0 * Math.PI) / afterEncoderReduction;

    inputs.leftVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity())
            / afterEncoderReduction;
    inputs.rightVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity())
            / afterEncoderReduction;

    inputs.leftAppliedVolts = leftLeader.getAppliedOutput() * 12.0;
    inputs.rightAppliedVolts = rightLeader.getAppliedOutput() * 12.0;

    inputs.leftCurrentAmps = new double[] {leftLeader.getOutputCurrent(),
        leftFollower.getOutputCurrent()};

    inputs.rightCurrentAmps = new double[] {rightLeader.getOutputCurrent(),
        rightFollower.getOutputCurrent()};

    inputs.leftTempCelcius = new double[] {leftLeader.getMotorTemperature(),
        leftFollower.getMotorTemperature()};
    inputs.rightTempCelcius = new double[] {rightLeader.getMotorTemperature(),
        rightFollower.getMotorTemperature()};

    inputs.gyroPositionRad = Math.toRadians(gyro.getAngle());
    inputs.gyroVelocityRadPerSec = Math.toRadians(gyro.getRate());
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  @Override
  public void setVelocity(double leftVelocityRadPerSec,
      double rightVelocityRadPerSec, double leftFFVolts, double rightFFVolts) {
    double leftRPM =
        Units.radiansPerSecondToRotationsPerMinute(leftVelocityRadPerSec)
            * afterEncoderReduction;
    double rightRPM =
        Units.radiansPerSecondToRotationsPerMinute(rightVelocityRadPerSec)
            * afterEncoderReduction;
    leftPID.setReference(leftRPM, ControlType.kVelocity, 0, leftFFVolts,
        ArbFFUnits.kVoltage);
    rightPID.setReference(rightRPM, ControlType.kVelocity, 0, rightFFVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
    leftLeader.setIdleMode(mode);
    leftFollower.setIdleMode(mode);
    rightLeader.setIdleMode(mode);
    rightFollower.setIdleMode(mode);
  }

  @Override
  public void configurePID(double kp, double ki, double kd) {
    leftPID.setP(kp);
    leftPID.setI(ki);
    leftPID.setD(kd);

    rightPID.setP(kp);
    rightPID.setI(ki);
    rightPID.setD(kd);
  }
}
