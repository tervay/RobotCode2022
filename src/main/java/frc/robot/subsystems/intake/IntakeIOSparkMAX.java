// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.util.SparkMAXBurnManager;

public class IntakeIOSparkMAX implements IntakeIO {
  private boolean invertRoller = false;
  private boolean invertHopper = false;

  private double hopperAfterEncoderReduction = 1.0;
  private double rollerAfterEncoderReduction = 1.0;
  private final CANSparkMax rollerMotor;
  private final CANSparkMax hopperMotor;
  private final RelativeEncoder rollerEncoder;
  private final RelativeEncoder hopperEncoder;

  private final DoubleSolenoid solenoid;

  public IntakeIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        solenoid = new DoubleSolenoid(Pneumatics.revModuleID,
            PneumaticsModuleType.REVPH, 2, 5);
        rollerMotor = new CANSparkMax(4, MotorType.kBrushless);
        hopperMotor = new CANSparkMax(13, MotorType.kBrushless);
        rollerAfterEncoderReduction = 60.0 / 16.0;
        hopperAfterEncoderReduction = 9.0 * (48.0 / 24.0);
        invertRoller = true;
        invertHopper = false;

        rollerEncoder = rollerMotor.getEncoder();
        hopperEncoder = hopperMotor.getEncoder();
        break;
      default:
        throw new RuntimeException("Invalid robot for IntakeIOSparkMax!");
    }

    if (SparkMAXBurnManager.shouldBurn()) {
      rollerMotor.restoreFactoryDefaults();
      hopperMotor.restoreFactoryDefaults();
    }

    rollerMotor.setInverted(invertRoller);
    rollerMotor.setSmartCurrentLimit(30);
    rollerMotor.enableVoltageCompensation(12.0);

    hopperMotor.setInverted(invertHopper);
    hopperMotor.setSmartCurrentLimit(20);
    hopperMotor.enableVoltageCompensation(12.0);

    rollerMotor.setCANTimeout(0);
    hopperMotor.setCANTimeout(0);

    if (SparkMAXBurnManager.shouldBurn()) {
      rollerMotor.burnFlash();
      hopperMotor.burnFlash();
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.extended = solenoid.get() == Value.kReverse;

    inputs.rollerPositionRad =
        Units.rotationsToRadians(rollerEncoder.getPosition())
            / rollerAfterEncoderReduction;
    inputs.rollerVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rollerEncoder.getVelocity())
            / rollerAfterEncoderReduction;
    inputs.rollerAppliedVolts =
        rollerMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.rollerCurrentAmps = new double[] {rollerMotor.getOutputCurrent()};
    inputs.rollerTempCelcius = new double[] {rollerMotor.getMotorTemperature()};

    inputs.hopperPositionRad =
        Units.rotationsToRadians(hopperEncoder.getPosition())
            / hopperAfterEncoderReduction;
    inputs.hopperVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(hopperEncoder.getVelocity())
            / hopperAfterEncoderReduction;
    inputs.hopperAppliedVolts =
        hopperMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.hopperCurrentAmps = new double[] {hopperMotor.getOutputCurrent()};
    inputs.hopperTempCelcius = new double[] {hopperMotor.getMotorTemperature()};
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }

  @Override
  public void setHopperVoltage(double volts) {
    hopperMotor.setVoltage(volts);
  }

  @Override
  public void setRollerBrakeMode(boolean enable) {
    rollerMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setHopperBrakeMode(boolean enable) {
    hopperMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setExtended(boolean extended) {
    solenoid.set(extended ? Value.kReverse : Value.kForward);
  }
}
