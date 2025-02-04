// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Interface for all driver and operator controls (either single or dual Xbox).
 */
public interface HandheldOI {

  public default double getLeftDriveX() {
    return 0.0;
  }

  public default double getLeftDriveY() {
    return 0.0;
  }

  public default double getRightDriveX() {
    return 0.0;
  }

  public default double getRightDriveY() {
    return 0.0;
  }

  public default Trigger getSniperModeButton() {
    return new Trigger();
  }

  public default Trigger getAutoDriveButton() {
    return new Trigger();
  }

  public default Trigger getAutoAimButton() {
    return new Trigger();
  }

  public default Trigger getShootButton() {
    return new Trigger();
  }

  public default Trigger getLogMarkerButton() {
    return new Trigger();
  }

  public default Trigger getIntakeExtendButton() {
    return new Trigger();
  }

  public default Trigger getIntakeRetractButton() {
    return new Trigger();
  }

  public default Trigger getIntakeForwardsButton() {
    return new Trigger();
  }

  public default Trigger getIntakeBackwardsButton() {
    return new Trigger();
  }

  public default Trigger getStopFlywheelButton() {
    return new Trigger();
  }

  public default Trigger getStartLowerFenderButton() {
    return new Trigger();
  }

  public default Trigger getStartUpperFenderButton() {
    return new Trigger();
  }

  public default Trigger getStartUpperTarmacButton() {
    return new Trigger();
  }

  public default Trigger getTowerUpButton() {
    return new Trigger();
  }

  public default Trigger getTowerDownButton() {
    return new Trigger();
  }

  public default Trigger getClimbTop() {
    return new Trigger();
  }

  public default Trigger getClimbBottom() {
    return new Trigger();
  }

  public default Trigger getClimbAuto() {
    return new Trigger();
  }

  public default double getClimbStick() {
    return 0.0;
  }

  public default void setDriverRumble(double percent) {}

  public default void setOperatorRumble(double percent) {}
}
