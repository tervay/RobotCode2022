// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticLauncher extends SubsystemBase {
  private final List<DoubleSolenoid> solenoids =
      List.of(new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 1),
          new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 2, 3),
          new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 4, 5),
          new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 6, 7));

  /** Creates a new PneumaticLauncher. */
  public PneumaticLauncher() {
    setExtended(false);
  }

  @Override
  public void periodic() {}

  public void setExtended(boolean extended) {
    for (DoubleSolenoid solenoid : solenoids) {
      solenoid.set(extended ? Value.kForward : Value.kReverse);
    }
  }
}
