package frc.robot.commands;

import java.util.LinkedList;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.SysIdCommand.DriveTrainSysIdData;
import frc.robot.subsystems.drive.Drive;

public class OldCharacterizationCommand extends CommandBase {
  private final Drive drive;
  private final LinkedList<Double> entries = new LinkedList<>();
  private double startTime = 0;
  private int counter = 0;
  private double priorAutospeed;

  NetworkTableEntry autoSpeedEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  public OldCharacterizationCommand(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    counter = 0;
    priorAutospeed = 0;
    entries.clear();
    NetworkTableInstance.getDefault().setUpdateRate(0.01);
  }

  @Override
  public void execute() {
    DriveTrainSysIdData data = drive.getSysIdData();

    double autospeed = autoSpeedEntry.getDouble(0);
    double battery = RobotController.getBatteryVoltage();
    double motorVolts = battery * Math.abs(priorAutospeed);
    priorAutospeed = autospeed;

    drive.driveVoltage(
        (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed * 12,
        autospeed * 12);

    entries.add(Timer.getFPGATimestamp());
    entries.add(battery);
    entries.add(autospeed);
    entries.add(motorVolts);
    entries.add(motorVolts);
    entries.add(data.leftPosRad);
    entries.add(data.rightPosRad);
    entries.add(data.leftVelRadPerSec);
    entries.add(data.rightVelRadPerSec);
    entries.add(data.gyroPosRad);

    counter++;
  }

  @Override
  public void end(boolean interrupted) {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    drive.stop();

    String data = entries.toString();
    data = data.substring(1, data.length() - 1) + ", ";
    telemetryEntry.setString(data);
    System.out.println("Robot disabled");
    System.out
        .println("Collected: " + counter + " in " + elapsedTime + " seconds");

    NetworkTableInstance.getDefault().setUpdateRate(0.1);
  }

}
