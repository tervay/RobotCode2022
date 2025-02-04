// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoAimSimple;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FiveCargoAuto;
import frc.robot.commands.FourCargoAuto;
import frc.robot.commands.FourCargoAutoCross;
import frc.robot.commands.HPPractice;
import frc.robot.commands.IdleHood;
import frc.robot.commands.OneCargoAuto;
import frc.robot.commands.PrepareShooterAuto;
import frc.robot.commands.PrepareShooterPreset;
import frc.robot.commands.ResetClimber;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunClimberToPosition;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunTower;
import frc.robot.commands.Shoot;
import frc.robot.commands.Taxi;
import frc.robot.commands.ThreeCargoAuto;
import frc.robot.commands.TrackWidthCharacterization;
import frc.robot.commands.TwoCargoAuto;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.PrepareShooterPreset.ShooterPreset;
import frc.robot.oi.HandheldOI;
import frc.robot.oi.OISelector;
import frc.robot.oi.OverrideOI;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkMAX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIORomi;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSparkMAX;
import frc.robot.subsystems.drive.DriveIOTalonSRX;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsIO;
import frc.robot.subsystems.flywheels.FlywheelsIOSim;
import frc.robot.subsystems.flywheels.FlywheelsIOSparkMAX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMAX;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOSparkMAX;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.LedsIO;
import frc.robot.subsystems.leds.LedsIORio;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.subsystems.pneumatics.PneumaticsIO;
import frc.robot.subsystems.pneumatics.PneumaticsIOCTRE;
import frc.robot.subsystems.pneumatics.PneumaticsIOREV;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerIO;
import frc.robot.subsystems.tower.TowerIOSparkMAX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.Alert;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedChoosers;
import frc.robot.util.SparkMAXBurnManager;
import frc.robot.util.Alert.AlertType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private Drive drive;
  private Vision vision;
  private Flywheels flywheels;
  private Hood hood;
  private Kicker kicker;
  private Tower tower;
  private Intake intake;
  private Climber climber;
  private Pneumatics pneumatics;
  private Leds leds;

  // OI objects
  private OverrideOI overrideOI = new OverrideOI();
  private HandheldOI handheldOI = new HandheldOI() {};

  // Choosers
  private final LoggedChoosers choosers = new LoggedChoosers();
  private final Map<String, AutoRoutine> autoRoutineMap =
      new HashMap<String, AutoRoutine>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Check if flash should be burned
    SparkMAXBurnManager.update();

    // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2022C:
          drive = new Drive(new DriveIOSparkMAX());
          vision = new Vision(new VisionIOLimelight());
          flywheels = new Flywheels(new FlywheelsIOSparkMAX());
          hood = new Hood(new HoodIOReal());
          kicker = new Kicker(new KickerIOSparkMAX());
          tower = new Tower(new TowerIOSparkMAX());
          intake = new Intake(new IntakeIOSparkMAX());
          climber = new Climber(new ClimberIOSparkMAX());
          pneumatics = new Pneumatics(new PneumaticsIOREV());
          leds = new Leds(new LedsIORio());
          break;
        case ROBOT_2022P:
          drive = new Drive(new DriveIOSparkMAX());
          flywheels = new Flywheels(new FlywheelsIOSim());
          break;
        case ROBOT_2020:
          drive = new Drive(new DriveIOSparkMAX());
          pneumatics = new Pneumatics(new PneumaticsIOCTRE());
          break;
        case ROBOT_KITBOT:
          drive = new Drive(new DriveIOTalonSRX());
          break;
        case ROBOT_SIMBOT:
          drive = new Drive(new DriveIOSim());
          flywheels = new Flywheels(new FlywheelsIOSim());
          break;
        case ROBOT_ROMI:
          drive = new Drive(new DriveIORomi());
          break;
      }
    }

    // Instantiate missing subsystems
    drive = drive != null ? drive : new Drive(new DriveIO() {});
    vision = vision != null ? vision : new Vision(new VisionIO() {});
    flywheels =
        flywheels != null ? flywheels : new Flywheels(new FlywheelsIO() {});
    hood = hood != null ? hood : new Hood(new HoodIO() {});
    kicker = kicker != null ? kicker : new Kicker(new KickerIO() {});
    tower = tower != null ? tower : new Tower(new TowerIO() {});
    intake = intake != null ? intake : new Intake(new IntakeIO() {});
    climber = climber != null ? climber : new Climber(new ClimberIO() {});
    pneumatics =
        pneumatics != null ? pneumatics : new Pneumatics(new PneumaticsIO() {});
    leds = leds != null ? leds : new Leds(new LedsIO() {});

    // Set up subsystems
    drive.setSuppliers(() -> overrideOI.getDriveDisable(),
        () -> overrideOI.getOpenLoop(), () -> overrideOI.getInternalEncoders(),
        hood::getState);
    drive.setDefaultCommand(new DriveWithJoysticks(drive,
        () -> choosers.getJoystickMode(), () -> handheldOI.getLeftDriveX(),
        () -> handheldOI.getLeftDriveY(), () -> handheldOI.getRightDriveX(),
        () -> handheldOI.getRightDriveY(),
        () -> handheldOI.getSniperModeButton().get()));
    drive.setLeds(leds);
    vision.setSuppliers(() -> overrideOI.getVisionLedMode(),
        () -> overrideOI.getClimbMode(), hood::getState);
    vision.setTranslationConsumer(drive::addVisionMeasurement);
    flywheels.setLeds(leds);
    hood.setDefaultCommand(
        new IdleHood(hood, drive, () -> overrideOI.getVisionLedMode()));
    tower.setLeds(leds);
    tower.setOverride(() -> overrideOI.getCargoSensorDisable());

    // Set up auto routines
    autoRoutineMap.put("Do Nothing",
        new AutoRoutine(AutoPosition.ORIGIN, true, new InstantCommand()));

    autoRoutineMap.put("Five cargo (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, false, new FiveCargoAuto(drive,
            vision, flywheels, hood, tower, kicker, intake, leds)));
    autoRoutineMap.put("Four cargo, standard (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, false, new FourCargoAuto(drive,
            vision, flywheels, hood, tower, kicker, intake, leds)));
    autoRoutineMap.put("Four cargo, cross field (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, false, new FourCargoAutoCross(
            drive, vision, flywheels, hood, tower, kicker, intake, leds)));
    autoRoutineMap.put("Three cargo (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, false, new ThreeCargoAuto(drive,
            vision, flywheels, hood, tower, kicker, intake, leds)));

    autoRoutineMap.put("Two cargo (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, false,
            new TwoCargoAuto(true, AutoPosition.TARMAC_A, drive, vision,
                flywheels, hood, tower, kicker, intake, leds)));
    autoRoutineMap.put("Two cargo (TC)",
        new AutoRoutine(AutoPosition.TARMAC_C, false,
            new TwoCargoAuto(true, AutoPosition.TARMAC_C, drive, vision,
                flywheels, hood, tower, kicker, intake, leds)));
    autoRoutineMap.put("Two cargo (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, false,
            new TwoCargoAuto(true, AutoPosition.TARMAC_D, drive, vision,
                flywheels, hood, tower, kicker, intake, leds)));

    autoRoutineMap.put("One cargo (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, true, new OneCargoAuto(false,
            drive, vision, flywheels, hood, tower, kicker, leds)));
    autoRoutineMap.put("One cargo (TB)",
        new AutoRoutine(AutoPosition.TARMAC_B, true, new OneCargoAuto(false,
            drive, vision, flywheels, hood, tower, kicker, leds)));
    autoRoutineMap.put("One cargo (TC)",
        new AutoRoutine(AutoPosition.TARMAC_C, true, new OneCargoAuto(false,
            drive, vision, flywheels, hood, tower, kicker, leds)));
    autoRoutineMap.put("One cargo (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, true, new OneCargoAuto(false,
            drive, vision, flywheels, hood, tower, kicker, leds)));
    autoRoutineMap.put("One cargo (FA)",
        new AutoRoutine(AutoPosition.FENDER_A, true, new OneCargoAuto(true,
            drive, vision, flywheels, hood, tower, kicker, leds)));
    autoRoutineMap.put("One cargo (FB)",
        new AutoRoutine(AutoPosition.FENDER_B, true, new OneCargoAuto(true,
            drive, vision, flywheels, hood, tower, kicker, leds)));

    autoRoutineMap.put("Taxi (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, true, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (TB)",
        new AutoRoutine(AutoPosition.TARMAC_B, true, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (TC)",
        new AutoRoutine(AutoPosition.TARMAC_C, true, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, true, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (FA)",
        new AutoRoutine(AutoPosition.FENDER_A, true, new Taxi(drive, true)));
    autoRoutineMap.put("Taxi (FB)",
        new AutoRoutine(AutoPosition.FENDER_B, true, new Taxi(drive, true)));

    autoRoutineMap.put("HP Practice", new AutoRoutine(AutoPosition.ORIGIN,
        false, new HPPractice(drive, intake, tower, kicker, leds)));

    autoRoutineMap.put("Track Width Characterization",
        new AutoRoutine(AutoPosition.ORIGIN, false,
            new TrackWidthCharacterization(drive, drive::driveVoltage,
                drive::getLeftPositionMeters, drive::getRightPositionMeters,
                drive::getCharacterizationGyroPosition)));

    FeedForwardCharacterizationData driveLeftData =
        new FeedForwardCharacterizationData("Drive/Left");
    FeedForwardCharacterizationData driveRightData =
        new FeedForwardCharacterizationData("Drive/Right");
    autoRoutineMap.put("FF Characterization (Drive/Forwards)",
        new AutoRoutine(AutoPosition.ORIGIN, false,
            new FeedForwardCharacterization(drive, true, driveLeftData,
                driveRightData, drive::driveVoltage,
                drive::getCharacterizationVelocityLeft,
                drive::getCharacterizationVelocityRight)));
    autoRoutineMap.put("FF Characterization (Drive/Backwards)",
        new AutoRoutine(AutoPosition.ORIGIN, false,
            new FeedForwardCharacterization(drive, false, driveLeftData,
                driveRightData, drive::driveVoltage,
                drive::getCharacterizationVelocityLeft,
                drive::getCharacterizationVelocityRight)));

    FeedForwardCharacterizationData flywheelData =
        new FeedForwardCharacterizationData("Flywheels");
    autoRoutineMap.put("FF Characterization (Flywheels/Forwards)",
        new AutoRoutine(AutoPosition.ORIGIN, false,
            new FeedForwardCharacterization(flywheels, true, flywheelData,
                volts -> flywheels.runVoltage(volts),
                flywheels::getCharacterizationVelocity)));
    autoRoutineMap.put("FF Characterization (Flywheels/Backwards)",
        new AutoRoutine(AutoPosition.ORIGIN, false,
            new FeedForwardCharacterization(flywheels, false, flywheelData,
                volts -> flywheels.runVoltage(volts),
                flywheels::getCharacterizationVelocity)));

    // Alert if in tuning mode
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, expect decreased network performance.",
          AlertType.INFO).set(true);
    }

    // Instantiate OI classes and bind buttons
    updateOI();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().clearButtons();
    overrideOI = OISelector.findOverrideOI();
    handheldOI = OISelector.findHandheldOI();

    handheldOI.getLogMarkerButton().whileActiveContinuous(new RunCommand(
        () -> Logger.getInstance().recordOutput("Marker", true)));

    // *** DRIVER CONTROLS ***
    handheldOI.getAutoDriveButton().whileActiveContinuous(
        new DriveToTarget(drive, vision, handheldOI::getLeftDriveY));

    Trigger simpleAutoAim = new Trigger(overrideOI::getSimpleAutoAim);
    handheldOI.getAutoAimButton().and(simpleAutoAim.negate())
        .whileActiveOnce(new AutoAim(drive, vision, handheldOI::getLeftDriveY));
    handheldOI.getAutoAimButton().and(simpleAutoAim).whileActiveOnce(
        new AutoAimSimple(drive, vision, handheldOI::getLeftDriveY));

    Trigger flywheelsReady = new Trigger(flywheels::profileComplete);
    handheldOI.getShootButton().and(flywheelsReady).whileActiveContinuous(
        new Shoot(tower, kicker, hood, leds, handheldOI::setDriverRumble),
        false);

    // *** OPERATOR CONTROLS ***
    Trigger climbMode = new Trigger(overrideOI::getClimbMode);
    Trigger normalMode = climbMode.negate();

    handheldOI.getIntakeExtendButton().and(normalMode)
        .whenActive(intake::extend, intake);
    handheldOI.getIntakeRetractButton().and(normalMode)
        .whenActive(intake::retract, intake);
    handheldOI.getIntakeForwardsButton().and(normalMode)
        .whileActiveContinuous(new RunIntake(true, intake, tower, kicker, leds,
            handheldOI::setOperatorRumble));
    handheldOI.getIntakeBackwardsButton().and(normalMode)
        .whileActiveContinuous(new RunIntake(false, intake, tower, kicker, leds,
            handheldOI::setOperatorRumble));

    Command lowerFenderCommand =
        new PrepareShooterAuto(false, flywheels, hood, tower, drive);
    Command upperFenderCommand = new PrepareShooterPreset(flywheels, hood,
        tower, ShooterPreset.UPPER_FENDER);
    Command upperTarmacCommand = new PrepareShooterPreset(flywheels, hood,
        tower, ShooterPreset.UPPER_TARMAC);

    handheldOI.getStartLowerFenderButton().and(normalMode)
        .whenActive(lowerFenderCommand);
    handheldOI.getStartUpperFenderButton().and(normalMode)
        .whenActive(upperFenderCommand);
    handheldOI.getStartUpperTarmacButton().and(normalMode)
        .whenActive(upperTarmacCommand);
    handheldOI.getStopFlywheelButton().and(normalMode)
        .cancelWhenActive(lowerFenderCommand)
        .cancelWhenActive(upperFenderCommand)
        .cancelWhenActive(upperTarmacCommand);

    handheldOI.getTowerUpButton().and(normalMode)
        .whileActiveContinuous(new RunTower(tower, true));
    handheldOI.getTowerDownButton().and(normalMode)
        .whileActiveContinuous(new RunTower(tower, false));

    // *** CLIMB CONTROLS ***
    climbMode.whileActiveContinuous(new StartEndCommand(
        () -> leds.setClimbing(true), () -> leds.setClimbing(false)));
    climbMode.whileActiveContinuous(new RunClimber(climber,
        handheldOI::getClimbStick, overrideOI::getClimbOpenLoop));
    climbMode.cancelWhenActive(lowerFenderCommand)
        .cancelWhenActive(upperFenderCommand)
        .cancelWhenActive(upperTarmacCommand);
    climbMode.whenActive(intake::retract, intake);
    climbMode.whileActiveContinuous(
        new RunCommand(() -> hood.moveToPosition(true), hood));

    Trigger climbClosedLoop =
        new Trigger(overrideOI::getClimbOpenLoop).negate();
    handheldOI.getClimbTop().and(climbMode).and(climbClosedLoop)
        .toggleWhenActive(new RunClimberToPosition(climber, true), false);
    handheldOI.getClimbBottom().and(climbMode).and(climbClosedLoop)
        .toggleWhenActive(new RunClimberToPosition(climber, false), false);
    handheldOI.getClimbAuto().and(climbMode).and(climbClosedLoop)
        .toggleWhenActive(new AutoClimb(climber, drive, leds), false);
  }

  /** Called at the start of teleop to begin zeroing the climber. */
  public void resetClimber() {
    new ResetClimber(climber).schedule(false);
  }

  /** Updates the LED mode periodically. */
  public void updateLeds() {
    leds.update();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String routineString = choosers.getAutoRoutine();
    if (autoRoutineMap.containsKey(routineString)) {
      AutoRoutine routine = autoRoutineMap.get(routineString);
      drive.setPose(routine.position.getPose(), true);
      drive.resetOnNextVision();
      vision.setAutoEnabled(routine.useVision);
      return routine.command;

    } else {
      DriverStation.reportError("Unknown auto routine: '" + routineString + "'",
          false);
      return null;
    }
  }

  private static class AutoRoutine {
    public final AutoPosition position;
    public final boolean useVision;
    public final Command command;

    public AutoRoutine(AutoPosition position, boolean useVision,
        Command command) {
      this.position = position;
      this.useVision = useVision;
      this.command = command;
    }
  }

  public static enum AutoPosition {
    ORIGIN, TARMAC_A, TARMAC_B, TARMAC_C, TARMAC_D, FENDER_A, FENDER_B;

    public Pose2d getPose() {
      switch (this) {
        case ORIGIN:
          return new Pose2d();
        case TARMAC_A:
          return FieldConstants.referenceA
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.7));
        case TARMAC_B:
          return FieldConstants.referenceB
              .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.2));
        case TARMAC_C:
          return FieldConstants.referenceC
              .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.1));
        case TARMAC_D:
          return FieldConstants.referenceD
              .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.7));
        case FENDER_A:
          return FieldConstants.fenderA
              .transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
        case FENDER_B:
          return FieldConstants.fenderB
              .transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
        default:
          return new Pose2d();
      }
    }
  }
}
