// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.Constants.CanIds.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import lombok.Builder;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
public final class DriveConstants {
  public static final DriveConfig driveConfig =
      switch (Constants.getRobot()) {
        case SIMBOT, COMPBOT -> DriveConfig.builder()
            .wheelRadius(Units.inchesToMeters(1.942))
            .trackWidthX(Units.inchesToMeters(20.75))
            .trackWidthY(Units.inchesToMeters(20.75))
            .bumperWidthX(Units.inchesToMeters(37))
            .bumperWidthY(Units.inchesToMeters(33))
            .maxLinearVelocity(Units.feetToMeters(15.0))
            .maxLinearAcceleration(Units.feetToMeters(75.0))
            .maxAngularVelocity(12.0)
            .maxAngularAcceleration(6.0)
            .build();
        case DEVBOT -> DriveConfig.builder()
            .wheelRadius(Units.inchesToMeters(1))
            .trackWidthX(Units.inchesToMeters(24))
            .trackWidthY(Units.inchesToMeters(24))
            .bumperWidthX(Units.inchesToMeters(26))
            .bumperWidthY(Units.inchesToMeters(26))
            .maxLinearVelocity(Units.feetToMeters(14.0))
            .maxLinearAcceleration(Units.feetToMeters(75.0))
            .maxAngularVelocity(12.0)
            .maxAngularAcceleration(6.0)
            .build();
      };
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(driveConfig.trackWidthX() / 2.0, driveConfig.trackWidthY() / 2.0),
        new Translation2d(driveConfig.trackWidthX() / 2.0, -driveConfig.trackWidthY() / 2.0),
        new Translation2d(-driveConfig.trackWidthX() / 2.0, driveConfig.trackWidthY() / 2.0),
        new Translation2d(-driveConfig.trackWidthX() / 2.0, -driveConfig.trackWidthY() / 2.0)
      };
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);

  // Odometry Constants

  // High priority
  public static final double odometryFrequencyHigh =
      switch (Constants.getRobot()) {
        case SIMBOT, DEVBOT, COMPBOT -> 250.0;
      };
  public static final double odometryFrequencyLow =
      switch (Constants.getRobot()) {
        case SIMBOT -> 50.0;
        case DEVBOT -> 100.0;
        case COMPBOT -> 150.0;
      };

  // Module Constants
  // FL, FR, BL, BR
  public static final ModuleConfig[] moduleConfigs =
      switch (Constants.getRobot()) {
        case COMPBOT -> new ModuleConfig[] {
          new ModuleConfig(
              FL_DRIVE_CAN_ID,
              FL_TURN_CAN_ID,
              FL_CANCODER_CAN_ID,
              "",
              Rotation2d.fromRotations(0.49365234375),
              true),
          new ModuleConfig(
              FR_DRIVE_CAN_ID,
              FR_TURN_CAN_ID,
              FR_CANCODER_CAN_ID,
              "",
              Rotation2d.fromRotations(0.19873046875),
              true),
          new ModuleConfig(
              BL_DRIVE_CAN_ID,
              BL_TURN_CAN_ID,
              BL_CANCODER_CAN_ID,
              "",
              Rotation2d.fromRotations(-0.467041015625),
              true),
          new ModuleConfig(
              BR_DRIVE_CAN_ID,
              BR_TURN_CAN_ID,
              BR_CANCODER_CAN_ID,
              "",
              Rotation2d.fromRotations(-0.300537109375),
              true)
        };
        case DEVBOT -> new ModuleConfig[] {
          new ModuleConfig(
              FL_DRIVE_CAN_ID,
              FL_TURN_CAN_ID,
              FL_CANCODER_CAN_ID,
              "",
              Rotation2d.fromRotations(0.49365234375),
              true),
          new ModuleConfig(
              FR_DRIVE_CAN_ID,
              FR_TURN_CAN_ID,
              FR_CANCODER_CAN_ID,
              "",
              Rotation2d.fromRotations(0.19873046875),
              true),
          new ModuleConfig(
              BL_DRIVE_CAN_ID,
              BL_TURN_CAN_ID,
              BL_CANCODER_CAN_ID,
              "",
              Rotation2d.fromRotations(-0.467041015625),
              true),
          new ModuleConfig(
              BR_DRIVE_CAN_ID,
              BR_TURN_CAN_ID,
              BR_CANCODER_CAN_ID,
              "",
              Rotation2d.fromRotations(-0.300537109375),
              true)
        };
        case SIMBOT -> {
          ModuleConfig[] configs = new ModuleConfig[4];
          for (int i = 0; i < configs.length; i++)
            configs[i] = new ModuleConfig(0, 0, 0, "", new Rotation2d(0), false);
          yield configs;
        }
      };

  public static final ModuleConstants moduleConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new ModuleConstants(
            40.0,
            40.0,
            5.0,
            0.0,
            1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp, // A/(N*m)
            35.0,
            0.0,
            4000.0,
            50.0,
            Mk4iReductions.L3.reduction,
            Mk4iReductions.TURN.reduction);
        case DEVBOT -> new ModuleConstants(
            40.0,
            40.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            7.0,
            0.0,
            Mk4iReductions.L2.reduction,
            Mk4iReductions.TURN.reduction);
        case SIMBOT -> new ModuleConstants(
            40.0,
            40.0,
            0.014,
            0.134,
            0.0,
            0.1,
            0.0,
            10.0,
            0.0,
            Mk4iReductions.L3.reduction,
            Mk4iReductions.TURN.reduction);
      };

  // Trajectory Following
  public static final TrajectoryConstants trajectoryConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> TrajectoryConstants.builder().linearkP(8.0).thetakP(4.0).build();
        case DEVBOT -> TrajectoryConstants.builder().linearkP(6.0).thetakP(8.0).build();
        case SIMBOT -> TrajectoryConstants.builder().linearkP(5.0).thetakP(8.0).build();
      };

  @Builder
  public record DriveConfig(
      double wheelRadius,
      double trackWidthX,
      double trackWidthY,
      double bumperWidthX,
      double bumperWidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    public double driveBaseRadius() {
      return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    }
  }

  public record ModuleConfig(
      int driveID,
      int turnID,
      int cancoderID,
      String canbusName,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted) {}

  public record ModuleConstants(
      double driveStallAmps,
      double turnStallAmps,
      double ffkS,
      double ffkV,
      double ffkT,
      double drivekP,
      double drivekD,
      double turnkP,
      double turnkD,
      double driveReduction,
      double turnReduction) {}

  @Builder
  public record TrajectoryConstants(
      double linearkP,
      double linearkD,
      double thetakP,
      double thetakD,
      double linearTolerance,
      double thetaTolerance,
      double goalLinearTolerance,
      double goalThetaTolerance,
      double linearVelocityTolerance,
      double angularVelocityTolerance) {}

  private enum Mk4iReductions {
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
