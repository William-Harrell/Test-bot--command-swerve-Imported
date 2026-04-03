// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.z_constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class SwerveConstants {

    public static final class IndividualModules {

        public static final boolean kFLDriveEncoderReversed = true;
        public static final boolean kFLSteerEncoderReversed = true;
        public static final boolean kFLabsEncoderReversed = false;

        public static final boolean kFRDriveEncoderReversed = false;
        public static final boolean kFRSteerEncoderReversed = true;
        public static final boolean kFRabsEncoderReversed = false;

        public static final boolean kBLDriveEncoderReversed = true;
        public static final boolean kBLSteerEncoderReversed = true;
        public static final boolean kBLabsEncoderReversed = false;

        public static final boolean kBRSteerEncoderReversed = true;
        public static final boolean kBRDriveEncoderReversed = false;
        public static final boolean kBRabsEncoderReversed = false;
    }

    public static final class AllModules {


        public static final class PhysicalConstants {
                // Minimum speed to move in m/s
            public static final double kMinSpeed = 0.001;
                // Distance between right and left wheels
            public static final double kTrackWidth = Units.inchesToMeters(21);
                // Distance between front and back wheels
            public static final double kWheelBase = Units.inchesToMeters(25.5);
                // literal max speed in m/s
            public static final double kPhysicalMaxSpeed = 5; 
                // literal max turning speed in rads/s 
            public static final double kPhysicalMaxAngularSpeed = 2 * 2 * Math.PI; 

            public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
            public static final double kDriveMotorGearRatio = 1 / 5.8462;
            public static final double kSteerMotorGearRatio = 1 / 18.0;

            public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
            public static final double kSteerEncoderRot2Rad = kSteerMotorGearRatio * 2 * Math.PI;
            public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
            public static final double kSteerEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60;
            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
        }

        public static final class TeleopDriving {
            public static final double kTeleDriveMaxSpeedMetersPerSecond = 
                PhysicalConstants.kPhysicalMaxSpeed / 4;
            public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                PhysicalConstants.kPhysicalMaxAngularSpeed / 4;
            public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
            public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        }

        public static final class AutoDriving {
            public static final double kMaxSpeedMetersPerSecond = 
                PhysicalConstants.kPhysicalMaxSpeed / 4;
            public static final double kMaxAngularSpeedRadiansPerSecond =
                PhysicalConstants.kPhysicalMaxAngularSpeed / 10;
            public static final double kMaxAccelerationMetersPerSecondSquared = 3;
            public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
            public static final double kPXController = 1.5;
            public static final double kPYController = 1.5;
            public static final double kPThetaController = 3;

            public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularAccelerationRadiansPerSecondSquared);
        }

        public static final class CurrentLimits {
                // All Drive motors
            public static final int kDriveStatorCurrentLimit = 80;
            public static final int kDriveSupplyCurrentLimit = 60;
                // All Steer motors
            public static final int kSteerStatorCurrentLimit = 50;
            public static final int kSteerSupplyCurrentLimit = 40;
        }
    }


}