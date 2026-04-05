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
            //TODO: Might have to fix these
        public static final boolean kFLDriveEncoderReversed = false;
        public static final boolean kFLSteerEncoderReversed = false;
        // abs flag flipped true: CANcoder counts opposite to NEO hardware
        // direction on this chassis; negating getabsEncoderRad() puts both
        // sensors in the same sign convention so the PID measurement and
        // the abs seed agree after motion, not just at boot.
        public static final boolean kFLabsEncoderReversed = true;

        public static final boolean kFRDriveEncoderReversed = false;
        public static final boolean kFRSteerEncoderReversed = false;
        public static final boolean kFRabsEncoderReversed = true;

        public static final boolean kBLDriveEncoderReversed = false;
        public static final boolean kBLSteerEncoderReversed = false;
        public static final boolean kBLabsEncoderReversed = true;

        public static final boolean kBRDriveEncoderReversed = false;
        public static final boolean kBRSteerEncoderReversed = false;
        public static final boolean kBRabsEncoderReversed = true;
    }

    public static final class AllModules {


        public static final class PhysicalConstants {
                // Minimum speed to move in m/s
            public static final double kMinSpeed = 0.01;
                // Distance between right and left wheels
            public static final double kTrackWidth = Units.inchesToMeters(19);
                // Distance between front and back wheels
            public static final double kWheelBase = Units.inchesToMeters(19);
                // literal max speed in m/s
            public static final double kPhysicalMaxSpeed = 5; 
                // literal max turning speed in rads/s 
            public static final double kPhysicalMaxAngularSpeed = 2 * 2 * Math.PI; // No idea

            public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
            public static final double kDriveMotorGearRatio = 6.75;
                private static final double SteerGearRatioNUM = 150;
                private static final double SteerGearRatioDOM = 7;
                private static final double SteerGearRatio = SteerGearRatioNUM / SteerGearRatioDOM;
            public static final double kSteerMotorGearRatio = SteerGearRatio;


            public static final double kDriveEncoderRot2Meter = Math.PI * kWheelDiameterMeters / kDriveMotorGearRatio;
            public static final double kSteerEncoderRot2Rad = 2 * Math.PI / kSteerMotorGearRatio;
            public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
            public static final double kSteerEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60;
            // Order must match SwerveSubsystem.setModuleStates: FL, FR, BL, BR
            // WPILib convention: +x forward, +y left
            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // FL
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // FR
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // BL
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));// BR
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
            public static final int kDriveStatorCurrentLimit = 100;
            public static final int kDriveSupplyCurrentLimit = 60;
                // All Steer motors
            public static final int kSteerStatorCurrentLimit = 50;
            public static final int kSteerSupplyCurrentLimit = 40;
        }
    }


}