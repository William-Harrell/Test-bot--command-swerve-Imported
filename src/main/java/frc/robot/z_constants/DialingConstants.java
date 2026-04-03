// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.z_constants;

/** Add your docs here. */
public class DialingConstants {

    public static final class Swerve {
        public static final double kFLabsEncoderOffsetRad = -0.254;
        public static final double kBLabsEncoderOffsetRad = -1.252;
        public static final double kFRabsEncoderOffsetRad = -1.816;
        public static final double kBRabsEncoderOffsetRad = -4.811;

        public static final class SteeringPIDController {
            public static final double kSteerP = 0.5;
            public static final double kSteerI = 0;
            public static final double kSteerD = 0;
        }

        public static final class MotorPID {
            // All Drive Motors
            public static final double kDriveP = 0.5;
            public static final double kDriveI = 0;
            public static final double kDriveD = 0;
            // All Steer Motors
            public static final double kSteerP = 0.5;
            public static final double kSteerI = 0;
            public static final double kSteerD = 0;
        }
    }
}
