// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.z_constants;

/** Add your docs here. */
public class DialingConstants {

    public static final class Swerve {
        public static final double kFLabsEncoderOffsetRad = -0.198242;
        public static final double kBLabsEncoderOffsetRad = -0.242188;
        public static final double kFRabsEncoderOffsetRad = -0.210938;
        public static final double kBRabsEncoderOffsetRad = -0.224121;

        public static final class HeadingPID {
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
            public static final double kSteerP = 0.3;
            public static final double kSteerI = 0;
            public static final double kSteerD = 0;
        }
    }
}
