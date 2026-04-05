// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import frc.robot.z_constants.DialingConstants.Swerve;
import frc.robot.z_constants.SwerveConstants.AllModules.CurrentLimits;
import frc.robot.z_constants.SwerveConstants.AllModules.PhysicalConstants;

public class SwerveTemplate {

    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final CANcoder absEncoder;
    
    private final PIDController HeadingPidController;

    private final boolean absEncoderReversed;
    private final double absEncoderOffsetRad;

    @SuppressWarnings("removal")
    public SwerveTemplate(int driveMotorId, int steerMotorId, int absEncoderId,
        boolean driveMotorReversed, boolean steerMotorReversed, boolean absEncoderReversed,
        double absEncoderOffset) {

      driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        SparkMaxConfig configsDrive = new SparkMaxConfig();
        configsDrive.inverted(driveMotorReversed);
        configsDrive.smartCurrentLimit(
            CurrentLimits.kDriveStatorCurrentLimit,
            CurrentLimits.kDriveSupplyCurrentLimit);
        configsDrive.idleMode(IdleMode.kCoast);
        // Force the built-in encoder back to raw motor rotations / RPM,
        // in case a prior deploy left a persisted conversion factor on the
        // controller. Unit conversion lives in getDrivePosition/Velocity.
        configsDrive.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);
        // kResetSafeParameters wipes any persisted config from prior deploys
        // before applying ours; kPersistParameters saves our config to NVRAM
        // so it survives power cycles deterministically.
        driveMotor.configure(configsDrive,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);


      steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);
        SparkMaxConfig configsSteer = new SparkMaxConfig();
        configsSteer.inverted(steerMotorReversed);
        configsSteer.smartCurrentLimit(
            CurrentLimits.kSteerStatorCurrentLimit,
            CurrentLimits.kSteerSupplyCurrentLimit);
        configsSteer.idleMode(IdleMode.kBrake);
        // Same reasoning as the drive config above: force the NEO encoder
        // back to raw motor rotations so getSteerPosition()'s rot->rad
        // conversion is the only scaling in the signal path.
        configsSteer.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);
        steerMotor.configure(configsSteer,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        

        this.absEncoderOffsetRad = absEncoderOffset;
        this.absEncoderReversed = absEncoderReversed;
        absEncoder = new CANcoder(absEncoderId);

        HeadingPidController = new PIDController(
            Swerve.HeadingPID.kSteerP,
            Swerve.HeadingPID.kSteerI,
            Swerve.HeadingPID.kSteerD
        );
        HeadingPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        double DrivePosition = driveMotor.getEncoder().getPosition() * 
            PhysicalConstants.kDriveEncoderRot2Meter;
        return DrivePosition;
    }

    public double getSteerPosition() {
        double SteerPosition = steerMotor.getEncoder().getPosition() * 
            PhysicalConstants.kSteerEncoderRot2Rad;
        return SteerPosition;
    }

    public double getDriveVelocity() {
        double DriveVelocity = driveMotor.getEncoder().getVelocity() * 
            PhysicalConstants.kDriveEncoderRPM2MeterPerSec;
        return DriveVelocity;
    }

    public double getSteerVelocity() {
        double SteerVelocity = steerMotor.getEncoder().getVelocity() * 
            PhysicalConstants.kSteerEncoderRPM2RadPerSec;
        return SteerVelocity;
    }

    public double getabsEncoderRad() {
        // CANcoder getAbsolutePosition() is in rotations. Offsets stored in
        // DialingConstants are also in rotations. Subtract in rotation-space,
        // then convert to radians.
        double rotations = absEncoder.getAbsolutePosition().getValueAsDouble();
        rotations -= absEncoderOffsetRad;
        double radians = rotations * 2.0 * Math.PI;
        return radians * (absEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        // getabsEncoderRad() is in radians; NEO relative encoder's setPosition()
        // expects motor rotations, so divide by the rot->rad conversion factor.
        steerMotor.getEncoder().setPosition(
            getabsEncoderRad() / PhysicalConstants.kSteerEncoderRot2Rad);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) <
            PhysicalConstants.kMinSpeed) {
            stop();
            return;
        }

        // Offset is already baked into the NEO steer encoder via resetEncoders(),
        // so pass the desired angle through unmodified.
        // Optimize in the same reference frame as the PID measurement (getSteerPosition()).
        desiredState.optimize(new Rotation2d(getSteerPosition()));

        driveMotor.set(desiredState.speedMetersPerSecond /
            PhysicalConstants.kPhysicalMaxSpeed);
        steerMotor.set(HeadingPidController.calculate(getSteerPosition(),
            desiredState.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + driveMotor.getDeviceId() +
            "] state", desiredState.toString());
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerPosition()));
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}
