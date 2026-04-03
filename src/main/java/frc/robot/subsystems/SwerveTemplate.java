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
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.z_constants.DialingConstants.Swerve;
import frc.robot.z_constants.SwerveConstants.AllModules.CurrentLimits;
import frc.robot.z_constants.SwerveConstants.AllModules.PhysicalConstants;

public class SwerveTemplate extends SubsystemBase {

    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final CANcoder absEncoder;
    

    private final PIDController steerPidController;

    private final boolean absEncoderReversed;
    private final double absEncoderOffsetRad;

    @SuppressWarnings("removal")
    public SwerveTemplate(int driveMotorId, int steerMotorId, int absEncoderId,
        boolean driveMotorReversed, boolean steerMotorReversed, boolean absEncoderReversed,
        double absEncoderOffset) {

      driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        SparkMaxConfig configsDrive = new SparkMaxConfig();
        configsDrive.closedLoop.pid(
            Swerve.MotorPID.kDriveP,
            Swerve.MotorPID.kDriveI, 
            Swerve.MotorPID.kDriveD);
        configsDrive.inverted(driveMotorReversed);
        configsDrive.smartCurrentLimit(
            CurrentLimits.kDriveStatorCurrentLimit, 
            CurrentLimits.kDriveSupplyCurrentLimit);
        configsDrive.idleMode(IdleMode.kCoast);
        driveMotor.configure(configsDrive, 
            ResetMode.kNoResetSafeParameters, 
            PersistMode.kNoPersistParameters);

      steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);
        SparkMaxConfig configsSteer = new SparkMaxConfig();
        configsSteer.closedLoop.pid(
            Swerve.MotorPID.kSteerP,
            Swerve.MotorPID.kSteerI, 
            Swerve.MotorPID.kSteerD);
        configsSteer.inverted(steerMotorReversed);
        configsSteer.smartCurrentLimit(
            CurrentLimits.kSteerStatorCurrentLimit, 
            CurrentLimits.kSteerSupplyCurrentLimit);
        configsSteer.idleMode(IdleMode.kBrake);
        steerMotor.configure(configsSteer, 
            ResetMode.kNoResetSafeParameters, 
            PersistMode.kNoPersistParameters);
        

        this.absEncoderOffsetRad = absEncoderOffset;
        this.absEncoderReversed = absEncoderReversed;
        absEncoder = new CANcoder(absEncoderId);

        steerPidController = new PIDController(
            Swerve.SteeringPIDController.kSteerP,
            Swerve.SteeringPIDController.kSteerI,
            Swerve.SteeringPIDController.kSteerD
        );
        steerPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        var DrivePosition = driveMotor.getEncoder().getPosition() * 
            PhysicalConstants.kDriveEncoderRot2Meter;
        return DrivePosition;
    }

    public double getSteerPosition() {
        var SteerPosition = steerMotor.getEncoder().getPosition() * 
            PhysicalConstants.kSteerEncoderRot2Rad;
        return SteerPosition;
    }

    public double getDriveVelocity() {
        var DriveVelocity = driveMotor.getEncoder().getVelocity() * 
            PhysicalConstants.kDriveEncoderRPM2MeterPerSec;
        return DriveVelocity;
    }

    public double getSteerVelocity() {
        var SteerVelocity = steerMotor.getEncoder().getVelocity() * 
            PhysicalConstants.kSteerEncoderRPM2RadPerSec;
        return SteerVelocity;
    }

    public double getabsEncoderRad() {
        double angle = absEncoder.getAbsolutePosition().getValueAsDouble();
        angle *= 2.0 * Math.PI;
        angle -= absEncoderOffsetRad;
        return angle * (absEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        steerMotor.getEncoder().setPosition(getabsEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    // TODO: Fix below in the command/Module area
    public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 
        PhysicalConstants.kMinSpeed) {
      stop();
      return;
    }
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(absEncoderOffsetRad));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(absEncoder.getPosition().getValueAsDouble()));

    driveMotor.set(desiredState.speedMetersPerSecond / 
        PhysicalConstants.kPhysicalMaxSpeed);
    steerMotor.set(steerPidController.calculate(getSteerPosition(), 
        desiredState.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absEncoder.getDeviceID() + 
            "] state", desiredState.toString());
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerPosition()));
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // might have to put signal requester for the falcons here
  }
}
