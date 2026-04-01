// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveTemplate extends SubsystemBase {

    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final CANcoder absEncoder;
    

    private final PIDController steerPidController;

    private final boolean absEncoderReversed;
    private final double absEncoderOffsetRad;

    public SwerveTemplate(int driveMotorId, int steerMotorId, int absEncoderId,
        boolean driveMotorReversed, boolean steerMotorReversed, boolean absEncoderReversed,
        double absEncoderOffset) {

      driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
      steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);

        

        this.absEncoderOffsetRad = absEncoderOffset;
        this.absEncoderReversed = absEncoderReversed;


        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        steerEncoder.setPositionConversionFactor(ModuleConstants.ksteerEncoderRot2Rad);
        steerEncoder.setVelocityConversionFactor(ModuleConstants.ksteerEncoderRPM2RadPerSec);

        steerPidController = new PIDController(ModuleConstants.kPsteer, 0, 0);
        steerPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getsteerPosition() {
        return steerEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getsteerVelocity() {
        return steerEncoder.getVelocity();
    }

    public double getabsEncoderRad() {
        double angle = absEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absEncoderOffsetRad;
        return angle * (absEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        steerEncoder.setPosition(getabsEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getsteerPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        steerMotor.set(steerPidController.calculate(getsteerPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}
