// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.z_constants.SwerveConstants.AllModules.PhysicalConstants;
import frc.robot.z_constants.CanIDs;
import frc.robot.z_constants.SwerveConstants.IndividualModules;
import frc.robot.z_constants.DialingConstants.Swerve;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveTemplate frontLeft = new SwerveTemplate(
            CanIDs.kFLDriveMotorPort,
            CanIDs.kFLSteerMotorPort,
            CanIDs.kFLabsEncoderPort,
            IndividualModules.kFLDriveEncoderReversed,
            IndividualModules.kFLSteerEncoderReversed,
            IndividualModules.kFLabsEncoderReversed,            
            Swerve.kFLabsEncoderOffsetRad);

    private final SwerveTemplate frontRight = new SwerveTemplate(
            CanIDs.kFRDriveMotorPort,
            CanIDs.kFRSteerMotorPort,
            CanIDs.kFRabsEncoderPort,
            IndividualModules.kFRDriveEncoderReversed,
            IndividualModules.kFRSteerEncoderReversed,
            IndividualModules.kFRabsEncoderReversed,
            Swerve.kFRabsEncoderOffsetRad);

    private final SwerveTemplate backLeft = new SwerveTemplate(
            CanIDs.kBLDriveMotorPort,
            CanIDs.kBLSteerMotorPort,
            CanIDs.kBLabsEncoderPort,
            IndividualModules.kBLDriveEncoderReversed,
            IndividualModules.kBLSteerEncoderReversed,
            IndividualModules.kBLabsEncoderReversed,
            Swerve.kBLabsEncoderOffsetRad);

    private final SwerveTemplate backRight = new SwerveTemplate(
            CanIDs.kBRDriveMotorPort,
            CanIDs.kBRSteerMotorPort,
            CanIDs.kBRabsEncoderPort,
            IndividualModules.kBRDriveEncoderReversed,
            IndividualModules.kBRSteerEncoderReversed,
            IndividualModules.kBRabsEncoderReversed,
            Swerve.kBRabsEncoderOffsetRad);

    private final Pigeon2 gyro = new Pigeon2(CanIDs.kPigeon2Port);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        PhysicalConstants.kDriveKinematics,
        getRotation2d(),
        new SwerveModulePosition[]{
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            backRight.getSwerveModulePosition()
        },
        new Pose2d(0,0, new Rotation2d(0))
        // TODO: change to actual starting pose when doing AUTO stuff
        );

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
          frontLeft.getSwerveModulePosition(),   
          frontRight.getSwerveModulePosition(), 
          backLeft.getSwerveModulePosition(),
          backRight.getSwerveModulePosition()}, 
          pose);;
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), new SwerveModulePosition[] {
          frontLeft.getSwerveModulePosition(),   
          frontRight.getSwerveModulePosition(), 
          backLeft.getSwerveModulePosition(),
          backRight.getSwerveModulePosition()});
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        SmartDashboard.putNumber("FLDriveMotor", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("FLSteerMotor", frontLeft.getSteerVelocity());

        SmartDashboard.putNumber("FRDriveMotor", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("FRSteerMotor", frontRight.getSteerVelocity());

        SmartDashboard.putNumber("BLDriveMotor", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("BLSteerMotor", backLeft.getSteerVelocity());
        
        SmartDashboard.putNumber("BRDriveMotor", backRight.getDriveVelocity());
        SmartDashboard.putNumber("BRSteerMotor", backRight.getSteerVelocity());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 
            PhysicalConstants.kPhysicalMaxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
