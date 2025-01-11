// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Optional;

import org.opencv.photo.CalibrateCRF;

import com.fasterxml.jackson.databind.JsonSerializable.Base;
//import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;

public class NavXGyro extends AHRS{

    private static NavXGyro instance;
    public static double zeroHeading;
    public static double zeroAngle;

    /** Creates a new NavXGyro. */
    private NavXGyro() {
        //super(SPI.Port.kMXP);
        super(NavXComType.kMXP_SPI);  //ML Drycoded must test
        // super(NavXComType.kI2C);  //ML kMXP_SPI value is 0, kI2C value is 4, original code uses SPI.Port.kMXP which is 4
        reset();

        double angle = -90;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                angle = 90;
            }
        }
        zeroHeading = getNavHeading() + angle;
        zeroAngle = getNavAngle();
        System.out.println("Setup ZeroAngle " + zeroAngle);

    }

    // Public Methods
    public static NavXGyro getInstance() {
        if (instance == null) {
            instance = new NavXGyro();
        }
        return instance;
    }

    public double getNavHeading() {
        double heading = getFusedHeading();
        return heading;
    }

    public double getNavAngle() {
        double angle = getAngle();
        return angle;
    }

    public void zeroNavHeading() {
        //navX.zeroYaw();
        reset();

    }

    public double getZeroHeading(){
        return zeroHeading;
    }

    public double getZeroAngle(){
        return zeroAngle;
    }

    public Rotation2d getNavXRotation2D(){
        return Rotation2d.fromDegrees(getAngle());
    }

    public double getHeading() {
        double angle = 0;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                angle = 180;
            }
        }
        return Math.IEEEremainder(-getNavAngle(), 360) - angle;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetGyro(){
        reset();
    }
    
    /* 
    public void calibrateGyro(){ 
        calibrate();
    }
    */
}
