// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.VisionSubsystem;

/** Not sure if you can do this using built-in WPILib stuff. */
public class SimulateRobotPosition extends SubsystemBase {
    private static class SimulateRobotPositionHolder {
        private static final SimulateRobotPosition INSTANCE = new SimulateRobotPosition();
    }
    
    public static SimulateRobotPosition getInstance() {
        return SimulateRobotPositionHolder.INSTANCE;
    }

    private SimulateRobotPosition() {
        super("SimulateRobotPosition");

        // Publish values so they show up in NetworkTables
        SmartDashboard.putNumber("X Position", 0);
        SmartDashboard.putNumber("Y Position", 0);
        SmartDashboard.putNumber("T Rotation", 0);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        SmartDashboard.putNumberArray(
            "SimulatedRobotPose",
            VisionSubsystem.pose2dToArray(getPoseFromNetworkTables())
        );
    }

    public Pose2d getPoseFromNetworkTables() {
        double x = SmartDashboard.getNumber("X Position", 0);
        double y = SmartDashboard.getNumber("Y Position", 0);
        double t = SmartDashboard.getNumber("T Rotation", 0);

        return new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(t));
    }
}
