// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.List;

import javax.security.auth.x500.X500Principal;

import com.fasterxml.jackson.databind.deser.std.NumberDeserializers.DoubleDeserializer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;


/** Add your docs here. */
public class ScorpTrajectory3 {
  private DriveSubsystem m_drive;
private ScorpTrajectory3 m_scorp;
  public Trajectory path = new Trajectory();
  public Trajectory zerotrajectory = new Trajectory();




  public ScorpTrajectory3(DriveSubsystem m_drive,ScorpTrajectory3 m_scorp) {

    this.m_drive = m_drive;
this.m_scorp = m_scorp;
     var autoVoltageConstraint =
         new DifferentialDriveVoltageConstraint(
             new SimpleMotorFeedforward(
                 Constants.ODOMETRY.kS, Constants.ODOMETRY.kV, Constants.ODOMETRY.kA),
             Constants.ODOMETRY.kinematics,
            7);


             
     TrajectoryConfig config =
         new TrajectoryConfig(
                 Constants.ODOMETRY.kMaxSpeedMetersPerSecond,
                Constants.ODOMETRY.kMaxAccelerationMetersPerSecondSquared)
             .setKinematics(Constants.ODOMETRY.kinematics)
             .addConstraint(autoVoltageConstraint);


     TrajectoryConfig configBackward =
         new TrajectoryConfig(
                 Constants.ODOMETRY.kMaxSpeedMetersPerSecond,
                 Constants.ODOMETRY.kMaxAccelerationMetersPerSecondSquared)
             .setKinematics(Constants.ODOMETRY.kinematics)
             .addConstraint(autoVoltageConstraint);
     configBackward.setReversed(true);


    


//path listesi




/*path = TrajectoryGenerator.generateTrajectory(
    List.of(
      new Pose2d(0.74, 4.775, new Rotation2d(-2.9671)), 
      new Pose2d(4, 4, new Rotation2d(2.4435)),
      new Pose2d(6.57, 2.58, new Rotation2d(-2.1293))),
      
     config

   );
*/

 
    path = TrajectoryGenerator.generateTrajectory(
       // start at the origin facing the +X direction
       new Pose2d(0, 0, new Rotation2d(0)),
       // Pass through these two interior waypoints, making an 's' curve path
       List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
       // End 3 meters straight ahead of where we started, facing forward
       new Pose2d(3, 0, new Rotation2d(0)),
       // Pass config
       config);


//s çizmesi lazım


/*
 

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);



 */




}
public RamseteCommand ramseteGenerator(){
//Trajectory trajectory

return new RamseteCommand(
path,
 m_drive::getPose,
 new RamseteController(Constants.ODOMETRY.kRamseteB, Constants.ODOMETRY.kRamseteZeta),
 new SimpleMotorFeedforward(Constants.ODOMETRY.kS, Constants.ODOMETRY.kV, Constants.ODOMETRY.kA),
 Constants.ODOMETRY.kinematics,
 m_drive::getWheelSpeeds, 
 new PIDController(Constants.ODOMETRY.kP, 0, 0), 
 new PIDController(Constants.ODOMETRY.kP, 0, 0), 
 m_drive::tankDriveVolts,
 m_drive);

}

}
