package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.VoltageConstants;
import frc.robot.commands.IntakeAndKickerCMD;
import frc.robot.commands.shootFromPodiumSCG;
import frc.robot.commands.shootFromSubSCG;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeAndKickerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class ThreeNoteAuto extends SequentialCommandGroup {
    public ThreeNoteAuto(Swerve s_Swerve, ShooterSubsystem shooterSubsystem, KickerSubsystem kickerSubsystem,
            ArmSubsystem armSubsystem, IntakeAndKickerSubsystem intakeAndKickerSubsystem) {
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-1.0, -.5), new Translation2d(-2.0, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-3.0, 0, new Rotation2d(0)),
                config);

        Trajectory secondTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(-3.0, -.5, new Rotation2d(1.570796)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(.5, 0), new Translation2d(0, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(1, 0, new Rotation2d(0)), // 1.570796 radian = 90 degrees
                config);

        Trajectory thirdTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(1, 0, new Rotation2d(1.570796)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0, 0), new Translation2d(0, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 0, new Rotation2d(0.7853982)),
                config);

        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand swerveControllerCommandTwo = new SwerveControllerCommand(
                secondTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand swerveControllerCommandThree = new SwerveControllerCommand(
                thirdTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
                new shootFromSubSCG(shooterSubsystem, armSubsystem, kickerSubsystem), //

                new InstantCommand(() -> s_Swerve.setPose(exampleTrajectory.getInitialPose())),
                swerveControllerCommand.alongWith(new IntakeAndKickerCMD(intakeAndKickerSubsystem,
                        VoltageConstants.vk_KickerForward, VoltageConstants.vk_IntakeForward, kickerSubsystem)), //

                new shootFromPodiumSCG(armSubsystem, shooterSubsystem, kickerSubsystem), //

                new InstantCommand(() -> s_Swerve.setPose(secondTrajectory.getInitialPose())),
                swerveControllerCommandTwo.alongWith(new IntakeAndKickerCMD(intakeAndKickerSubsystem,
                        VoltageConstants.vk_KickerForward, VoltageConstants.vk_IntakeForward, kickerSubsystem)), //

                new InstantCommand(() -> s_Swerve.setPose(thirdTrajectory.getInitialPose())),
                swerveControllerCommandThree, //
                new shootFromPodiumSCG(armSubsystem, shooterSubsystem, kickerSubsystem)//

        );
    }
}