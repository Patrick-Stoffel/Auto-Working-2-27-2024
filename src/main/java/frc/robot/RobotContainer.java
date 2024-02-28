package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.ShootOnlyAuto;
import frc.robot.autos.ThreeNoteAuto;
import frc.robot.autos.TwoNoteAuto;
//import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operatorJoystick = new Joystick(1);
    // private final CommandXboxController driver = new CommandXboxController(0);

    // Subsystems
    // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final KickerSubsystem kickerSubsystem = new KickerSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeAndKickerSubsystem intakeAndKickerSubsystem = new IntakeAndKickerSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private final Swerve s_Swerve = new Swerve();

    // Auto names
    private final Command ShootOnlyAuto = new ShootOnlyAuto(shooterSubsystem, kickerSubsystem, armSubsystem);
    private final Command ShootAndMoveAuto = new frc.robot.autos.ShootAndMoveAuto(s_Swerve, shooterSubsystem,
            kickerSubsystem, armSubsystem);

    private final Command TwoNoteAuto = new TwoNoteAuto(s_Swerve, shooterSubsystem, kickerSubsystem, armSubsystem,
            intakeAndKickerSubsystem);
    private final Command ThreeNoteAuto = new ThreeNoteAuto(s_Swerve, shooterSubsystem, kickerSubsystem, armSubsystem,
            intakeAndKickerSubsystem);
    // private final Command FourNoteAuto = new FourNoteAuto(s_Swerve,
    // shooterSubsystem, kickerSubsystem, armSubsystem, intakeAndKickerSubsystem);

    // add auto chooser
    SendableChooser<Command> m_Chooser = new SendableChooser<>();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        // NamedCommands.registerCommand("marker 1", Commands.print("Passed marker 1"));
        // NamedCommands.registerCommand("marker 2", Commands.print("Passed marker 2"));

        // NamedCommands.registerCommand("shootFromSub", new
        // shootFromSubParallel(shooterSubsystem, armSubsystem, kickerSubsystem));

        m_Chooser.setDefaultOption("Shoot Only Auto", ShootOnlyAuto);

        m_Chooser.addOption("Shoot and Move Auto", ShootAndMoveAuto);
        m_Chooser.addOption("Two Note Auto", TwoNoteAuto);
        m_Chooser.addOption("Three Note Auto", ThreeNoteAuto);
        // m_Chooser.addOption("Four Note Auto", FourNoteAuto);

        // Shuffleboard.getTab("Auto").add(m_Chooser);

        SmartDashboard.putData("Auto's", m_Chooser);

        // Configure the button bindings
        configureButtonBindings();

        // autoChooser = AutoBuilder.buildAutoChooser(); Teteza's Pathplanner lines
        // SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        new JoystickButton(driver, 7).whileTrue(new IntakeCMD(intakeSubsystem, VoltageConstants.vk_IntakeReverse));

        // driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        new JoystickButton(operatorJoystick, 1).onTrue(new IntakeAndKickerCMD(intakeAndKickerSubsystem,
                VoltageConstants.vk_IntakeForward, VoltageConstants.vk_KickerForward, kickerSubsystem));

        new JoystickButton(operatorJoystick, 2)
                .onTrue(new shootFromPodiumSCG(armSubsystem, shooterSubsystem, kickerSubsystem));

        new JoystickButton(operatorJoystick, 8)
                .onTrue(new shootFromSubSCG(shooterSubsystem, armSubsystem, kickerSubsystem));

        new JoystickButton(operatorJoystick, 4)
                .onTrue(new shootAmpSCG(armSubsystem, shooterSubsystem, kickerSubsystem));

        new JoystickButton(operatorJoystick, 7)
                .onTrue(new shootFromBackPodiumSCG(armSubsystem, shooterSubsystem, kickerSubsystem));

        new JoystickButton(operatorJoystick, 11).whileTrue(new MoveArmCMD(armSubsystem, VoltageConstants.vk_ArmUp));
        new JoystickButton(operatorJoystick, 12).whileTrue(new MoveArmCMD(armSubsystem, VoltageConstants.vk_ArmDown));

        new JoystickButton(operatorJoystick, 6)
                .whileTrue(new IntakeCMD(intakeSubsystem, VoltageConstants.vk_IntakeForward));

        new JoystickButton(operatorJoystick, 5)
                .whileTrue(new RunKickerCMD(kickerSubsystem, VoltageConstants.vk_KickerForward));

        new JoystickButton(operatorJoystick, 10)
                .whileTrue(new RunKickerCMD(kickerSubsystem, VoltageConstants.vk_KickerReverse));

        new JoystickButton(operatorJoystick, 9).whileTrue(new RunShooterCMD(shooterSubsystem,
                VoltageConstants.vk_TopShooterForward, VoltageConstants.vk_BottomShooterForward));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_Chooser.getSelected();
    }

}
