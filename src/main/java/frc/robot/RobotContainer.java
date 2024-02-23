// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.drive.GyroReset;
import frc.robot.commands.drive.SnapToHeadingCommand;
import frc.robot.commands.drive.TwistCommand;
import frc.robot.commands.drive.XCommand;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.transit.StartTransit;
import frc.robot.commands.transit.StopTransit;
import frc.robot.commands.camera.AutoAimCommand;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.commands.auto.TransitShootCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final TransitSubsystem transitSubsystem = new TransitSubsystem();
    public final LightSubsystem lightSubsystem = new LightSubsystem();
    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final TransitShootCommand transitShootCommand = new TransitShootCommand(shooterSubsystem, transitSubsystem);
    private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.CONTROLLER.DRIVE_JOYSTICK);
    private final Joystick operatorJoystick = new Joystick(RobotConstants.PortConstants.CONTROLLER.OPERATOR_JOYSTICK);

    SendableChooser<Command> m_autoPositionChooser = new SendableChooser<>();

    PowerDistribution PDP = new PowerDistribution(16, ModuleType.kRev);

    private final Field2d field = new Field2d();

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveSubsystem, driveJoystick));
        climberSubsystem.setDefaultCommand(new ClimberCommand(climberSubsystem, operatorJoystick));
        
        createNamedCommands();

        configureButtonBindings();

        m_autoPositionChooser = AutoBuilder.buildAutoChooser("lineAuto");
        Shuffleboard.getTab("Autonomous").add(m_autoPositionChooser);
        Shuffleboard.getTab("Power").add(PDP);
    }

    private void createNamedCommands() {
        NamedCommands.registerCommand("StartShooter", new StartShooter(shooterSubsystem));
        NamedCommands.registerCommand("StopShooter", new StopShooter(shooterSubsystem));
        NamedCommands.registerCommand("AutoShooter", new TransitShootCommand(shooterSubsystem, transitSubsystem));
        NamedCommands.registerCommand("StartIntake", new StartIntake(intakeSubsystem));
        NamedCommands.registerCommand("StopIntake", new StopIntake(intakeSubsystem));
        NamedCommands.registerCommand("StartTransit", new StartTransit(transitSubsystem));
        NamedCommands.registerCommand("StopTransit", new StopTransit(transitSubsystem));
        NamedCommands.registerCommand("AutoAimCommand", new AutoAimCommand(driveSubsystem, visionSubsystem));
        
    }

    private void configureButtonBindings(){
        new JoystickButton(driveJoystick, 1).whileTrue(new TwistCommand());
        new JoystickButton(driveJoystick,11).onTrue(new GyroReset(driveSubsystem));
        new JoystickButton(driveJoystick, 3).onTrue((new XCommand()));
        new JoystickButton(driveJoystick, 7).onTrue(new AutoAimCommand(driveSubsystem, visionSubsystem));
        
 
        ///////////////////     Up = DriveJoystick, Down = OperatorJoystick     /////////////////////////////////////////


        new JoystickButton(operatorJoystick, 3).onTrue((new StartIntake(intakeSubsystem))).onFalse(new StopIntake(intakeSubsystem));
        
        
        new JoystickButton(operatorJoystick, 1).onTrue((new StartShooter(shooterSubsystem))).onFalse(new StopShooter(shooterSubsystem));

        new JoystickButton(operatorJoystick, 2).onTrue(new TransitShootCommand(shooterSubsystem,transitSubsystem));


        new POVButton(driveJoystick, 0).whileTrue(new SnapToHeadingCommand(driveSubsystem, 0));
        new POVButton(driveJoystick, 90).whileTrue(new SnapToHeadingCommand(driveSubsystem, 90));
        new POVButton(driveJoystick, 180).whileTrue(new SnapToHeadingCommand(driveSubsystem, 180));
        new POVButton(driveJoystick, 270).whileTrue(new SnapToHeadingCommand(driveSubsystem, 270));
    }

    public Command getAutonomousCommand() {
        //return new LEDMatch(lightSubsystem, 2);
        return m_autoPositionChooser.getSelected();
    }

    public Field2d getField() {
        return field;
    }
    
    public final class UserPolicy {
        public static boolean twistable = false;
        public static boolean xLocked = false;
    }
}
