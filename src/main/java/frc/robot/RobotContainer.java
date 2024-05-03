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
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.drive.TwistCommand;
import frc.robot.commands.drive.XCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.automatedCommands.RobotSystemsCheckCommand;
import frc.robot.commands.drive.GyroReset;

public class RobotContainer {
        
    public final DriveSubsystem driveSubsystem = new DriveSubsystem();

    private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.Controller.DRIVE_JOYSTICK);
    private final Joystick operatorJoystick = new Joystick(RobotConstants.PortConstants.Controller.OPERATOR_JOYSTICK);
    //Define joysticks on specified ports. Only for usb input type controllers, use another class for xbox.

    SendableChooser<Command> m_autoPositionChooser = new SendableChooser<>();

    PowerDistribution pdp;

    private final Field2d field = new Field2d();

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveSubsystem, driveJoystick));
        
        createNamedCommands();

        configureButtonBindings();

        try {
            pdp = new PowerDistribution(16, ModuleType.kRev);
            m_autoPositionChooser = AutoBuilder.buildAutoChooser("");
            Shuffleboard.getTab("Autonomous Selection").add(m_autoPositionChooser);
            Shuffleboard.getTab("Power").add(pdp);
        }
        catch (Exception e){
            e.printStackTrace();
        }
    }

    private void createNamedCommands() {
        //Add commands here to be able to execute in auto through pathplanner
    }

    private void configureButtonBindings(){
        new JoystickButton(driveJoystick, 1).whileTrue(new TwistCommand());
        new JoystickButton(driveJoystick,11).onTrue(new GyroReset(driveSubsystem));
        new JoystickButton(driveJoystick, 3).onTrue((new XCommand()));

        ///////////////////     Above = DriveJoystick, Below = OperatorJoystick     /////////////////////////////////////////


    }

    public Command getAutonomousCommand() {
        if (m_autoPositionChooser.getSelected() != null){
            return m_autoPositionChooser.getSelected();
        }
        else {
            return new GyroReset(driveSubsystem);
        }
    }

    public Command getTestingCommand(){
        return new RobotSystemsCheckCommand(driveSubsystem);
    }

    public Field2d getField() {
        return field;
    }
    public final class UserPolicy {
        public static boolean twistable = false;
        public static boolean xLocked = false;
        //You can put booleans that change throughout the code here
    }
}
