package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI.UserPolicy;

public class TwistCommand extends Command {
    public TwistCommand() {

    }

    @Override
    public void initialize() {
        UserPolicy.twistable = true;
    }

    @Override
    public void end(boolean interrupted) {
        UserPolicy.twistable = false;
    }
}
