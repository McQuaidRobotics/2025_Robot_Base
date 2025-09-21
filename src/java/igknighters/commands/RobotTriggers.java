package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotTriggers {
  public static final Trigger DISABLED = RobotModeTriggers.disabled();
  public static final Trigger AUTONOMOUS = RobotModeTriggers.autonomous();
  public static final Trigger TELEOP =
      RobotModeTriggers.teleop().onChange(Commands.print("SWAPING TO TELEOP"));
  public static final Trigger TEST = RobotModeTriggers.test();
}
