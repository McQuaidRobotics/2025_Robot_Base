package igknighters.subsystems;

// import monologue.Logged;

public abstract class Component {

  /** Should be called every cycle in the parent subsystems periodic method */
  public void periodic() {}
  ;

  public abstract String getOverrideName();
}
