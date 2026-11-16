package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;
import frc.robot.subsystems.TankSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final TankSubsystem tank = new TankSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController driver = new CommandXboxController(0);
  private final XboxController driverget = new XboxController(0);


  private SimIntake mSimIntake = new SimIntake(20, 1.5, 3);
  private SimElevator mSimElevator = new SimElevator(400, 1.5);
  private SimHand mSimHand = new SimHand(-180, 180, 1.5, 3);
  
  private final SendableChooser<Command> autChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  autChooser.setDefaultOption("Nenhum", Commands.none());
  autChooser.addOption("Quadrado", Autos.Quadrado(tank));
  autChooser.addOption("Reto", Autos.reto(tank));
  autChooser.addOption("autoZigZag", Autos.autoZigZag(tank));

    // Configure the trigger bindings
    configureBindings();
    tank.setDefaultCommand(tank.run(() -> {
      double leftX = driver.getLeftX();
      double leftY = driver.getLeftY();

      double joysticksMagnitude = Math.hypot(leftX, leftY);

      if (joysticksMagnitude > 0.08) {
        tank.driveWithAngleControl(getLeftJoystickAngle(), driver.getRawAxis(3));

      }else{ 
        double forward = driver.getRawAxis(3);
        double backward = driver.getRawAxis(2);
        double rotation = driver.getLeftX();

        tank.drive(forward, -backward, -rotation);
      }
    }));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    driver.start().onTrue(tank.runOnce(() -> tank.resetPose(new Pose2d(2.0, 4.0, new Rotation2d()))));

    driver.povUp().onTrue(mSimElevator.CMDsetPosition(400, 10));
    driver.povDown().onTrue(mSimElevator.CMDsetPosition(0, 3));

    driver.povLeft().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-45, 3)));
    driver.povRight().onTrue(Commands.runOnce(() -> mSimHand.setAngle(45, 3)));
    driver.a().onTrue(Commands.runOnce(() -> mSimHand.setAngle(135, 3)));
    driver.b().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-135, 3)));

    driver.back().onTrue(Commands.runOnce(() -> mSimHand.setAngle(0, 3)));

    coletar_Ground();
    autoHand();
    driver.y().onTrue(deposit());

  }

  void coletar_Ground(){
    driver.leftBumper().whileTrue(mSimIntake.CMDintakeOn().onlyIf(() -> !tank.isFuelInsideRobot() && SimElevator.getRealPosition() >= 200));
  driver.leftBumper().whileFalse(mSimIntake.CMDintakeOff());

  }

  void autoHand(){
    BooleanSupplier autoHandOk = () -> tank.isFuelInsideRobot() && !tank.isHandCollected();

    onCommandCondition(autoHandOk, Commands.sequence(
      mSimElevator.CMDsetPosition(400, 3),
      Commands.waitUntil(() -> SimElevator.getRealPosition() > 390),
      mSimHand.CMDsetAngle(0, 3),
      Commands.waitUntil(() -> Math.abs(SimHand.getRealAngle()) < 5),
      mSimElevator.CMDsetPosition(0, 3),
      mSimHand.CMDsetVelocity(-1),
      Commands.waitSeconds(1),
      mSimHand.CMDsetVelocity(0)));   
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autChooser.getSelected();
  }

  public double getLeftJoystickAngle(){
    double X = -driverget.getLeftX();
    double Y = -driverget.getLeftY();

  if (Math.hypot(X, Y) < 0.1) {
    return 0.0;
  }

  return Math.toDegrees(Math.atan2(Y, X));
  
  }

   private void onCommandCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).onTrue(command);
  }

  private void whileCommandCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).whileTrue(command);
  }

  Command deposit() {
  return Commands.sequence(
    Commands.runOnce(() -> mSimHand.setVelocity(1)),
    tank.depositFuelCommand(),
    Commands.waitSeconds(1),
    Commands.runOnce(() -> mSimHand.setVelocity(0)));
  }


}
