// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.ColorSensorV3.Register;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class RobotContainer {

  private final TankSubsystem tank = new TankSubsystem();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final XboxController driverget = new XboxController(0);

  private SimIntake mSimIntake = new SimIntake(20, 1.5, 3);
  private SimElevator mSimElevator = new SimElevator(400, 1.5);
  private SimHand mSimHand = new SimHand(-180, 180, 1.5, 3);

//autonomo algas
  private static final Pose2d START_POSE = new Pose2d(7.118, 1.402, Rotation2d.fromDegrees(180));

  private static final double Aproach_Distance = 0.72;
  private static final double Colect_Heading = 180.0;
  private static final double Score_HandAngle = 135.0;

  private static final double ViaX = 3.0;
  private static final double ViaY = 2.2;


  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    tank.configureAutoBuilder();

    Autos.registerNamedCommands(
        tank, mSimIntake, mSimElevator, mSimHand,
        this::deposit,
        () -> autoHandCommand != null && autoHandCommand.isScheduled());

    autoChooser.setDefaultOption("Nenhum", Commands.none());
    autoChooser.addOption("Quadrado", Autos.quadrado(tank));
    autoChooser.addOption("Reto", Autos.reto(tank));
    autoChooser.addOption("autoZigZag", Autos.autoZigZag(tank, mSimIntake));
    autoChooser.addOption("Ciclo Auto", AutoBuilder.buildAuto("Ciclo Auto"));
    // autoChooser.addOption("Algae_3", Autos.Algae_3(tank));

    SmartDashboard.putData("Auto Selector", autoChooser);

    configureBindings();

    tank.setDefaultCommand(tank.run(() -> {

      
      double leftX = driver.getLeftX();
      double leftY = driver.getLeftY();

      double joystickMagnitude = Math.hypot(leftX, leftY);

      // Joystick esquerdo sendo utilizado
      if (joystickMagnitude > 0.08) {

        tank.driveWithAngleControl(getLeftJoystickAngle(), driver.getRawAxis(3));

        // Tank normal
      } else {

        double forward = driver.getRawAxis(3);
        double backward = driver.getRawAxis(2);
        double rotation = driver.getRightX();

        tank.drive(forward, -backward, -rotation);
      }
    }));
  }

  private void configureBindings() {



    driver.start().onTrue(tank.runOnce(() -> tank.resetPose(new Pose2d(2.0, 4.0, new Rotation2d()))));

    driver.povUp().onTrue(mSimElevator.CMDsetPosition(400, 10));
    driver.povDown().onTrue(mSimElevator.CMDsetPosition(0, 3));

    driver.povLeft().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-45, 3)));
    driver.povRight().onTrue(Commands.runOnce(() -> mSimHand.setAngle(45, 3)));
    driver.a().onTrue(Commands.runOnce(() -> mSimHand.setAngle(135, 3)));
    driver.b().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-135, 3)));

    driver.back().onTrue(Commands.runOnce(() -> mSimHand.setAngle(0, 3)));

    /*
     * Positivo = Depositar
     * Negativo = Coletar
     */
    coletar_Ground();
    autoHand();
    // deposit_Pose();

    driver.y().onTrue(deposit());

  }

  void coletar_Ground() {
    driver.leftBumper().whileTrue(
      mSimIntake.CMDintakeOn()
    );
    driver.leftBumper().whileFalse(mSimIntake.CMDintakeOff());
  }

  void autoHand() {

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








  private Command autoHandCommand;

  public Command Alga_3(){
    return Commands.sequence( tank.CMDresetPose(START_POSE), algaCycle(1.223, 2.200, false),
    algaCycle(1.223, 4.023, true),
    algaCycle(1.223, 5.858, true),

    mSimElevator.CMDsetPosition(0, 3));
  }




  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public double getLeftJoystickAngle() {

    double x = -driverget.getLeftX();
    double y = -driverget.getLeftY();

    if (Math.hypot(x, y) < 0.1) {
      return 0.0;
    }

    return Math.toDegrees(Math.atan2(x, y));
  }

  /**
   * Ativa o comando alvo, quando a condição é verdadeira.
   * 
   * @param null
   */
  private void onCommandCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).onTrue(command);
  }

  /**
   * Matém um comando ativo, enquanto a condição é verdade.
   * 
   * @param null
   */
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

  private Command algaCycle(double gpX, double gpY, boolean aroundReef){
    return Commands.sequence(collectAlga(gpX, gpY, aroundReef).onlyIf(() -> ! tank.isHandCollected()),
     returnToScore(aroundReef), scoreAlga());
  }
private Command collectAlga(double gpX, double gpY, boolean aroundReef) {
    return Commands.sequence(
        // Mesma condição do teleop: intake só com elevador >= 200
        mSimElevator.CMDsetPosition(400, 3),

        // Dirige até a alga com o intake abrindo e ligando junto
        Commands.deadline(
            Commands.sequence(
                aroundReef ? tank.driveToPoint(ViaX, ViaY, 0.4) : Commands.none(),
                tank.driveToPose(gpX + Aproach_Distance, gpY, Colect_Heading)),
            mSimIntake.CMDintakeOn()),

        // Espera a alga entrar no robô e recolhe o intake
        Commands.waitUntil(tank::isFuelInsideRobot).withTimeout(2.0),
        mSimIntake.CMDintakeOff());
  }

  private Command returnToScore(boolean aroundReef) {
    return Commands.sequence(
        aroundReef ? tank.driveToPoint(ViaX, ViaY, 0.4) : Commands.none(),
        tank.driveToPose(START_POSE.getX(), START_POSE.getY(), START_POSE.getRotation().getDegrees()));
  }

  private Command scoreAlga() {
    return Commands.sequence(
        // O autoHand roda em paralelo durante a volta; espera ele terminar
        Commands.waitUntil(tank::isHandCollected).withTimeout(6.0),
        Commands.waitUntil(() -> !autoHandCommand.isScheduled()).withTimeout(4.0),

        // Levanta o elevador e estica a mão
        mSimElevator.CMDsetPosition(400, 3),
        Commands.waitUntil(() -> SimElevator.getRealPosition() > 390),
        mSimHand.CMDsetAngle(Score_HandAngle, 3),
        Commands.waitUntil(() -> Math.abs(SimHand.getRealAngle() - Score_HandAngle) < 5),

        // Atira
        deposit(),

        // Recolhe a mão (o elevador fica em cima para a próxima coleta)
        mSimHand.CMDsetAngle(0, 3),
        Commands.waitUntil(() -> Math.abs(SimHand.getRealAngle()) < 5));
  }
}