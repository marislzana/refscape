package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class Autos {

    private static final double SCORE_HAND_ANGLE = 135.0; 
   

    public static void registerNamedCommands(
            TankSubsystem tank,
            SimIntake intake,
            SimElevator elevator,
            SimHand hand,
            Supplier<Command> deposit,
            BooleanSupplier autoHandBusy) {

        for (String name : new String[] { "coleta", "coleta2"}) {
            register(name, () -> coletar(tank, intake, elevator));
        }

        for (String name : new String[] { "pontuar", "pontuar2"}) {
            register(name, () -> pontuar(tank, elevator, hand, deposit, autoHandBusy));
        }
    }

    private static void register(String name, Supplier<Command> supplier) {
        NamedCommands.registerCommand(name, Commands.defer(supplier, Set.of()));
    }
   
    private static Command coletar(TankSubsystem tank, SimIntake intake, SimElevator elevator) {
        return Commands.sequence(
                elevator.CMDsetPosition(400, 3),
                Commands.waitUntil(() -> SimElevator.getRealPosition() >= 200),
                intake.CMDintakeOn(),
                Commands.waitUntil(tank::isFuelInsideRobot).withTimeout(3.0),
                intake.CMDintakeOff())
                .onlyIf(() -> !tank.isHandCollected()); // regra: 1 alga por vez no braço
    }

   

    private static Command pontuar(
            TankSubsystem tank,
            SimElevator elevator,
            SimHand hand,
            Supplier<Command> deposit,
            BooleanSupplier autoHandBusy) {
        return Commands.sequence(
                
                Commands.waitUntil(tank::isHandCollected).withTimeout(6.0),
                Commands.waitUntil(() -> !autoHandBusy.getAsBoolean()).withTimeout(4.0),

            
                elevator.CMDsetPosition(400, 3),
                Commands.waitUntil(() -> SimElevator.getRealPosition() > 390),
                hand.CMDsetAngle(SCORE_HAND_ANGLE, 3),
                Commands.waitUntil(() -> Math.abs(SimHand.getRealAngle() - SCORE_HAND_ANGLE) < 5),

                deposit.get(),

                hand.CMDsetAngle(0, 3),
                Commands.waitUntil(() -> Math.abs(SimHand.getRealAngle()) < 5));
    }

    // AUTOS ANTIGOS 

    public static Command quadrado(TankSubsystem tank) {
        return Commands.sequence(
                tank.driveToPose(2.0, 2.0, 0),
                tank.driveToPose(4.0, 2.0, 90),
                tank.driveToPose(4.0, 5.0, 180),
                tank.driveToPose(2.0, 5.0, -90));
    }

    public static Command reto(TankSubsystem tank) {
        return Commands.sequence(
                tank.driveToPose(2.0, 2.0, 0),
                tank.driveToPose(5.0, 2.0, 0));
    }

    public static Command autoZigZag(TankSubsystem tank, SimIntake intake) {
        return Commands.sequence(
                tank.driveToPose(2.0, 2.0, 0),
                intake.CMDintakeOn(),
                tank.driveToPose(4.0, 3.0, 45),
                tank.driveToPose(6.0, 2.0, -45));
    }
}