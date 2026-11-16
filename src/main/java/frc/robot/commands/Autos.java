// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.TankSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

//public static Command Quadrado(TankSubsystem tank) {
//    // TODO Auto-generated method stub
//    throw new UnsupportedOperationException("Unimplemented method 'Quadrado'");
//}
//
//public static Command reto(TankSubsystem tank) {
//    // TODO Auto-generated method stub
//    throw new UnsupportedOperationException("Unimplemented method 'reto'");
//}
//
//public static Command autoZigZag(TankSubsystem tank) {
//    // TODO Auto-generated method stub
//    throw new UnsupportedOperationException("Unimplemented method 'autoZigZag'");
//}

public static Command Quadrado(TankSubsystem tank) {
    double lado = 1.0; // metros

    return Commands.sequence(
        tank.CMDresetPose(new Pose2d()),
        tank.driveToPose(lado, 0.0, 90.0),
        tank.driveToPose(lado, lado, 180.0),
        tank.driveToPose(0.0, lado, -90.0),
        tank.driveToPose(0.0, 0.0, 0.0)
    );
}

public static Command reto(TankSubsystem tank) {
    return Commands.sequence(
        tank.CMDresetPose(new Pose2d()),
        tank.driveToPoint(2.0, 0.0)
    );
}

public static Command autoZigZag(TankSubsystem tank) {
    return Commands.sequence(
        tank.CMDresetPose(new Pose2d()),
        tank.driveToPoint(1.0, 0.5),
        tank.driveToPoint(2.0, -0.5),
        tank.driveToPoint(3.0, 0.5),
        tank.driveToPoint(4.0, 0.0)
    );
}
}
