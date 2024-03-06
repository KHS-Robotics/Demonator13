package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.arm.WaitForArmStow;
import frc.robot.commands.intake.SetIntakeState;
import frc.robot.commands.shooter.SetShooterState;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;

public class StateCommandGenerator {
    public static Command goToStowCommand() {
        if (RobotContainer.arm.getPosition() > ArmState.kShoot.rotations + 0.02) {
            // arm down
            // move arm then move intake
            return new ParallelCommandGroup(new SetShooterState(ShooterState.kShootFromSubwooferAuto), new SequentialCommandGroup(new SetArmState(ArmState.kStow), new SetIntakeState(IntakeState.kUp)));
        } else {
            // arm up
            // move arm and intake together
            return new ParallelCommandGroup(new SetShooterState(ShooterState.kShootFromSubwooferAuto), new SetArmState(ArmState.kStow), new SetIntakeState(IntakeState.kUp));
        }
    }

    public static Command goToAmpCommand() {
        if (RobotContainer.arm.getPosition() < ArmState.kShoot.rotations + 0.02) {
            // arm up
            // move arm and intake together
            return new ParallelCommandGroup(new SetArmState(ArmState.kAmp), new SetShooterState(ShooterState.kAmp));
        } else {
            // arm down
            // move arm then move intake
            return new ParallelCommandGroup(new SetShooterState(ShooterState.kAmp), new SetArmState(ArmState.kAmp), new SequentialCommandGroup(new WaitForArmStow()));
        }
    }

    public static Command goToFlatCommand() {
        if (RobotContainer.intake.isIntakeUp()) {
            // intake up
            // move arm flat
            return new ParallelCommandGroup(new SetArmState(ArmState.kShoot), new SetShooterState(ShooterState.kShootFromPodium));
        } else if (RobotContainer.arm.getPosition() < ArmState.kStow.rotations + 0.02) {
            // intake down and arm down
            // move arm to stow move intake up move arm to flat
            return new ParallelCommandGroup(new SetArmState(ArmState.kStow), new SequentialCommandGroup(new WaitForArmStow(), new SetIntakeState(IntakeState.kUp), new SetArmState(ArmState.kShoot)), new SetShooterState(ShooterState.kShootFromPodium));
        } else {
            // intake down and arm up
            // 
            return new ParallelCommandGroup(new SequentialCommandGroup(new ParallelCommandGroup(new SetArmState(ArmState.kStow), new SetIntakeState(IntakeState.kUp)), new SetArmState(ArmState.kShoot)), new SetShooterState(ShooterState.kShootFromPodium));
        }

    }

    public static Command goToHandoffCommand() {
        if (RobotContainer.arm.getPosition() < ArmState.kStow.rotations + 0.2) {
            // intake up
            // move intake then move 
            return new SequentialCommandGroup(new SetIntakeState(IntakeState.kDown), new ParallelCommandGroup(new SetArmState(ArmState.kIntake), new SetShooterState(ShooterState.kIntake)));
        } else if (!(RobotContainer.arm.isArmClearingIntake() || RobotContainer.intake.isIntakeDown())) {
            // arm flat
            // move arm up and intake down together, then arm down and shooter down together
            return new SequentialCommandGroup(new SequentialCommandGroup(new SetArmState(ArmState.kStow), new SetIntakeState(IntakeState.kDown)), new ParallelCommandGroup(new SetArmState(ArmState.kIntake), new SetShooterState(ShooterState.kIntake)));
        } else {
            // arm down
            // move arm and shooter together
            return new ParallelCommandGroup(new SetArmState(ArmState.kIntake), new SetShooterState(ShooterState.kIntake));
        }
    }

    public static Command goToSubwooferCommand() {
        if (RobotContainer.arm.getPosition() < ArmState.kStow.rotations + 0.2) {
            // arm up
            // move intake then move 
            return new SequentialCommandGroup(new SetIntakeState(IntakeState.kDown), new ParallelCommandGroup(new SetArmState(ArmState.kShootFromSubwoofer), new SetShooterState(ShooterState.kShootFromSubwoofer)));
        } else if (RobotContainer.arm.isAtState(ArmState.kShootFromPodium)) {
            // arm flat
            // move arm up and intake down together, then arm down and shooter down together
            return new SequentialCommandGroup(new SequentialCommandGroup(new SetArmState(ArmState.kStow), new SetIntakeState(IntakeState.kDown)), new ParallelCommandGroup(new SetArmState(ArmState.kShootFromSubwoofer), new SetShooterState(ShooterState.kShootFromSubwoofer)));
        } else {
            // arm down
            // move arm and shooter together
            return new ParallelCommandGroup(new SetArmState(ArmState.kShootFromSubwoofer), new SetShooterState(ShooterState.kShootFromSubwoofer));
        }
    }
}
