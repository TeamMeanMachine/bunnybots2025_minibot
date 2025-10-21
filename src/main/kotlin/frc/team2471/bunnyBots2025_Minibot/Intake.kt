package frc.team2471.bunnyBots2025_Minibot

import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted

// Outer intake rollers use a falcon (phoenix 6) inner indexer uses bag (TalonSRX using phoenix 5). Both must use velocity control.
object Intake: SubsystemBase("Intake") {
   val table = NetworkTableInstance.getDefault().getTable("Intake")

   val upperIntakingVelocityEntry = table.getEntry("Upper Intaking Velocity")
   val sideIntakingVelocityEntry = table.getEntry("Side Intaking Velocity")
   val indexingVelocityEntry = table.getEntry("Indexing Velocity")
   val indexerMotorShootingVelocityEntry = table.getEntry("Index Motor Shooting Velocity")

   val upperIntakeMotor = TalonFX(Talons.INTAKE_TOP)
   val sidesIntakeMotor = TalonFX(Falcons.INTAKE_SIDES)
   val indexerMotor = TalonFX(Talons.UPTAKE)

   val lowerBeambreakSensor = DigitalInput(DigitalSensors.LOWER_BEAMBREAK)
   val upperBeambreakSensor = DigitalInput(DigitalSensors.UPPER_BEAMBREAK)

   val lowerBeambreak: Boolean get() = lowerBeambreakSensor.get()
   val upperBeambreak: Boolean get() = upperBeambreakSensor.get()

   val upperIntakingVelocity: Double get() = upperIntakingVelocityEntry.getDouble(5.0) // 70%
   val sideIntakingVelocity: Double get() = sideIntakingVelocityEntry.getDouble(5.0) // 70%
   val indexingVelocity: Double get() = indexingVelocityEntry.getDouble(5.0)
   val indexerMotorShootingVelocity: Double get() = indexerMotorShootingVelocityEntry.getDouble(5.0) // 30%

   var currentIntakeState = IntakeState.HOLDING

   init {
      if (!upperIntakingVelocityEntry.exists()) {
         upperIntakingVelocityEntry.setDouble(upperIntakingVelocity)
      }
      if (!sideIntakingVelocityEntry.exists()) {
         sideIntakingVelocityEntry.setDouble(sideIntakingVelocity)
      }
      if (!indexingVelocityEntry.exists()) {
         indexingVelocityEntry.setDouble(indexingVelocity)
      }
      if (!indexerMotorShootingVelocityEntry.exists()) {
         indexerMotorShootingVelocityEntry.setDouble(indexerMotorShootingVelocity)
      }



      upperIntakeMotor.applyConfiguration {
         currentLimits(30.0, 40.0, 1.0)
         inverted(false)
         brakeMode()
      }

      sidesIntakeMotor.applyConfiguration {
         currentLimits(30.0, 40.0, 1.0)
         inverted(false)
         brakeMode()
      }

      indexerMotor.applyConfiguration {
         currentLimits(30.0, 40.0, 1.0)
         inverted(false)
         brakeMode()
      }

   }

   override fun periodic() {
      when (currentIntakeState) {
         IntakeState.INTAKING -> {
            // switch to indexing when carrot passes first beambreak
            if (lowerBeambreak) currentIntakeState = IntakeState.SHIFTING

            upperIntakeMotor.setControl(VelocityVoltage(upperIntakingVelocity))
            sidesIntakeMotor.setControl(VelocityVoltage(sideIntakingVelocity))
            indexerMotor.setControl(VelocityVoltage(indexingVelocity))
         }

         IntakeState.SHIFTING -> {
            // switch to holding when carrot passes second beambreak
            if (upperBeambreak) currentIntakeState = IntakeState.HOLDING

            sidesIntakeMotor.setControl(VelocityVoltage(sideIntakingVelocity))
            indexerMotor.setControl(VelocityVoltage(indexingVelocity))
         }

         IntakeState.HOLDING -> {
            upperIntakeMotor.setControl(VelocityVoltage(0.0))
            sidesIntakeMotor.setControl(VelocityVoltage(0.0))
            indexerMotor.setControl(VelocityVoltage(0.0))
         }

         IntakeState.SHOOTING -> {
            upperIntakeMotor.setControl(VelocityVoltage(upperIntakingVelocity))
            sidesIntakeMotor.setControl(VelocityVoltage(sideIntakingVelocity))
            indexerMotor.setControl(VelocityVoltage(indexerMotorShootingVelocity))
         }
      }
   }

   enum class IntakeState {
      INTAKING,
      SHIFTING,
      HOLDING,
      SHOOTING
   }
}