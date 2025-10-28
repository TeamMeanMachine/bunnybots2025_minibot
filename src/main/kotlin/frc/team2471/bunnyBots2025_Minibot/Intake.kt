package frc.team2471.bunnyBots2025_Minibot

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted

// Outer intake rollers use a falcon (phoenix 6) inner indexer uses bag (TalonSRX using phoenix 5). Both must use velocity control.
object Intake: SubsystemBase("Intake") {
   val table = NetworkTableInstance.getDefault().getTable("Intake")

   private val upperIntakingVoltageEntry = table.getEntry("Upper Intaking Velocity")
   private val sideIntakingVelocityEntry = table.getEntry("Side Intaking Velocity")
   private val indexingVelocityEntry = table.getEntry("Indexing Velocity")
   private val indexerMotorShootingVelocityEntry = table.getEntry("Index Motor Shooting Velocity")

   val upperIntakeMotor = TalonSRX(Talons.INTAKE_TOP)
   val sidesIntakeMotor = TalonFX(Falcons.INTAKE_SIDES)
   val indexerMotor = TalonFX(Talons.UPTAKE)

   val lowerBeambreakSensor = DigitalInput(DigitalSensors.LOWER_BEAMBREAK)
   val upperBeambreakSensor = DigitalInput(DigitalSensors.UPPER_BEAMBREAK)

   val lowerBeambreak: Boolean get() = lowerBeambreakSensor.get()
   val upperBeambreak: Boolean get() = upperBeambreakSensor.get()

   val upperIntakingVoltage: Double get() = upperIntakingVoltageEntry.getDouble(0.7 * 12.0) // 70%
   val sideIntakingVelocity: Double get() = sideIntakingVelocityEntry.getDouble(70.0) // 70%
   val indexingVelocity: Double get() = indexingVelocityEntry.getDouble(5.0)
   val indexerMotorShootingVelocity: Double get() = indexerMotorShootingVelocityEntry.getDouble(37.65) // 30%

   var currentIntakeState = IntakeState.HOLDING

   init {
      if (!upperIntakingVoltageEntry.exists()) {
         upperIntakingVoltageEntry.setDouble(upperIntakingVoltage)
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



//      upperIntakeMotor.applyConfiguration {
//         currentLimits(30.0, 40.0, 1.0)
//         inverted(false)
//         brakeMode()
//      }

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
            // switch to holding if full
            if (upperBeambreak) currentIntakeState = IntakeState.HOLDING
            // switch to indexing when carrot passes first beambreak
            if (lowerBeambreak) currentIntakeState = IntakeState.SHIFTING

//            upperIntakeMotor.set(VoltageOut(upperIntakingVoltage))
            sidesIntakeMotor.setControl(VelocityVoltage(sideIntakingVelocity))

            if (upperBeambreak) {
               indexerMotor.setControl(VelocityVoltage(0.0))
            } else {
               indexerMotor.setControl(VelocityVoltage(indexingVelocity))
            }
         }

         IntakeState.SHIFTING -> {
            // switch to holding if full or piece in indexer
            if (upperBeambreak || !lowerBeambreak) currentIntakeState = IntakeState.HOLDING

            sidesIntakeMotor.setControl(VelocityVoltage(sideIntakingVelocity))
            indexerMotor.setControl(VelocityVoltage(indexingVelocity))
         }

         IntakeState.HOLDING -> {
//            upperIntakeMotor.setControl(VelocityVoltage(0.0))
            sidesIntakeMotor.setControl(VelocityVoltage(0.0))
            indexerMotor.setControl(VelocityVoltage(0.0))
         }

         IntakeState.SHOOTING -> {
//            upperIntakeMotor.setControl(VelocityVoltage(upperIntakingVoltage))
            sidesIntakeMotor.setControl(VelocityVoltage(sideIntakingVelocity))
            indexerMotor.setControl(VelocityVoltage(indexerMotorShootingVelocity))
         }
         IntakeState.REVERSING -> {
//            upperIntakeMotor.setControl(VelocityVoltage(-upperIntakingVoltage))
            sidesIntakeMotor.setControl(VelocityVoltage(-sideIntakingVelocity))
            indexerMotor.setControl(VelocityVoltage(-indexingVelocity))
         }
      }
   }

   enum class IntakeState {
      INTAKING,
      SHIFTING,
      HOLDING,
      SHOOTING,
      REVERSING
   }
}