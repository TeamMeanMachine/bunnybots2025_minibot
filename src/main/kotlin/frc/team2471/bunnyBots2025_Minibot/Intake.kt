package frc.team2471.bunnyBots2025_Minibot

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.loggedTalonFX.LoggedTalonFX

object Intake: SubsystemBase("Intake") {
   val table = NetworkTableInstance.getDefault().getTable("Intake")

   private val upperIntakingPercentEntry = table.getEntry("Upper Intaking Percentage")
   private val sideIntakingPercentEntry = table.getEntry("Side Intaking Percentage")
   private val indexingPercentEntry = table.getEntry("Indexing Percentage")
   private val indexerMotorShootingPercentEntry = table.getEntry("Index Motor Shooting Percentage")

   private val upperBeambreakEntry = table.getEntry("Upper Beambreak Entry")
   private val lowerBeambreakEntry = table.getEntry("Lower Beambreak Entry")

   val upperIntakeMotor = TalonSRX(Talons.INTAKE_TOP)
   val sidesIntakeMotor = LoggedTalonFX(Falcons.INTAKE_SIDES)
   val indexerMotor = LoggedTalonFX(Talons.UPTAKE)

   val lowerBeambreakSensor = DigitalInput(DigitalSensors.LOWER_BEAMBREAK)
   val upperBeambreakSensor = DigitalInput(DigitalSensors.UPPER_BEAMBREAK)

   val lowerBeambreak: Boolean get() = !lowerBeambreakSensor.get()
   val upperBeambreak: Boolean get() = !upperBeambreakSensor.get()

   val upperIntakingPercentage: Double get() = upperIntakingPercentEntry.getDouble(0.7) // 70%
   val sideIntakingPercentage: Double get() = sideIntakingPercentEntry.getDouble(0.7) // 70%
   val indexingPercentage: Double get() = indexingPercentEntry.getDouble(0.5)
   val indexerMotorShootingPercentage: Double get() = indexerMotorShootingPercentEntry.getDouble(0.7)

   var currentIntakeState = IntakeState.HOLDING

   init {
      if (!upperIntakingPercentEntry.exists()) {
         upperIntakingPercentEntry.setDouble(upperIntakingPercentage)
      }
      if (!sideIntakingPercentEntry.exists()) {
         sideIntakingPercentEntry.setDouble(sideIntakingPercentage)
      }
      if (!indexingPercentEntry.exists()) {
         indexingPercentEntry.setDouble(indexingPercentage)
      }
      if (!indexerMotorShootingPercentEntry.exists()) {
         indexerMotorShootingPercentEntry.setDouble(indexerMotorShootingPercentage)
      }



      upperIntakeMotor.configContinuousCurrentLimit(30)
      upperIntakeMotor.configPeakCurrentLimit(40)
      upperIntakeMotor.configPeakCurrentDuration(1000)

      sidesIntakeMotor.applyConfiguration {
         currentLimits(30.0, 40.0, 1.0)
         inverted(false)
         coastMode()
      }

      indexerMotor.applyConfiguration {
         currentLimits(30.0, 40.0, 1.0)
         inverted(true)
         coastMode()
      }

   }

   override fun periodic() {
      when (currentIntakeState) {
         IntakeState.INTAKING -> {
            // switch to holding if full
            if (upperBeambreak) currentIntakeState = IntakeState.HOLDING
            // switch to indexing when carrot passes first beambreak
            if (lowerBeambreak) currentIntakeState = IntakeState.SHIFTING

            upperIntakeMotor.set(TalonSRXControlMode.PercentOutput, upperIntakingPercentage)
            sidesIntakeMotor.setControl(VoltageOut(sideIntakingPercentage * 12.0))

//            if (upperBeambreak) {
//               indexerMotor.setControl(VoltageOut(0.0))
//            } else {
//               indexerMotor.setControl(VoltageOut(indexingPercentage * 12.0))
//            }
         }

         IntakeState.SHIFTING -> {
            // switch to holding if full or piece in indexer
            if (upperBeambreak || !lowerBeambreak) currentIntakeState = IntakeState.HOLDING

            sidesIntakeMotor.setControl(VoltageOut(sideIntakingPercentage * 12.0))
            indexerMotor.setControl(VoltageOut(indexingPercentage * 12.0))
         }

         IntakeState.HOLDING -> {
            upperIntakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0)
            sidesIntakeMotor.setControl(VoltageOut(0.0))
            indexerMotor.setControl(VoltageOut(0.0))
         }

         IntakeState.SHOOTING -> {
            upperIntakeMotor.set(TalonSRXControlMode.PercentOutput, upperIntakingPercentage)
            sidesIntakeMotor.setControl(VoltageOut(sideIntakingPercentage * 12.0))
            indexerMotor.setControl(VoltageOut(indexerMotorShootingPercentage * 12.0))
         }

         IntakeState.REVERSING -> {
            upperIntakeMotor.set(TalonSRXControlMode.PercentOutput, -upperIntakingPercentage)
            sidesIntakeMotor.setControl(VoltageOut(-sideIntakingPercentage * 12.0))
            indexerMotor.setControl(VoltageOut(-indexingPercentage * 12.0))
         }
      }

      Logger.recordOutput("Intake state", currentIntakeState.name)

      upperBeambreakEntry.setBoolean(upperBeambreak)
      lowerBeambreakEntry.setBoolean(lowerBeambreak)
   }

   enum class IntakeState {
      INTAKING,
      SHIFTING,
      HOLDING,
      SHOOTING,
      REVERSING
   }
}