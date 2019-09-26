package frc.team2036.robot

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.CounterBase.EncodingType

import kotlin.math.*

enum class ElevatorMode {
    NONE, /* no speed */
    MANUAL, /* set to a value */
    ENCODER /* set to an encoder target to run to */
}

class Elevator (val motor: WPI_TalonSRX, val encoder: Encoder) {
    var mode: ElevatorMode = ElevatorMode.NONE
    var motor_speed: Double = 0.0
    var encoder_target: Long = 0
    var offset: Int = 0

    fun run() {
        if(mode == ElevatorMode.NONE) {
            motor.set(0.0)
        } else if(mode == ElevatorMode.MANUAL) {
            motor.set(motor_speed)
        } else if (mode == ElevatorMode.ENCODER) {
            val diff = encoder_target - (encoder.get() - offset)
            if (abs(diff) > 10.0) {
                val sign = abs(diff) / diff
                motor.set(-0.75 * sign)
            } else {
                motor.set(0.0)
            }
        }
    }

    fun setSpeed(speed: Double) {
        mode = ElevatorMode.MANUAL
        motor_speed = speed
    }

    fun setEncoder(target: Long) {
        mode = ElevatorMode.ENCODER
        encoder_target = target
    }

    fun setNone() {
        mode = ElevatorMode.NONE
    }

    fun reset() {
        offset = encoder.get()
    }
}