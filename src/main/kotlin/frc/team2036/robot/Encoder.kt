package frc.team2036.robot

import edu.wpi.first.wpilibj.AnalogInput

class Encoder(val a_port: Int, val b_port: Int): Thread() {


    var a_chanel: AnalogInput
    var b_chanel: AnalogInput

    /**
     * John Deere Rotary Encoder
     * Signal Output on Forward Motion:
     * \     |\     |\
     *   \   |  \   |  \
     *     \ |    \ |    \
     *       \      \      \
     * 0     1      2
     *      Rotation
     * Chanel A and B are offset by a half rotation - thus, when a is transitioning from 0 to 1, b is 0.5
     * Notes: Voltage only ranges from 0.1 to 0.9 of full voltage (0.5 - 0.45)
     */

     val volt_low: Double = 0.5
     val volt_high: Double = 4.5

     /* Algorithm:
      Sum difference between last measurment and current measurement
      problem occurs when signal transitions from low to high (or vice versa) - how
      can we know it was transition instead of the motor just going backwards

      Because we have the two signals, we just take the difference on the chanel currently closest
      to 2.5 volts - this is the chanel least likely to have taken a jump, and we can just sum difference

      This only works if we get at least 2 samples per rotation (assuming perfect theoretical
      operation). We should really aim more towards 4-8 samples per rotation */

      /* past vals for diff summing */
      var past_a: Double = 0.0
      var past_b: Double = 0.0

      /* position (in rotations) */
      var pos: Double = 0.0


    init {
        /* Init analog input hardware globals */
        AnalogInput.setGlobalSampleRate(62500.0)

        this.a_chanel = AnalogInput(a_port)
        this.b_chanel = AnalogInput(b_port)

        this.a_chanel.setOversampleBits(4)
        this.a_chanel.setAverageBits(2)

        this.b_chanel.setOversampleBits(4)
        this.b_chanel.setAverageBits(2)
    }

    /* scale voltage to 0 to 1 range */
    fun scaleVolts(voltage: Double): Double {
        val res: Double = (voltage - this.volt_low) / (this.volt_high - this.volt_low)

        return maxOf(minOf(res, 1.0), 0.0)
    }

    /* Main Measuring Loop */
    fun loop(){
        val a_val: Double = this.scaleVolts(this.a_chanel.getVoltage())
        val b_val: Double = this.scaleVolts(this.b_chanel.getVoltage())


        var close_val: Double = 0.0
        var close_past: Double = 0.0

        /* choose channel to use */
        /* Use A chanel */
        if(Math.abs(a_val - 0.5) < Math.abs(b_val - 0.5)){
            close_val = a_val
            close_past = past_a
        }
        /* Use B chanel */
        else {
            close_val = b_val
            close_past = past_b
        }

        /* Sum diff */
        this.pos += close_val - close_past

        this.past_a = a_val
        this.past_b = b_val
    }

    override fun run(){
        this.past_a = this.scaleVolts(this.a_chanel.getVoltage())
        this.past_b = this.scaleVolts(this.b_chanel.getVoltage())
        while(true){
            this.loop()
        }
    }

}
