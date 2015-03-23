/**************************************************************************
 * Arduino driver library for the MAX31865.
 *
 * Copyright (C) 2015 Ole Wolf <wolf@blazingangles.com>
 *
 *
 * Wire the circuit as follows, assuming that level converters have been
 * added for the 3.3V signals:
 *
 *    Arduino Uno            -->  MAX31865
 *    ------------------------------------
 *    CS: any available pin  -->  CS
 *    MOSI: pin 11           -->  SDI (mandatory for hardware SPI)
 *    MISO: pin 12           -->  SDO (mandatory for hardware SPI)
 *    SCK: pin 13            -->  SCLK (mandatory for hardware SPI)
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
**************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <MAX31865.h>


/**
 * The constructor for the MAX31865_RTD class registers the CS pin and
 * configures it as an output.
 *
 * @param [in] cs_pin Arduino pin selected for the CS signal.
 */
MAX31865_RTD::MAX31865_RTD( ptd_type type, uint8_t cs_pin )
{
  /* Set the type of PTD. */
  this->type = type;

  /* CS pin for the SPI device. */
  this->cs_pin = cs_pin;
  pinMode( this->cs_pin, OUTPUT );

  /* Pull the CS pin high to avoid conflicts on SPI bus. */
  digitalWrite( this->cs_pin, HIGH );
}



/**
 * Configure the MAX31865.  The parameters correspond to Table 2 in the MAX31865
 * datasheet.  The parameters are combined into a control bit-field that is stored
 * internally in the class for later reconfiguration, as are the fault threshold values.
 *
 * @param [in] v_bias Vbias enabled (@a true) or disabled (@a false).
 * @param [in] conversion_mode Conversion mode auto (@a true) or off (@a false).
 * @param [in] one_shot 1-shot measurement enabled (@a true) or disabled (@a false).
 * @param [in] three_wire 3-wire enabled (@a true) or 2-wire/4-wire (@a false).
 * @param [in] fault_detection Fault detection cycle control (see Table 3 in the MAX31865
 *             datasheet).
 * @param [in] fault_clear Fault status auto-clear (@a true) or manual clear (@a false).
 * @param [in] filter_50hz 50 Hz filter enabled (@a true) or 60 Hz filter enabled
 *             (@a false).
 * @param [in] low_threshold Low fault threshold.
 * @param [in] high_threshold High fault threshold.
*/
void MAX31865_RTD::configure( bool v_bias, bool conversion_mode, bool one_shot,
                              bool three_wire, uint8_t fault_cycle, bool fault_clear,
                              bool filter_50hz, uint16_t low_threshold,
                              uint16_t high_threshold )
{
  uint8_t control_bits = 0;

  /* Assemble the control bit mask. */
  control_bits |= ( v_bias ? 0x80 : 0 );
  control_bits |= ( conversion_mode ? 0x40 : 0 );
  control_bits |= ( one_shot ? 0x20 : 0 );
  control_bits |= ( three_wire ? 0x10 : 0 );
  control_bits |= fault_cycle & 0b00001100;
  control_bits |= ( fault_clear ? 0x02 : 0 );
  control_bits |= ( filter_50hz ? 0x01 : 0 );

  /* Store the control bits and the fault threshold limits for reconfiguration
     purposes. */
  this->configuration_control_bits   = control_bits;
  this->configuration_low_threshold  = low_threshold;
  this->configuration_high_threshold = high_threshold;

  /* Perform an initial "reconfiguration." */
  reconfigure( );
}



/**
 * Reconfigure the MAX31865 by writing the stored control bits and the stored fault
 * threshold values back to the chip.
 */ 
void MAX31865_RTD::reconfigure( )
{
  /* Write the configuration to the MAX31865. */
  digitalWrite( this->cs_pin, LOW );
  SPI.transfer( 0x80 );
  SPI.transfer( this->configuration_control_bits );
  digitalWrite( this->cs_pin, HIGH );

  /* Write the threshold values. */
  digitalWrite( this->cs_pin, LOW );
  SPI.transfer( 0x83 );
  SPI.transfer( ( this->configuration_high_threshold >> 8 ) & 0x00ff );
  SPI.transfer(   this->configuration_high_threshold        & 0x00ff );
  SPI.transfer( ( this->configuration_low_threshold >> 8 ) & 0x00ff );
  SPI.transfer(   this->configuration_low_threshold        & 0x00ff );
  digitalWrite( this->cs_pin, HIGH );
}



/**
 * Apply the Callendar-Van Dusen equation to convert the RTD resistance
 * to temperature:
 *
 *   \f[
 *   t=\frac{-A\pm \sqrt{A^2-4B\left(1-\frac{R_t}{R_0}\right)}}{2B}
 *   \f],
 *
 * where
 *
 * \f$A\f$ and \f$B\f$ are the RTD coefficients, \f$R_t\f$ is the current
 * resistance of the RTD, and \f$R_0\f$ is the resistance of the RTD at 0
 * degrees Celcius.
 *
 * For more information on measuring with an RTD, see:
 * <http://newton.ex.ac.uk/teaching/CDHW/Sensors/an046.pdf>.
 *
 * @param [in] resistance The measured RTD resistance.
 * @return Temperature in degrees Celcius.
 */
double MAX31865_RTD::temperature( ) const
{
  static const double a2   = 2.0 * RTD_B;
  static const double b_sq = RTD_A * RTD_A;

  const double rtd_resistance =
    ( this->type == RTD_PT100 ) ? RTD_RESISTANCE_PT100 : RTD_RESISTANCE_PT1000;

  double c = 1.0 - resistance( ) / rtd_resistance;
  double D = b_sq - 2.0 * a2 * c;
  double temperature_deg_C = ( -RTD_A + sqrt( D ) ) / a2;

  return( temperature_deg_C );
}



/**
 * Read all settings and measurements from the MAX31865 and store them
 * internally in the class.
 *
 * @return Fault status byte
 */
uint8_t MAX31865_RTD::read_all( )
{
  uint16_t combined_bytes;

  /* Start the read operation. */
  digitalWrite( this->cs_pin, LOW );
  /* Tell the MAX31865 that we want to read, starting at register 0. */
  SPI.transfer( 0x00 );

  /* Read the MAX31865 registers in the following order:
       Configuration
       RTD
       High Fault Threshold
       Low Fault Threshold
       Fault Status */

  this->measured_configuration = SPI.transfer( 0x00 );

  combined_bytes  = SPI.transfer( 0x00 ) << 8;
  combined_bytes |= SPI.transfer( 0x00 );
  this->measured_resistance = combined_bytes >> 1;

  combined_bytes  = SPI.transfer( 0x00 ) << 8;
  combined_bytes |= SPI.transfer( 0x00 );
  this->measured_high_threshold = combined_bytes >> 1;

  combined_bytes  = SPI.transfer( 0x00 ) << 8;
  combined_bytes |= SPI.transfer( 0x00 );
  this->measured_low_threshold = combined_bytes >> 1;

  this->measured_status = SPI.transfer( 0x00 );

  digitalWrite( this->cs_pin, HIGH );

  /* Reset the configuration if the measured resistance is
     zero or a fault occurred. */
  if(    ( this->measured_resistance == 0 )
      || ( this->measured_status != 0 ) )
  {
    reconfigure( );
  }

  return( status( ) );
}

