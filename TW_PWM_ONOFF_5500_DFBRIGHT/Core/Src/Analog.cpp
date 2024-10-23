/*************************************************************
 * Copyright (c) 2019 Apple Enterprises. This code is
 * proprietary and a trade secret of Apple Enterprises.
 *
 * $Workfile: Analog.cpp $
 *
 * $Creator: Chris Apple $
 *
 * $Description: Handles ADC hardware. $
 *
 * $Log:$
 *
 * $NoKeywords: $
 *************************************************************/

#include "ADC.h"
#include "Analog.h"

/*******************************************************************************
*
*	Defines
*
*******************************************************************************/

// TS ADC raw data acquired at a temperature of 30 �C (� 5 �C), VDDA= 3.3 V (� 10 mV)
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFF7A2C))

// TS ADC raw data acquired at a temperature of 110 �C (� 5 �C), VDDA= 3.3 V (� 10 mV)
#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFF7A2E))

#define VDD_CALIB_VREFINT ((uint16_t) (3300))                   // voltage that VREFINT was calibrated
#define VDD_BOARD ((uint16_t) (3300))
#define VDD_24 3884

#define V24_TRANSFER_FUNCTION (11)      // V24 / VIN

/*
 * Clock is the APB2 clock 84 MHz
 * prescaler of 6 = 14MHz (Must be less than 18MHz)
 * sampling time is set to 480 cycles = 34 us    (Minimum: 0.2us = 3 * 1/14MHz; Maximum: 34us = 480 * 1/14MHz)
 * internal VREF sensing time should be a minimum of 10us
 * internal VREF sampling time is set to 480 cycles = 34 us
 * temperature sensing time should be a minimum of 10us
 * temperature sampling time is set to 480 cycles = 34 us
 *
 * prescaler of 8 = 10.5MHz (Must be less than 18MHz)
 * sampling time is set to 480 cycles = 46 us    (Minimum: 0.28us = 3 * 1/10.5MHz; Maximum: 46us = 480 * 1/10.5MHz)
 * internal VREF sensing time should be a minimum of 10us
 * internal VREF sampling time is set to 480 cycles = 46 us
 * temperature sensing time should be a minimum of 10us
 * temperature sampling time is set to 480 cycles = 46 us
 */

/***************************************************************************
 *
 * Local Variables
 *
 ***************************************************************************/

CAnalog gAnalog;	// The one and only CAnalog object


#ifdef TRACK_ADC_INTERRUPTS_PER_SECOND
uint32_t gInterruptCounterAdc1;
uint32_t gNumberOfInterruptsAdc1;

#endif  //  TRACK_ADC_INTERRUPTS_PER_SECOND

/***************************************************************************
 *
 * Local Prototypes
 *
 ***************************************************************************/

/*****************************************************************************
 *
 * functions
 *
 ****************************************************************************/


CAnalog::CAnalog()
{
}

CAnalog::~CAnalog()
{
}

void CAnalog::InitializeCold()
{
    // read calibrated ADC values
    m_calibratedTemperatureDegrees30 = *TEMP30_CAL_ADDR;
    m_calibratedTemperatureDegrees110 = *TEMP110_CAL_ADDR;
    m_calibratedReferenceInternal = *VREFINT_CAL_ADDR;

    for ( int i=0; i<ADC_CHANNEL_TOTAL_ENUM; i++ ) {
        m_adcRawAccumulator[i] = 0;
        m_adcRawAccumulatorLatched[i] = 0;
        m_adcRawAverage[i] = 0;
        m_movingAverageAccumulator[i] = 0;
        for ( int j=0; j<RUNNING_AVERAGE_LENGTH; j++ ) {
            m_movingAverageTable[i][j] = 0;
        }
    }

    m_adc1RawAccumulatorReady = false;
    m_adc1RawAccumulatorCounter = 0;
    m_adc1MovingAverageIndex = 0;

    m_readingsValid = VALID_READING_NONE;

    // 1 - Initialize ADC peripheral
    MX_ADC1_Init();

    // 4 - Start conversion in DMA mode
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)m_dmaResultForADC1, ADC_CHANNEL_TOTAL) != HAL_OK)
    {
        Error_Handler();
    }
}

void CAnalog::PrepareForAbort()
{
}

void CAnalog::HandleIdleLoop()
{
    // check if a new latched totals ready
    if ( m_adc1RawAccumulatorReady )
    {
        m_adc1RawAccumulatorReady = false;
        HandleAdc1IdleLoop();
    }
}

void CAnalog::HandleAdc1IdleLoop()
{
    // totals get latched 18452 / RAW_ACCUMULATOR_SIZE times per second = (18452 / 1024) = 18 times per second
	AverageAdcReading(ADC_CHANNEL_VBUS_SNSF, m_adc1MovingAverageIndex, RAW_ACCUMULATOR_SIZE * NUMBER_OF_VBUS_SNSF_READINGS);
	AverageAdcReading(ADC_CHANNEL_ICLAMP_SNSF, m_adc1MovingAverageIndex, RAW_ACCUMULATOR_SIZE * NUMBER_OF_ICLAMP_SNSF_READINGS);
	AverageAdcReading(ADC_CHANNEL_VREF_INT, m_adc1MovingAverageIndex, RAW_ACCUMULATOR_SIZE * NUMBER_OF_VREFINT_READINGS);
	AverageAdcReading(ADC_CHANNEL_TEMPERATURE, m_adc1MovingAverageIndex, RAW_ACCUMULATOR_SIZE * NUMBER_OF_TEMPERATURE_READINGS);

    // bump running average index
    m_adc1MovingAverageIndex++;
    if ( m_adc1MovingAverageIndex >= RUNNING_AVERAGE_LENGTH ) {
        m_adc1MovingAverageIndex = 0;
        m_readingsValid = (VALID_READING_ADC)(m_readingsValid | VALID_READING_ADC_1);
    }

    // calculate actual board voltage
    if ( m_movingAverage[ADC_CHANNEL_VREF_INT] > 0 ) {
        m_millivoltsVdd = (m_calibratedReferenceInternal  * VDD_CALIB_VREFINT ) / m_movingAverage[ADC_CHANNEL_VREF_INT];
    } else {
        m_millivoltsVdd = 3300;
    }

    int32_t temperature; /* will contain the temperature in degree Celsius */
    int32_t vrefVoltage = m_millivoltsVdd; // VDD_BOARD
    temperature = (int32_t)m_movingAverage[ADC_CHANNEL_TEMPERATURE] * vrefVoltage / VDD_CALIB_VREFINT;
    temperature -= m_calibratedTemperatureDegrees30;
    temperature = temperature * (int32_t)(1100 - 300);
    temperature = temperature / (m_calibratedTemperatureDegrees110 - m_calibratedTemperatureDegrees30);
    m_temperature = (temperature + 300)/10;
    m_vbus = (24000*m_movingAverage[ADC_CHANNEL_VBUS_SNSF])/VDD_24;
}

void CAnalog::AverageAdcReading(ADC_CHANNEL adcReading, uint32_t movingAverageIndex, uint32_t numberOfReadingsInAcculumator)
{
    // compute average of raw readings
    m_adcRawAverage[adcReading] = (m_adcRawAccumulatorLatched[adcReading] + (numberOfReadingsInAcculumator>>1)) / numberOfReadingsInAcculumator;

    // compute new moving average by replacing oldest reading with neweset reading
    m_movingAverageAccumulator[adcReading] -=  m_movingAverageTable[adcReading][movingAverageIndex];
    m_movingAverageAccumulator[adcReading] +=  m_adcRawAverage[adcReading];

    // place the averaged reading in the running average table
    m_movingAverageTable[adcReading][movingAverageIndex] =  m_adcRawAverage[adcReading];

    // calculate moving average
    m_movingAverage[adcReading] = (m_movingAverageAccumulator[adcReading] + (RUNNING_AVERAGE_LENGTH-1)) / RUNNING_AVERAGE_LENGTH;
}

#ifdef TRACK_ADC_INTERRUPTS_PER_SECOND
void CAnalog::ProcessOneSecondInterrupt()
{
    gNumberOfInterruptsAdc1 = gInterruptCounterAdc1;
    gInterruptCounterAdc1 = 0;
}
#endif  //  TRACK_ADC_INTERRUPTS_PER_SECOND

void CAnalog::AddNewAdc1ToRawAccumulator()
{
   // add new readings to the total that will be averaged
    m_adcRawAccumulator[ADC_CHANNEL_VBUS_SNSF] += m_dmaResultForADC1[0];
    m_adcRawAccumulator[ADC_CHANNEL_ICLAMP_SNSF] += m_dmaResultForADC1[1];
    m_adcRawAccumulator[ADC_CHANNEL_TEMPERATURE] += m_dmaResultForADC1[2];
    m_adcRawAccumulator[ADC_CHANNEL_TEMPERATURE] += m_dmaResultForADC1[3];
    m_adcRawAccumulator[ADC_CHANNEL_VREF_INT] += m_dmaResultForADC1[4];
    
	m_adcRawAccumulator[ADC_CHANNEL_VBUS_SNSF] += m_dmaResultForADC1[5];
	m_adcRawAccumulator[ADC_CHANNEL_ICLAMP_SNSF] += m_dmaResultForADC1[6];
	m_adcRawAccumulator[ADC_CHANNEL_ICLAMP_SNSF] += m_dmaResultForADC1[7];
	m_adcRawAccumulator[ADC_CHANNEL_TEMPERATURE] += m_dmaResultForADC1[8];
    m_adcRawAccumulator[ADC_CHANNEL_VREF_INT] += m_dmaResultForADC1[9];
    
	m_adcRawAccumulator[ADC_CHANNEL_VBUS_SNSF] += m_dmaResultForADC1[10];
	m_adcRawAccumulator[ADC_CHANNEL_ICLAMP_SNSF] += m_dmaResultForADC1[11];
	m_adcRawAccumulator[ADC_CHANNEL_VBUS_SNSF] += m_dmaResultForADC1[12];
	m_adcRawAccumulator[ADC_CHANNEL_TEMPERATURE] += m_dmaResultForADC1[13];
    m_adcRawAccumulator[ADC_CHANNEL_VREF_INT] += m_dmaResultForADC1[14];


    // if enough readings have been read, latch the totals
    m_adc1RawAccumulatorCounter++;
    if ( m_adc1RawAccumulatorCounter >= RAW_ACCUMULATOR_SIZE ) {
		m_adcRawAccumulatorLatched[ADC_CHANNEL_VREF_INT] = m_adcRawAccumulator[ADC_CHANNEL_VREF_INT];
		m_adcRawAccumulator[ADC_CHANNEL_VREF_INT] = 0;

		m_adcRawAccumulatorLatched[ADC_CHANNEL_VBUS_SNSF] = m_adcRawAccumulator[ADC_CHANNEL_VBUS_SNSF];
		m_adcRawAccumulator[ADC_CHANNEL_VBUS_SNSF] = 0;

		m_adcRawAccumulatorLatched[ADC_CHANNEL_ICLAMP_SNSF] = m_adcRawAccumulator[ADC_CHANNEL_ICLAMP_SNSF];
		m_adcRawAccumulator[ADC_CHANNEL_ICLAMP_SNSF] = 0;

		m_adcRawAccumulatorLatched[ADC_CHANNEL_TEMPERATURE] = m_adcRawAccumulator[ADC_CHANNEL_TEMPERATURE];
        m_adcRawAccumulator[ADC_CHANNEL_TEMPERATURE] = 0;

        m_adc1RawAccumulatorCounter = 0;

        // set to convert ADC readings to useful values in background
        m_adc1RawAccumulatorReady = true;
    }
}

// HAL_ADC_ConvCpltCallback()_is called 18452 interrupts per second
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if ( hadc == &hadc1 ) {
        gAnalog.AddNewAdc1ToRawAccumulator();
#ifdef TRACK_ADC_INTERRUPTS_PER_SECOND
        gInterruptCounterAdc1++;
#endif  //  TRACK_ADC_INTERRUPTS_PER_SECOND
    }
}

/******************************************************************************
**                            End Of File
******************************************************************************/
