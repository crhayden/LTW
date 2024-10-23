/*************************************************************
 * Copyright (c) 2019 Apple Enterprises. This code is
 * proprietary and a trade secret of Apple Enterprises.
 *
 * $Workfile: Analog.h $
 *
 * $Creator: Chris Apple $
 *
 * $Description: Handles ADC hardware. $
 *
 * $Log:$
 *
 * $NoKeywords: $
 *************************************************************/

#ifndef __ANALOG_H__
#define __ANALOG_H__

/*
 * There are 3 ADCs: ADC1, ADC2 and ADC3. Each ADC is set to read 16 channels,
 * store the results using DMA and generate a callback when 16 channels are done.
 *
 * ADC1 only has internal ADCs: VRefInt and Temperature. Each channel is read 8 times
 * for each DMA callback.
 *
 */

#define RUNNING_AVERAGE_LENGTH 16       // maximum size is 16 for uint16_t
#define RAW_ACCUMULATOR_SIZE 16 // 1024
#define ADC_CHANNEL_TOTAL 15

class CAnalog
{
enum ADC_CHANNEL {
	ADC_CHANNEL_VBUS_SNSF,
	ADC_CHANNEL_ICLAMP_SNSF,
	ADC_CHANNEL_VREF_INT,
    ADC_CHANNEL_TEMPERATURE,
    ADC_CHANNEL_TOTAL_ENUM
};

#define NUMBER_OF_VBUS_SNSF_READINGS 4
#define NUMBER_OF_ICLAMP_SNSF_READINGS 4
#define NUMBER_OF_VREFINT_READINGS 3
#define NUMBER_OF_TEMPERATURE_READINGS 4

enum VALID_READING_ADC {
    VALID_READING_NONE = 0x00,
    VALID_READING_ADC_1 = 0x01,
    VALID_READING_ADC_2 = 0x02,
    VALID_READING_ADC_3 = 0x04,
    VALID_READING_ADC_ALL = (VALID_READING_ADC_1|VALID_READING_ADC_2|VALID_READING_ADC_3)
};

public:
	CAnalog();
	~CAnalog();

	void InitializeCold();
	void PrepareForAbort();
    void HandleIdleLoop();
#ifdef TRACK_ADC_INTERRUPTS_PER_SECOND
    void ProcessOneSecondInterrupt();
#endif  //  TRACK_ADC_INTERRUPTS_PER_SECOND
    bool AreReadingsValid() { return m_readingsValid == VALID_READING_ADC_ALL; }

	uint16_t GetRawAdcReadingVBus() { return m_adcRawAverage[ADC_CHANNEL_VBUS_SNSF]; }
	uint16_t GetRawAdcReadingIClampSnsf() { return m_adcRawAverage[ADC_CHANNEL_ICLAMP_SNSF]; }
	uint16_t GetRawAdcReadingVRefint() { return m_adcRawAverage[ADC_CHANNEL_VREF_INT]; }
    uint16_t GetRawAdcReadingTemperature() { return m_adcRawAverage[ADC_CHANNEL_TEMPERATURE]; }

    int32_t GetTemperature() { return m_temperature; }
    int32_t GetMillivoltsVdd() { return m_millivoltsVdd; }
    int32_t GetMillivoltsVbus() { return m_vbus; }

    void AddNewAdc1ToRawAccumulator();

protected:
    void HandleAdc1IdleLoop();
    void AverageAdcReading(ADC_CHANNEL adcReading, uint32_t movingAverageIndex, uint32_t numberOfReadingsInAcculumator);

    // variables per channel
private:
    uint32_t m_adcRawAccumulator[ADC_CHANNEL_TOTAL_ENUM];
    uint32_t m_adcRawAccumulatorLatched[ADC_CHANNEL_TOTAL_ENUM];
    uint32_t m_adcRawAverage[ADC_CHANNEL_TOTAL_ENUM];

    bool m_adc1RawAccumulatorReady;
    uint32_t m_adc1RawAccumulatorCounter;
    uint32_t m_adc1MovingAverageIndex;

    uint32_t m_movingAverageAccumulator[ADC_CHANNEL_TOTAL_ENUM];
    uint16_t m_movingAverageTable[ADC_CHANNEL_TOTAL_ENUM][RUNNING_AVERAGE_LENGTH];
    uint32_t m_movingAverage[ADC_CHANNEL_TOTAL_ENUM];

    int32_t m_calibratedTemperatureDegrees30;
    int32_t m_calibratedTemperatureDegrees110;
    int32_t m_calibratedReferenceInternal;

    int32_t m_temperature;
    int32_t m_millivoltsVdd;
    int32_t m_vbus;

    VALID_READING_ADC m_readingsValid;

public:
    // values set by DMA
    uint16_t m_dmaResultForADC1[ADC_CHANNEL_TOTAL];
};

extern CAnalog gAnalog;


#endif /* end __ANALOG_H__ */

/******************************************************************************
**                            End Of File
******************************************************************************/
