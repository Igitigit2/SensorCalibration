#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"

#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"

#define MAX_FIELDSTRNGTH_VAR        (400.0f)        // Max. allowed variation of the magnetic field strength
#define ABS_ERROR_THRESHOLD         (0.04f)         // Max. allowed position error
#define MAX_MEAN_ERROR              (0.02)          // Max allowed average error during one full turn. Must be very close to 0.0!

#ifndef RAD2DEG
#define RAD2DEG(r)      ((r) * 180.0f / PI)
#endif

// Classes to 
//  - calibrate SPI connected magnetic sensors
//  - test the accuracy of AS5x74 magnetic sensors connected via SPI.
//  - derive anti cogging parameters for the motor. (not really....)
// All functions may be called from setup() and do not require a FOC loop to run,
// The CalibratedSensorSPI class may be used exactly the same way, as the MagneticSensorAS5047 class,
// except for the declaration.
//
// Usage example:

/*
    #include "Wire.h"
    #include "SPI.h"
    #include <SimpleFOC.h>
    #include "SimpleFOCDrivers.h"
    #include "encoders/as5047/MagneticSensorAS5047.h"
    #include "CalibratedSensorSPI.h"

    // Header file, containing the data produced by the calibration routine (data copied from Serial output window). 
    // NOTE: If you did not yet run the calibration, you don't have this file yet, and you want to calibrate youzr sensor,
    // don't set the define CALIBRATION_DATA! Once the sensor is calibrated, copy the data which were printed to serial
    // into the header file which is used as the define here.
    #define CALIBRATION_DATA "CalibrationData.h"        

	SPIClass spi_ssi(PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK, AS5047_SS);    

	#ifdef CALIBRATION_DATA
		#include CALIBRATION_DATA
	#endif

    // The sensor must be an instance of the sensor diagnostics template!!!
	CalibratedSensorSPI<AS5xxxDiagnostics<MagneticSensorAS5047>> sensor(AS5047_SS);             // AS5xxx with diagnostics
	// CalibratedSensorSPI<DefaultSensorDiagnostics<MagneticSensorAS5047>> sensor(AS5047_SS);   // Sensor without diagnostics

    setup()
    {
        ...
	#ifdef CALIBRATION_DATA
        // Use sensor with existing calibration data.
        sensor.init(&spi_ssi, &motor, CalibrationCoefficients, NumCalibrationCoefficients); 
    #else
        // Perform new calibration.
        sensor.init(&spi_ssi, &motor); 
        sensor.Calibrate();     // This will print C-code to Serial. Copy it and safe it to the header file CALIBRATION_DATA!
    #endif

        // Optional:
        sensor.Validate();
        ...
	    motor.init();
        ...
		Serial.println("Sensor test and calibration...");
		if (!sensor.CheckCalibration()) 
		{	
			Serial.println("ERROR: Sensor test failed!");
			motor.disable();
			while(1);
		}
        ...
    }

    loop()
    {
        ...
        if (sensor.CheckSensor()!=0)
        {
            motor.disable();
            Serial.println("Sensor probably misaligned, stopping motor!");
            while(1);
        }
        ...
    }
*/

//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------



//********************************************************************

    // Print all correction coefficients to a table in C++ format,
    // including everything to use it as a header file.
    // Copy the text from the debug output and store it as a .h file.
    // The .h file can hen be included in your main code and calibration data from
    // the .h file can be used in subsequent calls to init(). 
    // Usually this function will be called directly by RunPositionTest().

/*
Typical format of the output:

//----------------  Calibration data header file start --------------------------
#ifndef XYZ_H
#define XYZ_Z

#define NumCoefficients     xxxx

float* SensorCalibrationData[] =
{
  1.3, 3.4, ....
};
#endif
//----------------  Calibration data header file end --------------------------

*/
template <typename T>
void PrintArray(const int ArraySize, const T* pArray, const char* ArrayName, const float Factor=1.0)
{
    Serial.printf("//----------------  %s header file start --------------------------\n", ArrayName);
    Serial.printf("#ifndef %s_H\n", ArrayName);
    Serial.printf("#define %s_H\n", ArrayName);
    Serial.println("");
    Serial.printf("#define Num%s\t%d", ArrayName, ArraySize);
    Serial.println("");
    Serial.printf ("float %s[] =\n", ArrayName);
    Serial.println("{");
    for (int i=0; i<ArraySize; i++)
    {
        if (i%16==0)
        {
            if (i==0)
                Serial.printf("\t");
            else
                Serial.printf(",\n\t");
            //Serial.printf("%7.5ff", i*Factor);
            Serial.printf("%7.5ff", pArray[i]*Factor);
        }
        else
            Serial.printf(", %7.5ff", pArray[i]*Factor);
            //Serial.printf(", %7.5ff", i*Factor);
    }

    Serial.println("\n};");
    Serial.println("#endif");
    Serial.printf("//----------------  %s header file end --------------------------\n", ArrayName);
}
 
template void PrintArray<> (const int ArraySize, const int8_t* pArray, const char* ArrayName, const float Factor);
template void PrintArray<> (const int ArraySize, const float*  pArray, const char* ArrayName, const float Factor);


void FIRFilter(float* pSrc, float* pDest, int NumSamples, int WindowSize)
{
    /// Perform filtering to linearize position sensor eccentricity
    /// FIR n-sample average, where n = number of samples in one electrical cycle
    /// This filter has zero gain at electrical frequency and all integer multiples
    /// So cogging effects should be completely filtered out.
    for (int i = 0; i < NumSamples; i++)
    {
        pDest[i] = 0;
        for (int j = 0; j < WindowSize; j++)
        {
            int ind = -WindowSize / 2 + j + i; // Indexes from -WindowSize/2 to + WindowSize/2
            if (ind < 0)
            {
                ind += NumSamples;
            } // Moving average wraps around
            else if (ind > NumSamples - 1)
            {
                ind -= NumSamples;
            }
            pDest[i] += pSrc[ind] / (float)WindowSize;
        }
    }
}

#define SENSOR_ERROR_NOT_SET                0
#define SENSOR_ERROR_POS_ERROR_TOO_HIGH     1
#define SENSOR_ERROR_MEAN_ERR_NOT_0         2
#define SENSOR_ERROR_PARITY                 3
#define SENSOR_ERROR_FRAMING                4
#define SENSOR_ERROR_INV_COMMAND            5
#define SENSOR_ERROR_FIELD_HIGH             6
#define SENSOR_ERROR_FIELD_LOW              7
#define SENSOR_ERROR_FIELD_VARIATION        8


//---------------------------------------------------------------------
//
//   Sensor specific templates which include the diagnostic functions
//   These must be insantiated with an SPI conected sensor type, e.g.
//   AS5xxxDiagnostics<MagneticSensorAS5047> sensor(AS5047_SS);   

// Default diagnostic class template. Any diagnostics class for
// specific sensors must be derived from this class template.
template <class T>
class DefaultSensorDiagnostics : public T
{
    public:
        DefaultSensorDiagnostics(const int SensorType) : T(SensorType)
        {
            DiagnosticsAvailable = false;
        }

        // Return SENSOR_ERROR_xxx 
        virtual int CheckSensor(float* MinMag=0, float* MaxMag=0)
        {
            return SENSOR_ERROR_NOT_SET;
        }

        bool DiagnosticsAvailable;
};

#ifdef AS5047_CPR           // Only available if MagneticSensorAS5047.h was included before this file!

    // Diagnostics for AS5xxx family sensors. Derived from DefaultSensorDiagnostics.
    template <class T>
    class AS5xxxDiagnostics : public DefaultSensorDiagnostics<T>
    {
        public:
            AS5xxxDiagnostics(const int SensorType) : DefaultSensorDiagnostics<T>::DefaultSensorDiagnostics(SensorType)
            {
                MinMagneticMagnitude=1e5f;
                MaxMagneticMagnitude=0.0f;
                DefaultSensorDiagnostics<T>::DiagnosticsAvailable = true;
                AS5047Error ErrorFlags = T::clearErrorFlag();
            }


            // Update available diagnostic information and make a quick accuracy check.
            // Does not move the motor. Returns SENSOR_ERROR_xxx.
            int CheckSensor(float* MinMag=0, float* MaxMag=0)
            {
                int rc = SENSOR_ERROR_NOT_SET;

                // Diagnostics from SPI
                // Read the CORDIC magnitude value, a measure of the magnet field strength
                float Mag = T::readMagnitude();
                MinMagneticMagnitude = min(MinMagneticMagnitude, Mag);
                MaxMagneticMagnitude = max(MaxMagneticMagnitude, Mag);

                if (MinMag) *MinMag = MinMagneticMagnitude;
                if (MaxMag) *MaxMag = MaxMagneticMagnitude;

                // Check for errors
                if (T::isErrorFlag()) 
                {
                    AS5047Error ErrorFlags = T::clearErrorFlag();
                    if (ErrorFlags.parityError)  
                        { Serial.println("Parity error"); rc = SENSOR_ERROR_PARITY; }       
                    if (ErrorFlags.framingError)  
                        { Serial.println("Framing error"); rc = SENSOR_ERROR_FRAMING; }       
                    if (ErrorFlags.commandInvalid)  
                        { Serial.println("Invalid command"); rc = SENSOR_ERROR_INV_COMMAND; }       
                }

                // Get diagnostics
                AS5047Diagnostics Diagnostics = T::readDiagnostics();
                
                // Error if magnetic field strength too high or too low
                if (Diagnostics.magh)
                    { Serial.printf("CalibratedSensorSPI: Magnetic field strength too high!\n\tMinMagnitude=%.0f, MaxMagnitude=%.0f, Delta Magnitude=%.0f\n", MinMagneticMagnitude, MaxMagneticMagnitude, MaxMagneticMagnitude-MinMagneticMagnitude); rc = SENSOR_ERROR_FIELD_HIGH; }

                if (Diagnostics.magl)
                    { Serial.printf("CalibratedSensorSPI: Magnetic field strength too low!\n\tMinMagnitude=%.0f, MaxMagnitude=%.0f, Delta Magnitude=%.0f\n", MinMagneticMagnitude, MaxMagneticMagnitude, MaxMagneticMagnitude-MinMagneticMagnitude); rc = SENSOR_ERROR_FIELD_LOW; }

                if (MaxMagneticMagnitude-MinMagneticMagnitude > MaxFieldStrengthVariation)
                    { Serial.printf("CalibratedSensorSPI: Magnetic field strength varied too much!\n\tMinMagnitude=%.0f, MaxMagnitude=%.0f, Delta Magnitude=%.0f\n", MinMagneticMagnitude, MaxMagneticMagnitude, MaxMagneticMagnitude-MinMagneticMagnitude); rc = SENSOR_ERROR_FIELD_VARIATION; }

                return rc;
            }
         
        private:
            const float MaxFieldStrengthVariation = MAX_FIELDSTRNGTH_VAR;
            float MinMagneticMagnitude;
            float MaxMagneticMagnitude;
    };
#endif



//---------------------------------------------------------------------
//
//   The actual sensor calibration routines.
//   init() will initialize the sensor. If pCalibrationCoefficients is provided,
//   Angle correction is automatically applied when getSensorAngle() is called.
//   If pCalibrationCoefficients is NOT  provided, the class behaves like any
//   SimpleFOC sensor class without calibration.
// 
//   RunPositionTest() called without pCalibrationCoefficients passed to init()
//   before, will create sensor calibration data and print them to Serial.
//   Copy the text from the debug output and store it as a .h file (be aware of unintentional line breaks!).
//   The .h file can then be included in your main code and can be used
//   when calling init() next time. Calling RunPositionTest() can then be omitted in future runs.
//   If RunPositionTest() is called with pCalibrationCoefficients available, then a much 
//   shorter position test will be performed and no new calibration data will be generated.
// 
//   The template argument T must be derived from or be DefaultSensorDiagnostics:
//	   CalibratedSensorSPI<AS5xxxDiagnostics<MagneticSensorAS5047>> sensor(AS5047_SS);
// 	 or, if the sensor driver does not support diagnostics:
//     CalibratedSensorSPI<DefaultSensorDiagnostics<MagneticSensorAS5047>> sensor(AS5047_SS);

template <class T>
class CalibratedSensorSPI : public T
{
  public:
    CalibratedSensorSPI(const int SensorType) : T(SensorType)
    {
        MinAngleError=0.0f;
        MaxAngleError=0.0f;
        CalibrationCoefficients=0;
        CoggingTable = 0;
        CoggingTableSize = 0;
        NumSamples = 0;
        CalibrationDataExternal = false;
        SensorErrorCode = SENSOR_ERROR_NOT_SET;
    }

    virtual ~CalibratedSensorSPI()
    {
        if (CalibrationCoefficients && !CalibrationDataExternal) delete[] CalibrationCoefficients;
    }

    virtual void init(SPIClass* _spi, BLDCMotor* Motor, int CPR, float* pCalibrationCoefficients=0, int _NumSamples=0)
    {
        pMotor = Motor;
        NumSamples = _NumSamples; 
        if (pCalibrationCoefficients)
        {
        if (CalibrationCoefficients && !CalibrationDataExternal) delete[] CalibrationCoefficients;
            CalibrationCoefficients = pCalibrationCoefficients;
            CalibrationDataExternal = true;
        }
        SensorCPR = CPR;
        if (CalibrationCoefficients)
            Serial.println("Sensor init():  Calibration data provided");
        T::init(_spi);
    }

    // Overload for the getSensorAngle() method from the base sensor class.
    // called by SimpleFOC. Here, the angle correction is applied.
    // The method calls the base class, applies the correction to the readings
    // and returns the corrected angle. 
    virtual float getSensorAngle();

    // Spin the motor NumRevolutions revolutions using open loop.
    // NumRevolutions>0 turns clockwise, <0 counter clockwise.
    void SpinMotor(float NumRevolutions);
    int SpinMotorWithCheck(float NumRevolutions);

    // Print a table with updated calibration coefficients to Serial.
    // Calibration will perform TurnsPerDirection revolutions in each direction,
    // starting clockwise if StartDir==+1 or counter clockwise if StartDir==-1.
    // More revolutions lead to more accurate results..
    bool Calibrate(const int StartDir=1, const int TurnsPerDirection=16, const bool Quiet=false)
    {
        return RunPositionTest(StartDir, TurnsPerDirection, Quiet, false, true); 
    }

    // Calculate and print the residual error of a calibrated sensor.
    // This will perform one revolution in each direction,
    // starting clockwise if StartDir==+1 or counter clockwise if StartDir==-1.
    bool Validate(const int StartDir, const bool Quiet=false)
    {
        return RunPositionTest(StartDir, 1, Quiet, true, false); 
    }

    // Perform a quick check of a calibrated sensor.
    // This will perform one revolution in each direction,
    // starting clockwise if StartDir==+1 or counter clockwise if StartDir==-1.
    bool CheckCalibration(const int StartDir=1)
    {
        return RunPositionTest(StartDir, 100, true, false, false); 
    }

    bool CalculateAntiCoggingData(const bool Quiet=false);

    int GetSensorErrorCode() {return SensorErrorCode;};

  protected:

    // Spin the motor Turns turns in StartDir (1=cw, -1=ccw) using openloop.
    // Determine the min and max deviation between electrical angle and sensor angle.
    // Min and max will update Min/MaxAngleError members.
    // If quiet is set, only summaries will be printed.
    // Return true if everything is within reasonable limits, false otherwise.
    // The motor will spin <Turns> turns in direction <StartDir> and the
    // the same number of turns in reverse direction. The larger <turns> is, the
    // more accurate the lookup table will be at the end.
    // <StartDir> >0 = cw, <0=ccw.
    // If <Quiet> is true, only minimal output will be printed to Serial.
    bool RunPositionTest(const int StartDir, const int Turns, const bool Quiet=false, bool Validate=false, const bool Calibrate=false);

    bool  CheckAngle(const float MeasuredAngle, const float ExpectedAngle, const float MaxDifference=_PI);
    float ReadAngle(int* TimeToSettleMS=0);
    float ReadRawAngle();
    float GetAngleOversampled(const int OversamplingFactor);
    float GetRawAngleOversampled(const int OversamplingFactor);    
    bool  MoveToSensorZero(const int StartDir, float *pInitialAngle, float *pInitialElectricalAngle);

    const float AbsErrorThreshold = ABS_ERROR_THRESHOLD;
 
    int NumSamples;                                // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)

    BLDCMotor* pMotor;
    int8_t* CoggingTable;
    int CoggingTableSize;

    float MinAngleError;
    float MaxAngleError;
    float* CalibrationCoefficients;
    float MinError;
    float MaxError;
    int TurnsPerDirection;
    int SensorCPR;

    bool CalibrationDataExternal;

    int SensorErrorCode;
};


// Make a coarse validation of a sensor angle. Return false if it is
// not a number or outside of an expected range.
template <class T>
bool CalibratedSensorSPI<T>::CheckAngle(const float MeasuredAngle, const float ExpectedAngle, const float MaxDifference)
{
    bool rc = true;
    if (isnan(MeasuredAngle)) rc=false;
    if (fabs(MeasuredAngle-ExpectedAngle)>_PI) rc=false;
    if (!rc)
        Serial.printf("Error: CalibratedSensorSPI<T>::CheckAngle(), MeasuredAngle=%.5f, ExpectedAngle=%.5f\n", MeasuredAngle, ExpectedAngle);
    return rc;
}

template <class T>
float CalibratedSensorSPI<T>::getSensorAngle()
{
    if (!CalibrationCoefficients)
        return T::getSensorAngle();
    else
    {
        float Angle = T::getSensorAngle();
        float RationalIndex = Angle / (_2PI/(float)NumSamples);

        // Get the corection factors for next smaller and larger indices from the LUT     
        int   SmallerIndex = floor(RationalIndex);
        float Offset0 = CalibrationCoefficients[SmallerIndex];
        float Offset1 = CalibrationCoefficients[(SmallerIndex+1)%NumSamples];

        // Interpolate between the correction value at SmallerIndex and at SmallerIndex+1 
        float Remainder = RationalIndex - SmallerIndex;
        float Offset = Remainder*Offset1 + (1-Remainder)*Offset0;
        float CorrectedAngle = Angle + Offset;

        return CorrectedAngle;
    }
}

template <class T>
float CalibratedSensorSPI<T>::GetAngleOversampled(const int OversamplingFactor)
{
    float Angle = 0.0f;
    for (int i=0; i<OversamplingFactor; i++)
    {
        _delay(1);
        T::update();
        Angle += T::getAngle();
    }
    Angle /= OversamplingFactor;

    return Angle;
}


template <class T>
float CalibratedSensorSPI<T>::GetRawAngleOversampled(const int OversamplingFactor)
{
    float Angle = 0.0f;
    for (int i=0; i<OversamplingFactor; i++)
    {
        _delay(4);
        T::update();
        float TempAngle = T::getSensorAngle();
        if (TempAngle>_2PI-10 * _2PI/SensorCPR)
            TempAngle -= _2PI;
        Angle += TempAngle;
    }
    Angle /= OversamplingFactor;
    if (Angle<0)
        Angle += _2PI;

    return Angle;
}


//*******************************************************************************
// Repeatedly read the sensor angle until it reached a stable value.
template <class T>
float CalibratedSensorSPI<T>::ReadAngle(int* TimeToSettleMS)
{
    float Angle; 
    int Retries=0;
    const int WaitTime=5;       // Time between rereading the sensor value in ms.
    int StableReadings=0;
    const float Precision=_2PI/(float)SensorCPR; 
    const int Oversampling = 16;

    int MinStableReadings=3;   	
    int MaxRepetitions=100;         
    if (CalibrationCoefficients)
    {
        // Use faster settings if do not need to recalibrate! 
        MinStableReadings = 1;
        MaxRepetitions = 20;
    }

    T::update();
    float LastAngle=GetAngleOversampled(Oversampling);
    Angle = GetAngleOversampled(Oversampling);
    while ((StableReadings<MinStableReadings && fabs(Angle-LastAngle)>Precision) && Retries++<MaxRepetitions)
    {
        _delay(WaitTime);
        LastAngle= Angle;
        Angle = GetAngleOversampled(Oversampling);
        if (fabs(Angle-LastAngle)<=Precision)
            StableReadings++;
        else
            StableReadings=0;
    }

    if (Retries>=MaxRepetitions)
        Serial.println("CalibratedSensorSPI<T>::ReadAngle(): Exceeded max number of retries");
    if (TimeToSettleMS) 
        *TimeToSettleMS = Retries*WaitTime;
    return Angle;
}


//*******************************************************************************
// Repeatedly read the sensor angle until it reached a stable value.
template <class T>
float CalibratedSensorSPI<T>::ReadRawAngle()
{
    float Angle; 
    int Retries=0;
    const int WaitTime=5;       // Time between rereading the sensor value in ms.
    int StableReadings=0;
    const float Precision=_2PI/(float)SensorCPR; 
    const int Oversampling = 8;

    int MinStableReadings=3;   	
    int MaxRepetitions=100;         
    if (CalibrationCoefficients)
    {
        // Use faster settings if do not need to recalibrate! 
        MinStableReadings = 1;
        MaxRepetitions = 10;
    }

    T::update();
    float LastAngle=T::getAngle();
    _delay(2);
    T::update();
    Angle = GetRawAngleOversampled(Oversampling);
    while ((StableReadings<MinStableReadings && fabs(Angle-LastAngle)>Precision) && Retries++<MaxRepetitions)
    {
        _delay(WaitTime);
        LastAngle= Angle;
        Angle = GetRawAngleOversampled(Oversampling);
        if (fabs(Angle-LastAngle)<=Precision)
            StableReadings++;
        else
            StableReadings=0;
    }
    return Angle;
}

//*************************************************************************************************************
// Move the motor to the next physical, not calibrated zero position of the sensor.
// pInitialAngle returns the physical, potentialy calibrated angle at which the raw sensor count is zero.
// pInitialElectricalAngle returns the electrical angle corresponding to pInitialAngle.
template <class T>
bool CalibratedSensorSPI<T>::MoveToSensorZero(const int StartDir, float *pInitialAngle, float *pInitialElectricalAngle)
{
    const int Pp=pMotor->pole_pairs;
    const float SensorStepInRad = _2PI/SensorCPR;
    const float ElAnglePerSensorStep = _2PI*Pp/(float)SensorCPR;
    const float UAlign = pMotor->voltage_sensor_align;

    float ElectricalAngle = 0.0;

    float Step = 0;
    float StepSize = 100;
    float SearchDir = StartDir;
    float RawPos;
    float PosInSteps;


    // Move quickly to somewhere just larger than step 0.
    // Electrical angle and sensor position must increase in the same direction!
    // Serial.println("Coarse search for sensor zero position");
    while (true)
    {
        Step+=(StepSize*SearchDir);
        ElectricalAngle = Step * ElAnglePerSensorStep;
        pMotor->setPhaseVoltage(UAlign/2.0, 0, ElectricalAngle);
        RawPos = ReadRawAngle();
        PosInSteps = RawPos/SensorStepInRad;

        if (PosInSteps<100.0f || PosInSteps>SensorCPR-100)
            break;

        if (PosInSteps<SensorCPR/2.0f)
            SearchDir = -1.0; 
        else
            SearchDir = 1.0; 

        // Decimation of step size
        if (PosInSteps>200 && PosInSteps<SensorCPR-200)
            StepSize = 100;
        else 
            StepSize = 10;
    }

    // Fine search around the zero position
    // To avoid problems with wrap around, we shift the scale.
    // Serial.println("Fine search for sensor zero position");
    while (true)
    {
        Step+=(StepSize*SearchDir);
        ElectricalAngle = Step * ElAnglePerSensorStep;
        pMotor->setPhaseVoltage(UAlign/2.0, 0, ElectricalAngle);
        RawPos = ReadRawAngle();
        PosInSteps = RawPos/SensorStepInRad;
        if (PosInSteps>SensorCPR/2) PosInSteps-=SensorCPR;
        // Serial.printf("   %.1f", PosInSteps);

        if (PosInSteps<0.5f && PosInSteps>-0.5f)
            break;

        if (PosInSteps>0)
            SearchDir = -1.0; 
        else
            SearchDir = 1.0; 

        // Decimation of step size
        if (PosInSteps>200 || PosInSteps<-200)
            StepSize = 100;
        else if (PosInSteps>20 || PosInSteps<-20)
            StepSize = 10;
        else if (PosInSteps>5 || PosInSteps<-5)
            StepSize = 1;
        else StepSize = 0.25;
    }


    *pInitialAngle = ReadAngle();
    *pInitialElectricalAngle = ElectricalAngle;
    float InitialError = fmod(*pInitialAngle, _2PI);
    if (InitialError> 3) InitialError = _2PI-InitialError;
    Serial.printf("      Initial raw sensor angle = %.5f\n", RawPos);

    return true;
}


// Spin NumRevolutions revolutions first to release possible tension on the string.
//   NumRevolutions >0 =cw, NumRevolutions<0 =ccw
template <class T>
void CalibratedSensorSPI<T>::SpinMotor(float NumRevolutions)
{
    int Direction = (NumRevolutions>0 ? 1 : -1);
    NumRevolutions *= Direction;
    Serial.printf("Spinning app. %.1f revolutions %s\n", NumRevolutions, Direction>0? "cw" : "ccw");
    int NumSteps = 8 * 16 * pMotor->pole_pairs;    // Number of steps per revolution
    const float DeltaElectricalAngle = _2PI*pMotor->pole_pairs/NumSteps;      
    Serial.printf("   %d steps per rev., el. angle increment = %.5f\n", NumSteps, DeltaElectricalAngle);
    NumSteps *= NumRevolutions;

    float StartAngle=ReadAngle();
    for(int i=0; i<NumSteps; i++)
    {
        float Angle = i*(Direction) * DeltaElectricalAngle;
        pMotor->setPhaseVoltage(pMotor->voltage_sensor_align, 0, Angle+StartAngle);
        delay(1);
    }
}


// Spin NumRevolutions revolutions first to release possible tension on the string.
//   At each step check the sensor state. At the end a summary is of errors is printed.
//   NumRevolutions >0 =cw, NumRevolutions<0 =ccw
template <class T>
int CalibratedSensorSPI<T>::SpinMotorWithCheck(float NumRevolutions)
{
    int Direction = (NumRevolutions>0 ? 1 : -1);
    NumRevolutions *= Direction;
    Serial.printf("Spinning app. %.1f revolutions %s with sensor check, ", NumRevolutions, Direction>0? "cw " : "ccw");
    int NumSteps = 8*16 * pMotor->pole_pairs;    // Number of steps per revolution
    const float DeltaElectricalAngle = _2PI*pMotor->pole_pairs/NumSteps;      
    // Serial.printf("   %d steps per rev., el. angle increment = %.5f\n", NumSteps, DeltaElectricalAngle);
    NumSteps *= NumRevolutions;

    int ErrorCount=0;
    int SensorReads=0;
    float StartAngle=ReadAngle();
    for(int i=0; i<NumSteps; i++)
    {
        float Angle = i*(Direction) * DeltaElectricalAngle;
        T::getAngle();
        if (i%16==0)   delay(30);
        pMotor->setPhaseVoltage(pMotor->voltage_sensor_align, 0, Angle+StartAngle);
        SensorReads++;
        if (SENSOR_ERROR_NOT_SET!=T::CheckSensor())
            ErrorCount++;
        delay(1);
    }
    Serial.printf("%d Sensor errors for %d sensor reads\n", ErrorCount, SensorReads);
    return ErrorCount;
}


// Stop if p is a null pointer.
#define CheckMemAlloc(p)    if (!p) { Serial.println("Error: Out of memory (" #p ")");  while(1); }



//*******************************************************************************
// Test the sensor, verify compensation effect or calculate sensor misalignment
// compensation data.
template <class T>
bool CalibratedSensorSPI<T>::RunPositionTest(const int StartDir, const int _Turns, const bool Quiet, bool Validate, const bool Calibrate)
{
    if (Validate)
        Serial.printf("Sensor validation\n");
    else if (Calibrate)
        Serial.printf("Sensor calibration\n");
    else
        Serial.printf("Sensor check\n");

    bool rc = true;
    TurnsPerDirection = _Turns;

    int SamplesPerPolePair = 64;
    
    const int Pp=pMotor->pole_pairs;
    const float SensorStepInRad = _2PI/SensorCPR;
    const float ElAnglePerSensorStep = _2PI*Pp/(float)SensorCPR;
    const float UAlign = pMotor->voltage_sensor_align;

    // Set voltage angle to zero, wait for rotor position to settle
    // keep the motor in position while getting the initial positions
    pMotor->setPhaseVoltage(pMotor->voltage_sensor_align, 0, 0);
    _delay(300);

    // If a calibration table was provided, we only run the test with the same number of samples as the
    // the table had, but we do not create a new table!
    // NumSamples MUST be a multiple of the pole pairs -> Filtering!!
    if (!NumSamples || Validate)
        NumSamples = SamplesPerPolePair*Pp;   
    else Serial.println("   Sensor calibration data provided");

    int ElStepsPerSample = 16;
    int SamplesPerRevolution = NumSamples;

    // For a plain check, without generation of calibration data, faster params can be chosen:
    if(!Calibrate && !Validate)
    {
        NumSamples = SamplesPerPolePair*Pp; ;
        ElStepsPerSample = ElStepsPerSample * 4; //  * 16;
        SamplesPerRevolution = NumSamples / 16;
    }

    // One mechanical revolution equals pMotor->pole_pairs electrical revolutions!
    float deltaElectricalAngle = _2PI*pMotor->pole_pairs/(SamplesPerRevolution*ElStepsPerSample);      // Electrical Angle increments for calibration steps    
    float elec_angle = 0.0;
    int TimeToSettle;

    MinError = 0;
    MaxError=0;

    // Move to a known starting position
    pMotor->setPhaseVoltage(UAlign, 0.0f, 0.0f);

  
    float AbsoluteSensorAngle;
    float AbsoluteElectricalAngle;
    float Error;
    float MinMag = 1e5f;
    float MaxMag = 0.0f;
    
    float AbsoluteSensorAngleStartDir;
    float AbsoluteElectricalAngleStartDir;
    float ErrStartDir;
    float TimeToSettleMean=0;

    float* pDeviations = new float[2*SamplesPerRevolution];      // First half cw, second ccw (if StartDir>=0)
    CheckMemAlloc(pDeviations);
    for (int i=0; i<2*SamplesPerRevolution; i++)
        pDeviations[i] = 0.0f;

    // If new calibration data are to be collected, start with sensor angle close to 0.0.
    float InitialAngle = 0.0;
    float InitialElectricalAngle = 0.0;

    // Move the motor to the next physical, not calibrated zero position of the sensor.
    MoveToSensorZero(StartDir, &InitialAngle, &InitialElectricalAngle);

    Serial.printf("   Initial sensor angle                = %.2f\n", InitialAngle);
    Serial.printf("   Initial electrical angle            = %.2f\n", InitialElectricalAngle);
    Serial.printf("   Taking %d samples, delta angle el. = %.5f rad (%.5f deg)\n", SamplesPerRevolution, deltaElectricalAngle, RAD2DEG(deltaElectricalAngle));
    Serial.printf("   Sensor resolution                   = %.5f rad (%.5f deg)\n", SensorStepInRad, RAD2DEG(SensorStepInRad));
    Serial.printf("   El. angle per sensor step           = %.5f rad\n", ElAnglePerSensorStep);

    //---------------------------------------------------------------------
    // Collect misalignments of positions for <Turns> forward and backward
    if (StartDir>0)
        Serial.printf("   Rotating %d turns cw then ccw  ", TurnsPerDirection);
    else
        Serial.printf("   Rotating %d turns ccw then cw  ", TurnsPerDirection);

    float MeanErrCw =0;
    float MeanErrCCw=0;

    // The actual test...
    int Direction = StartDir;
    int FullRevs = 0;
    const float DeltaElectricalAngle = deltaElectricalAngle;
    float ElectricalAngle;
    
    // Serial.printf("\n%d turns per direction, %d samples per revolution, %d steps in between\n", TurnsPerDirection, SamplesPerRevolution, ElStepsPerSample);
    int NumErrors = 0; 
    int NumChecks = 0;
    int Turns = TurnsPerDirection * 2;
    for (int r=0; r<Turns && rc; r++)
    {
        for(int i = 0; i<SamplesPerRevolution && rc; i++)
        {
            for(int j = 0; j<ElStepsPerSample && rc; j++)
            {   
                int AngleIndex;
                if (Direction>0)
                    AngleIndex = (FullRevs*SamplesPerRevolution*ElStepsPerSample + i*ElStepsPerSample + j);
                else
                    AngleIndex = (FullRevs*SamplesPerRevolution*ElStepsPerSample - i*ElStepsPerSample - j);
                ElectricalAngle = AngleIndex * DeltaElectricalAngle;
                pMotor->setPhaseVoltage(UAlign, 0, ElectricalAngle+InitialElectricalAngle);
                // Serial.printf("Dir:%d, revs:%d, i:%d, j:%d, AngleIndex: %d, Angle(+InitialElectricalAngle)=%.5f\n", Direction, FullRevs, i, j, AngleIndex, ElectricalAngle+InitialElectricalAngle);
                delay(1);
            }

            // Delay to settle in position before taking a position sample
            _delay(3);

            AbsoluteSensorAngle = ReadAngle(&TimeToSettle) - InitialAngle;
            AbsoluteElectricalAngle = ElectricalAngle/(float)Pp;
            rc = CheckAngle(AbsoluteSensorAngle, AbsoluteElectricalAngle, _2PI);
            if (rc)
            {
                Error = AbsoluteElectricalAngle-AbsoluteSensorAngle;
                TimeToSettleMean += TimeToSettle;

                if (Error>MaxError) MaxError=Error;
                if (Error<MinError) MinError=Error;
 
                int Index;
                if (Direction>0)
                {
                    MeanErrCw +=Error;
                    Index = i;
                }
                else
                {
                    MeanErrCCw +=Error;
                    Index = 2*SamplesPerRevolution-i-1;
                }

                pDeviations[Index]+=(Error/(float)TurnsPerDirection);
                if (isnan(Error) || Error<-_PI || Error>_PI)
                {
                    rc = false;
                    Serial.printf("\n   Invalid error value (A) detected (%.5f)!\n", Error);
                }
                else if (isnan(pDeviations[Index]) || pDeviations[Index]<-_PI || pDeviations[Index]>_PI)
                {
                    rc = false;
                    Serial.printf("\n   Invalid sensor deviation (A) detected (%.5f, t=%d, i=%d, Index=%d, Error=%.5f)!\n", pDeviations[Index], r, i, Index, Error);
                }
                else 
                {
                    NumChecks++;
                    int SensError = T::CheckSensor(&MinMag, &MaxMag);
                    if (SensError!=SENSOR_ERROR_NOT_SET)
                    {
                        SensorErrorCode = SensError;
                        NumErrors++;
                        // rc = false;
                        Serial.printf("   CheckSensor() returned an error (%d)!", SensError);
                    }
                }
            }
        }
        FullRevs += Direction;
        Serial.printf(" %s", Direction>0 ? "cw" : "ccw");
        Direction *= -1;
    }

    // Switch the phase voltages off!
    pMotor->setPhaseVoltage(0.0f, 0, elec_angle+InitialElectricalAngle);

    Serial.println("");
    Serial.printf("   %d out of %d sensor checks failed\n", NumErrors, NumChecks);

    if (rc)
    {
        MeanErrCw /= ((float)SamplesPerRevolution*TurnsPerDirection);
        MeanErrCCw /= ((float)SamplesPerRevolution*TurnsPerDirection);

        // Serial.printf("   Error after %d rounds was:\n%.3f\t%.3f\t%.3f\n", Turns, AbsoluteElectricalAngleStartDir, AbsoluteSensorAngleStartDir, ErrStartDir);
        Serial.printf("   MaxError=%.3f, MinError=%.3f\n", MaxError, MinError);
        Serial.printf("   MeanErrCw=%.3f, MeanErrCCw=%.3f\n", MeanErrCw, MeanErrCCw);

        MinAngleError = min(MinAngleError, MinError);
        MaxAngleError = max(MaxAngleError, MaxError);

        if ( (MinAngleError<-AbsErrorThreshold && MaxAngleError>AbsErrorThreshold) || isnan(MinAngleError) || isnan(MaxAngleError))
        {
            Serial.println("   Error: encoder position error too high!");
            SensorErrorCode = SENSOR_ERROR_POS_ERROR_TOO_HIGH;
            rc = false;
        }

        if (T::DiagnosticsAvailable)
            Serial.printf("   Longterm:  MaxMagneticMagnitude=%.0f, MinMagneticMagnitude=%.0f, DeltaMagnitude=%.0f\n", MaxMag, MinMag, MaxMag-MinMag);
        
        if (MeanErrCw>MAX_MEAN_ERROR || MeanErrCCw>MAX_MEAN_ERROR || isnan(MeanErrCw) || isnan(MeanErrCCw))
        {
            Serial.println("   Error: Mean position error >> 0! magnet loose?");
            SensorErrorCode = SENSOR_ERROR_MEAN_ERR_NOT_0;
            rc = false;
        }
    }
    else Serial.println("   Error: CalibratedSensorSPI<T>::RunPositionTest(), Invalid sensor readings");


    //-----------------------------------------------------------------
    // Prepare table with correction factors

    if (rc && Calibrate)
    {
        Serial.println("   Sensor calibration creating compensation coefficients");
        if (CalibrationCoefficients && !CalibrationDataExternal) delete[] CalibrationCoefficients;
        CalibrationCoefficients = new float[SamplesPerRevolution];
        CheckMemAlloc(CalibrationCoefficients);
        CalibrationDataExternal = false;
        NumSamples = SamplesPerRevolution;

        Validate = false;   // Validation will not work immediately after calculating new coefficients!

        // Here we combine cw and ccw samples and invert the direction. 
        for (int i=0; i<SamplesPerRevolution && rc; i++)
        {
            pDeviations[i] = 0.5* (pDeviations[i]-MeanErrCw + pDeviations[i+SamplesPerRevolution]-MeanErrCCw);
            if (pDeviations[i]<-_PI || pDeviations[i]>_PI || isnan(pDeviations[i]))
            {
                rc = false;
                Serial.printf("   Error: invalid position error detected (%.5f)!", pDeviations[i]);
            }
        }

        if (rc)
        {

            // Filter the errors contained in pDeviations (the first SamplesPerRevolution in one direction and the next SamplesPerRevolution in reverse direction).
            // Use FIR filter according to this: https://build-its-inprogress.blogspot.com/2017/03/encoder-autocalibration-for-brushless.html
            // It is actually just a moving average...
            FIRFilter(pDeviations, CalibrationCoefficients, SamplesPerRevolution, SamplesPerPolePair);
            
            // Finally, print the table
            PrintArray(SamplesPerRevolution, CalibrationCoefficients, "CalibrationCoefficients");
        }
    }

    //-----------------------------------------------------------------------
    // If requested, validate how well the sensor misalignment is compensated
    if (rc && Validate)
    {
        Serial.println("   Sensor calibration validation");
        float MeanErr=0;
        float MaxAbsErr=0;
        float MeanErrF=0;
        float MaxAbsErrF=0;
        for (int i=0; i<SamplesPerRevolution && rc; i++)
        {
            // Combine cw and ccw samples! 
            pDeviations[i] = 0.5* (pDeviations[i]-MeanErrCw + pDeviations[i+SamplesPerRevolution]-MeanErrCCw);
            if (pDeviations[i]<-_PI || pDeviations[i]>_PI || isnan(pDeviations[i]))
            {
                rc = false;
                Serial.printf("   Error: invalid position error detected (%.5f)!\n", pDeviations[i]);
            }
            else
            {
                MeanErr += pDeviations[i];
                float AbsErr = fabs(pDeviations[i]);
                MaxAbsErr = max(MaxAbsErr, AbsErr);
            }
        }
        MeanErr /= SamplesPerRevolution;

        if (rc)
        {
            // Reuse memory which is not required anymore...
            float* FilteredErrors = pDeviations + SamplesPerRevolution;

            FIRFilter(pDeviations, FilteredErrors, SamplesPerRevolution, SamplesPerPolePair);

            for (int i=0; i<SamplesPerRevolution && rc; i++)
            {
                if (isnan(FilteredErrors[i]) || FilteredErrors[i]<-_2PI || FilteredErrors[i]>_2PI)
                {
                    rc = false;
                    Serial.printf("   Error: invalid filtered position error detected (%.5f)!\n", FilteredErrors[i]);
                }
                MeanErrF += FilteredErrors[i];
                float AbsErr = fabs(FilteredErrors[i]);
                MaxAbsErrF = max(MaxAbsErrF, AbsErr);
            }
            MeanErrF /= SamplesPerRevolution;

            if (rc)
            {
                Serial.println("   ************ Sensor calibration validation, residual errors: ************");
                if (!Quiet)
                {
                    Serial.println("Angle;Error;FilteredError");
                    for (int i=0; i<SamplesPerRevolution; i++)
                        Serial.printf("%.4f;%.4f;%4f\n", (float)i * _2PI/(float)SamplesPerRevolution, pDeviations[i], FilteredErrors[i]);
                }
                Serial.println("   All errors below are mean free!");
                Serial.printf("   MeanErr   = %.4f\n", MeanErr);
                Serial.printf("   MaxAbsErr = %.4f\n", MaxAbsErr);
                Serial.printf("   MeanErrFiltered   = %.4f rad (%.4f deg)\n", MeanErrF, RAD2DEG(MeanErrF));
                Serial.printf("   MaxAbsErrFiltered = %.4f rad (%.4f deg)\n", MaxAbsErrF, RAD2DEG(MaxAbsErrF));
                Serial.println("   ******************************************");
            }
        }
    }

    delete[] pDeviations;

    return rc;
}


