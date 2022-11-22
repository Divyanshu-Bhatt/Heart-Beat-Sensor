#include "arduinoFFT.h"
#include "Metro.h"

const uint16_t samples = 128;
const double samplingFrequency = 16;
const int delay_ms = 1000/samplingFrequency;

Metro sampler = Metro(delay_ms);

//Low pass Frequency
const uint16_t LPF = (0.5/samplingFrequency)*samples;
//High pass Frequency
const uint16_t HPF = (3.0/samplingFrequency)*samples;

double vReal[samples];
double vImag[samples];
arduinoFFT FFT;

void setup() {
  // Rate Arduino-laptop communicate
  Serial.begin(115200); 
  pinMode(A0,INPUT);

  FFT = arduinoFFT();
}

void loop() {
  Serial.println("Measuring:-");
  int i = 0;
  while(i < samples) {
    if(sampler.check() == 1) 
    {
      if(i%16 == 0) Serial.println(".");
    
      vReal[i] = analogRead(A0)/1024.0;
      vImag[i] = 0;
      i++;
    }
  }
  Serial.println();

  double time_delay;
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples);
  time_delay = TimeDelay(vReal, LPF, HPF);
  
  //Finding peak
  uint16_t peakFreq = LPF;
  double peakMag = vReal[LPF];

  for(int i = LPF+1; i < HPF; i++) 
  {
    if(vReal[i] > peakMag) 
    {
      peakMag = vReal[i];
      peakFreq = i;
    }
  }

  double heart_rate = (60.0*samplingFrequency*peakFreq)/samples;

  Serial.print("Heart rate: ");
  Serial.println(heart_rate, 3);
  
  // T_d = HR in milliseconds - timedelay in milli seconds
  double T_d = ((60000/heart_rate) - (time_delay*1000));
  
  // Formula for Blood Pressure
  double SBP = (184.3 - 1.329*heart_rate + 0.0848*T_d);
  double DBP = (55.96 - 0.02912*heart_rate + 0.02302*T_d);

  Serial.print("   Blood Pressure: ");
  Serial.println(SBP, 3);
  Serial.print("Diastolic Blood Pressure: ");
  Serial.println(DBP, 3);
  Serial.println("-----------------------");
}

// Calculate Time Delay in seconds
double TimeDelay(double *vData, uint16_t start, uint16_t end)
{ 
  uint16_t i_max = start;
  double abscissa_max = vData[start];
  uint16_t i_min = start;
  double abscissa_min = vData[start];

  double abscissa_array[end - start];

  for (uint16_t i = start; i < end; i++)
  {
    double abscissa = (i * 1.0 * samplingFrequency) / samples;

    if (vData[i] > vData[i_max])
    {
      abscissa_max = abscissa;
      i_max = i;
    }
    abscissa_array[i-4] = abscissa;
  }

  for (int i = start; i < i_max; i++)
  {
    if (vData[i] < vData[i_min])
    {
      i_min = i;
      abscissa_min = abscissa_array[i-4];
    }
  }
  return abscissa_max - abscissa_min;
}