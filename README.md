# Instrumented-Mouthguard-Software-Development
This GitHub repository contains the STM32CubeIDE and MATLAB code developed for the instrumented mouthguard.

** Notes on code in the repository:
- Only one STM32CubeIDE code was developed and used for both the mandibular and maxillary components of the device. Changes were made within the code for sensor calibration and axis alighnment between the pair of mouthguards. The STM32CubeIDE code contained in this repository corresponds to that for the mandibular component.
- The MATLAB code under the file "Motion Analysis" contains the code written to perform the frequency analyses in Section 5.2.3 of the report.
- The MATLAB code under the file "Data Processing" contains the code developed to process the measurements transmitted by the instrumented mouthguard.
