# ORBSLAM2 - Release notes v1.1

# 1. Changes

Adding the following API calls to the wrapper:
```
void * initSLAM(char *strVocFile, char *strSettingsFile, int sensor, int saveMap);

void closeSLAM(void *System);

float * runSLAM(void *System, void *im, void *depth, int width, int height, double timestamp);

int statusSLAM(void *System);

int mapChangedSLAM(void *System);
```

# 2. initSLAM

This function initialize the ORBSLAM2 algorithm. It needs a vocabulary file `strVocFile` (in .fbow format), a settings file `strSettingsFile` for both the hardware and the algorithm parameters, an integer that describes the `sensor` modality (0 for Monocular, 1 for Stereo, 2 for RGBD) and a `saveMap` integer to tell the algorithm to save the map after closing the SLAM or not (it can be 0 or 1). It returns a class of type `System`.

# 3. closeSLAM

This function closes the ORBSLAM2, deallocating all the memory, calling all the destructors and saving map if this option has been specified in the initSLAM API call. In order to call this function a pointer to a class of type `System` must be given as input.

# 4. runSLAM

This function tells the ORBSLAM2 system to run. It needs a pointer to a class of type `System`, a pointer to both an infrared and depth images, the frames width and height and a timestamp of the frames. The function returns an array of floats with [x, y, z] position and a quaternion [qw, qx, qy, qz] for the orientation (the values are concat so it is a 7 elements array).

# 5. statusSLAM

This function checks the status of the tracker in ORBSLAM2 algorithm. It accepts as an input a class of type `System`. It returns an int with a value for the tracker state: -1 (system not ready), 0 (no images yet), 1 (not initialized), 2 (ok), 3 (track lost).

# 6. mapChangedSLAM

This function checks the status of the loop closure ORBSLAM2 algorithm. It accepts as an input a class of type `System`. It returns an int with 0 if no loop closure has been made from the last call to this function, 1 otherwise.