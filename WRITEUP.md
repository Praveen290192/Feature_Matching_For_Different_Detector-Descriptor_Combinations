#### MP0
* Here is the writeup.md file
#### MP1
* Line 85 in MidTermProject_Camera_Student.cpp
#### MP2
* Line 42 in MidTermProject_Camera_Student.cpp. All different keypoints detector list that been used.
#### MP3
* Line 123 in MidTermProject_Camera_Student.cpp
#### MP4
* Line 46 in MidTermProject_Camera_Student.cpp. All different keypoints descriptor list that been used.
#### MP5
* Line 30 in matching2D_Student.cpp.
#### MP6
* Line 46 in matching2D_Student.cpp.
#### MP7
* build/detector.csv contains number of keypoints detected and time required to detect those points for different detectors
#### MP8
* build/task8_task9.csv contains averag total time for detection/ average number of matches for different combinations of detectors and descriptors. Using BF method at descriptor distance ratio of 0.8
#### MP9
* After checking different combinations, best 3 combinations are as below
* FAST Detector with BRIEF/ORB/BRISK provides best overall performance since it gives more avgMatches in less time below 3.17ms for 3 different Descriptor. 
* Speed of processing will be important criteria I would consider and next thing will be important is Number of matches.
![Detector_Descriptor](https://user-images.githubusercontent.com/46629608/121829565-e7212c80-cc90-11eb-8da6-d128f9658a86.PNG)
