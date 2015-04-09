# GitShare
STM32 stuff

Code is a complete mess. Libaries have been modified and even fatfs has been changed for speed. For performance reasons the peripherals have been used. Main example is how FSMC and DMA transfers the data straight from the sd card to the display. I didn't think it was possible for the STM32f103vc to do any decoding, first because it might not be powerful enough and second it doesn't even have the memory to store a decoded frame. But I've seen some people with jpeg? video. It may be that decoding is possible and may be faster with the uSD card being the slowest part and the large files means reading smaller files and decoding them may be much faster, if you can decode part of display at a time, then it could work. 
It also plays sound, or outputs sound signal for a pwm based speaker. 

***Code is a complete mess, with lots of testing and degugging code left in. 
