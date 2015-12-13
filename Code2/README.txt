I worked alone on this problem set. 

List of all files you submitted:

asst8.cpp, 
shaders/normal-gl3.fshader
rigtform.h

Note the platform you used for development (Windows, OS X, ...):

Ubuntu 14.04

Provide instructions on how to compile and run your code, especially if you used a nonstandard Makefile, or you are one of those hackers who insists on doing things differently.

Using g++ directly with the command: 
g++ asst8.cpp geometry.cpp picker.cpp glsupport.cpp ppm.cpp material.cpp renderstates.cpp texture.cpp scenegraph.cpp -o asst -lGL -lGLU -lglut -lGLEW

Indicate if you met all problem set requirements (more importantly, let us know where your bugs are and what you did to try to eliminate the bugs; we want to give you as much partial credit as we can).

I believe I met all pset requirements! 

Provide some overview of the code design. Don't go into details; just give us the big picture.

My code for lerping + slerping is in rigtform.h. I then ported the code from the asst6_5-snippets.cpp. I then added physical lights to the scence; the geometry for these lights are stored as g_spheres, and the lights' coordinates are stored in a shared_ptr/Transform node. I then modified the shader as directed. 
 
Finally, did you implement anything above and beyond the problem set? If so, document it in order for the TFs to test it and evaluate it.

No
