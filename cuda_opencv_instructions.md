1. Install the latest version of Visual Studio build tools https://www.visualstudio.com/downloads/#build-tools-for-visual-studio-2017 
	Select to install "Visual C++ build tools", allow default options within this.
2. Install the latest CMake binary release win64-x64
3. Download and install CUDA https://developer.nvidia.com/cuda-zone
	Choose custom isntallation, uncheck Visual Studio Integration (Only if you installed the build tools alone, instead of w/ the 
	IDE as described in these instructions.)
4. Download the latest cuDNN libraries https://developer.nvidia.com/rdp/cudnn-download (will require making an account)
	Follow these instructions to install cuDNN https://docs.nvidia.com/deeplearning/sdk/cudnn-install/index.html
5. Download and install the latest Python3 release for Windows https://www.python.org/downloads/windows/
	Make sure it is added to your path.
	Make a virtual environment, if you prefer
	Install numpy `pip3 install numpy`
6. Download the latest stable OpenCV Source documents https://opencv.org/releases.html
	Extract the ZIP file to opencv-master directory within an opencv folder
7. Download the latest opencv_contrib source files https://github.com/opencv/opencv_contrib/tree/master/modules
	extract the ZIP file to an opencv_contrib-master directory within the same opencv folder
8. Now we build:
	Make a new directory called build within the opencv folder
	Open the "Developer Command Prompt for VS 2017" and navigate to the build directory you just made
	run this
		cmake ..\opencv-master\ -G"Visual Studio 15 2017 Win64" -DWITH_CUDA=ON -DCUDA_FAST_MATH=ON -DWITH_CUBLAS=ON -DBUILD_PYTHON_SUPPORT=ON -DBUILD_opencv_python3=ON -DBUILD_opencv_cudacodec=OFF -DINSTALL_TESTS=ON -DINSTALL_C_EXAMPLES=ON -DBUILD_EXAMPLES=ON -DOPENCV_EXTRA_MODULES_PATH=..\opencv_contrib-master\modules
	now run this, changing CL_MPCount to the number of cores on your PC. 
		msbuild /p:CL_MPCount=8 /p:Configuration=Release OpenCV.sln
		This will take awhile. The parallel building doesn't happen when compiling the CUDA stuff, which is the bulk of the time :(
9. Setup the package w/ Python
	Load your virtual environment
	run the following, filling in the right path for PATH_TO_OPPENCV_BUILD
		python -m pip install PATH_TO_OPENCV_BUILD\python_loader

You should now be able to run your scripts.