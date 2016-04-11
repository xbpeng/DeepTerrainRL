--
-- premake4 file to build TerrainRL_Optimizer
-- Copyright (c) 2009-2015 Glen Berseth
-- See license.txt for complete license.
--

local linuxLibraryLoc = "../external/"
local windowsLibraryLoc = "../library/"

project "TerrainRL_Optimizer"
	language "C++"
	kind "WindowedApp"

	files { 
		-- Source files for this project
		"../learning/*.h",
		"../learning/*.cpp",
		"../scenarios/*.h",
		"../scenarios/*.cpp",
		"../sim/*.h",
		"../sim/*.cpp",
		"../util/*.h",
		"../util/*.cpp",
		"../anim/*.h",
		"../anim/*.cpp",
		"Main.cpp",
		"./opt/*.h",
		"./opt/*.cpp",
		"./opt/*.c",
		"./scenarios/*.h",
		"./scenarios/*.cpp",

	}
	excludes {
		"../scenarios/Draw*.h",
		"../scenarios/Draw*.cpp",
		"../sim/CharTracer.cpp"
	}

	includedirs { 
		"./",
		"../"
	}

	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
		"CPU_ONLY",
		"GOOGLE_GLOG_DLL_DECL=",
		"ENABLE_TRAINING",
	}

	targetdir "../"
	buildoptions("-std=c++0x -ggdb -g" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }

		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		libdirs { 
			"lib",
			linuxLibraryLoc .. "Bullet/bin",
			linuxLibraryLoc .. "jsoncpp/build/debug/src/lib_json",
			linuxLibraryLoc .. "caffe/build/lib",
		}
		
		includedirs { 
			linuxLibraryLoc .. "Bullet/src",
			linuxLibraryLoc,
			linuxLibraryLoc .. "jsoncpp/include",
			linuxLibraryLoc .. "caffe/include/",
			linuxLibraryLoc .. "caffe/build/src/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			linuxLibraryLoc .. "3rdparty/include/hdf5",
			linuxLibraryLoc .. "3rdparty/include/",
			linuxLibraryLoc .. "3rdparty/include/openblas",
			linuxLibraryLoc .. "3rdparty/include/lmdb",
			"/usr/local/cuda/include/",
			linuxLibraryLoc .. "OpenCV/include",
			linuxLibraryLoc .. "caffe/src/",
			linuxLibraryLoc .. "CMA-ESpp/cma-es",
		}
		defines {
			"_LINUX_",
		}

		configuration "Debug*"
			defines { 
				"_DEBUG",
				"ENABLE_DEBUG_PRINT",
				"ENABLE_DEBUG_VISUALIZATION"
			}
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_debug",
				"BulletCollision_gmake_x64_debug",
				"LinearMath_gmake_x64_debug",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				"hdf5",
				"hdf5_hl",
				"f2c",
			}
	 
	 	-- release configs
		configuration "Release*"
			defines { "NDEBUG" }
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_release",
				"BulletCollision_gmake_x64_release",
				"LinearMath_gmake_x64_release",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				"hdf5",
				"hdf5_hl",
				"f2c",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		includedirs { 
			windowsLibraryLoc .. "Bullet/include",
			windowsLibraryLoc,
			windowsLibraryLoc .. "Json_cpp",
			windowsLibraryLoc .. "caffe/include/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			windowsLibraryLoc .. "caffe/3rdparty/include/hdf5",
			windowsLibraryLoc .. "caffe/3rdparty/include/",
			windowsLibraryLoc .. "caffe/3rdparty/include/openblas",
			windowsLibraryLoc .. "caffe/3rdparty/include/lmdb",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/include/",
			windowsLibraryLoc .. "OpenCV/include",
			windowsLibraryLoc .. "caffe/src/",
		}	
		links { 
			"opengl32",
			"glu32",
			-- Just a few dependancies....
			"BulletDynamics_Debug",
			"BulletCollision_Debug",
			"LinearMath_Debug",
			"jsoncpp_Debug",
			"opencv_core300d",
			"opencv_calib3d300d",
			"opencv_flann300d",
			"opencv_highgui300d",
			"opencv_imgproc300d",
			"opencv_imgcodecs300d",
			"opencv_ml300d",
			"opencv_objdetect300d",
			"opencv_photo300d",
			"opencv_features2d300d",
			"opencv_stitching300d",
			"opencv_video300d",
			"opencv_videostab300d",
			"opencv_hal300d",
			"libjpegd",
			"libjasperd",
			"libpngd",
			"IlmImfd",
			"libtiffd",
			"libwebpd",
			"cudart",
			"cuda",
			"nppi",
			"cufft",
			"cublas",
			"curand",
			"gflagsd",
			"libglogd",
			"libprotobufd",
			"libprotocd",
			"leveldbd",
			"lmdbd",
			"libhdf5_D",
			"libhdf5_hl_D",
			"Shlwapi",
			"zlibd",
			"libopenblas"
		}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"Cocoa.framework",
			"dl",
			"pthread"
		}


