#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>

#include <System.h>


std::string base_name(std::string const & path)
{
    return path.substr(path.find_last_of("/") + 1);
}

std::string remove_extension(std::string const & filename)
{
    std::size_t p = filename.rfind('.');
    return p > 0 && p != std::string::npos ? filename.substr(0, p) : filename;
}

int main(int argc, char **argv)
{  
    // Video file to do slam on.
    if (argc != 2) {
        std::cout << "Usage from main folder:\n$ ./Examples/Monocular/mono_iphonexr <path to your video file>\nUnexpected number of arguments found. Exiting...\n";
        return 1;
    }
    std::string file_path(argv[1]);
    std::cout << "Working on " << file_path << "\n";
    cv::VideoCapture cap(file_path);

    cout << endl << "-------" << endl;
    cout.precision(17);


    int fps = 20;
    float dT = 1.f/fps;

    std::string orb_vocabulary_path = "./Vocabulary/ORBvoc.txt";
    std::string settings_path = "./Examples/Monocular/IphoneXR.yaml";

    bool enablePanglinVisualization = false;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(orb_vocabulary_path, settings_path, ORB_SLAM3::System::MONOCULAR, enablePanglinVisualization);
    float imageScale = SLAM.GetImageScale();

    std::cout << "Image scale: " << imageScale << std::endl;

    // Main loop
    cv::Mat image;
    int proccIm = 0;
    int frame_count = 0;
    while(1)
    {

        // Read image from file
        cap >> image; 
        double tframe = frame_count * dT;

        if(image.empty())
        {
            cerr << endl << "Failed to load image at, end of video?\n";
            break; // Break out of loop to go to shutdown procedures.
        }

        if(imageScale != 1.f)
        {

            int width = image.cols * imageScale;
            int height = image.rows * imageScale;
            cv::resize(image, image, cv::Size(width, height));
        }

        // Pass the image to the SLAM system
        // cout << "tframe = " << tframe << endl;
        SLAM.TrackMonocular(image, tframe);

        // Wait to load the next frame
        
    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (1)
    {
        const string filename = remove_extension(base_name(file_path));
        const string kf_file =  "kf_" + filename + ".txt";
        const string f_file =  "f_" + filename + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}
