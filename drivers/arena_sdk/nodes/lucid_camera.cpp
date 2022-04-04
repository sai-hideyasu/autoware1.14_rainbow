/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
  This program requires ROS and Flycapture SDK installed
  Author: Abraham Monrroy (amonrroy@ertl.jp)
  Initial version     2014-11-14
  Added signal handler     2015-05-01
  Added CameraInfo msg     2015-05-01
 */

#include <ArenaApi.h>
#include <SaveApi.h>

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <autoware_config_msgs/ConfigArenaSDK.h>

#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <signal.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <string>

#define FILE_NAME				"polarize.raw"
#define TAB1					"  "
#define PIXEL_FORMAT			BGR8
#define RESIZE_WIDTH			640.0
#define RESIZE_HEIGHT			480.0


using namespace Arena;

enum {
	IMAGE_ALL = 0,
	IMAGE_DEG0,
	IMAGE_DEG45,
	IMAGE_DEG90,
	IMAGE_DEG135,
	IMAGE_UNKNOWN
};

static int degree_;
static double gain_;
static bool save_img_;

static volatile int running = 1;

static void signal_handler(int)
{
	running = 0;
	ros::shutdown();
}

static void start_streams(std::vector<IDevice *>& devices)
{
	for (auto *device : devices)
	{
		device->StartStream();
	}
}

static void stop_streams(std::vector<IDevice *>& devices)
{
	for (auto *device : devices)
	{
		device->StopStream();
	}
}

static void destroy_devices(ISystem *pSystem, std::vector<IDevice *>& devices)
{
	for (auto *device : devices)
	{
		pSystem->DestroyDevice(device);
	}
}

static void set_params(std::vector<IDevice *>& devices, GenICam::gcstring format, double gain, bool reverseX, bool reverseY)
{
	for (auto *device : devices)
	{
		SetNodeValue<GenICam::gcstring>(device->GetNodeMap(), "PixelFormat", format);
		SetNodeValue<double>(device->GetNodeMap(), "Gain", gain);
		SetNodeValue<bool>(device->GetNodeMap(), "ReverseX", reverseX);
		SetNodeValue<bool>(device->GetNodeMap(), "ReverseY", reverseY);

		// パケットサイズ自動設定
		SetNodeValue<bool>( device->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true );
		// パケット再送許可
		SetNodeValue<bool>( device->GetTLStreamNodeMap(), "StreamPacketResendEnable", true );

		// 画像サイズを縦横それぞれ1/2にする。
		SetNodeValue<GenICam::gcstring>(device->GetNodeMap(), "BinningSelector", "Sensor");
		SetNodeValue<GenICam::gcstring>(device->GetNodeMap(), "BinningHorizontalMode", "Average");
		SetNodeValue<GenICam::gcstring>(device->GetNodeMap(), "BinningVerticalMode", "Average");
		SetNodeValue<int64_t>(device->GetNodeMap(), "BinningHorizontal", 2);
		SetNodeValue<int64_t>(device->GetNodeMap(), "BinningVertical", 2);
	}
}


/*!
 * Reads the params from the console
 * @param private_nh[in] Private ROS node handle
 * @param fps[out] Read value from the console Hz
 * @param format[out] Read value from the console pixel format string
 * @param timeout[out] Read value from the console timeout in ms
 * @param gain[out] Read value from the console gain in 0.0 to 48.0
 */
static void ros_get_params(const ros::NodeHandle &private_nh, int &fps, GenICam::gcstring &pixel_format, int &timeout, double &gain, bool &reverseX, bool &reverseY)
{
	if (private_nh.getParam("fps", fps))
	{
		ROS_INFO("fps set to %d", fps);
	} else {
		fps = 20;
		ROS_INFO("No param received, defaulting fps to %d", fps);
	}

	std::string format;
	if (private_nh.getParam("format", format))
	{
	  ROS_INFO("format set to %s", format.c_str());
	} else {
	  format = "BayerRG8";
	  ROS_INFO("No param received, defaulting format to %s", format.c_str());
	}
	pixel_format = GenICam::gcstring(format.c_str());

	if (private_nh.getParam("timeout", timeout))
	{
	  ROS_INFO("timeout set to %d ms", timeout);
	} else {
	  timeout = 1000;
	  ROS_INFO("No param received, defaulting timeout to %d ms", timeout);
	}

	if (private_nh.getParam("gain", gain))
	{
	  ROS_INFO("gain set to %f", gain);
	} else {
	  gain = 0.0;
	  ROS_INFO("No param received, defaulting gain to %f", gain);
	}
	if (private_nh.getParam("reverse_x", reverseX))
	{
	  ROS_INFO("flip the image vertically set to %d", reverseX);
	} else {
	  reverseX = false;
	  ROS_INFO("No param received, defaulting reverseX to %d", reverseX);
	}
	if (private_nh.getParam("reverse_y", reverseY))
	{
	  ROS_INFO("flip the image vertically set to %d", reverseY);
	} else {
	  reverseY = false;
	  ROS_INFO("No param received, defaulting reverseY to %d", reverseY);
	}
}

static int GetPolarizeData( IImage* pImage, size_t width, size_t height, void* pDeg0, void* pDeg45, void* pDeg90, void* pDeg135 )
{
	uint64_t pixelFormat = pImage->GetPixelFormat();
	uint8_t* inputBuff = const_cast< uint8_t* >( pImage->GetData() );

	if ( pixelFormat == BayerRG8 )
	{
		uint8_t* src = inputBuff;
		uint8_t* dest0 = static_cast< uint8_t* >( pDeg0 );
		uint8_t* dest45 = static_cast< uint8_t* >( pDeg45 );
		uint8_t* dest90 = static_cast< uint8_t* >( pDeg90 );
		uint8_t* dest135 = static_cast< uint8_t* >( pDeg135 );

		// 偏光データを取り出しメモリに格納
		for ( size_t y = 0; y < height; y += 2 )
		{
			uint8_t* srcImage = src + y * width;

			for ( size_t x = 0; x < width; x += 2 )
			{
				*dest90++ = *srcImage;
				*dest45++ = *( srcImage + 1 );
				*dest135++ = *( srcImage + width );
				*dest0++ = *( srcImage + width + 1 );

				srcImage += 2;
			}
		}

		return 0;
	}
	else if ( pixelFormat == BayerRG16 )
	{
		uint16_t* src = reinterpret_cast< uint16_t* >( inputBuff );
		uint16_t* dest0 = reinterpret_cast< uint16_t* >( pDeg0 );
		uint16_t* dest45 = reinterpret_cast< uint16_t* >( pDeg45 );
		uint16_t* dest90 = reinterpret_cast< uint16_t* >( pDeg90 );
		uint16_t* dest135 = reinterpret_cast< uint16_t* >( pDeg135 );

		// 偏光データを取り出しメモリに格納
		for ( size_t y = 0; y < height; y += 2 )
		{
			uint16_t* srcImage = src + y * width;

			for ( size_t x = 0; x < width; x += 2 )
			{
				*dest90++ = *srcImage;
				*dest45++ = *( srcImage + 1 );
				*dest135++ = *( srcImage + width );
				*dest0++ = *( srcImage + width + 1 );

				srcImage += 2;
			}
		}

		return 0;
	}
	else
		return -1;
}

// demonstrates saving an image
// (1) converts image to a displayable pixel format
// (2) prepares image parameters
// (3) prepares image writer
// (4) saves image
// (5) destroys converted image
static void SaveImage(Arena::IImage* pImage, const char* filename)
{

	// Convert image
	//    Convert the image to a displayable pixel format. It is worth keeping in
	//    mind the best pixel and file formats for your application. This example
	//    converts the image so that it is displayable by the operating system.
	//std::cout << TAB1 << "Convert image to " << GetPixelFormatName(PIXEL_FORMAT) << "\n";

	auto pConverted = Arena::ImageFactory::Convert(
		pImage,
		PIXEL_FORMAT);

	// Prepare image parameters
	//    An image's width, height, and bits per pixel are required to save to
	//    disk. Its size and stride (i.e. pitch) can be calculated from those 3
	//    inputs. Notice that an image's size and stride use bytes as a unit
	//    while the bits per pixel uses bits.
	//std::cout << TAB1 << "Prepare image parameters\n";

	Save::ImageParams params(
		pConverted->GetWidth(),
		pConverted->GetHeight(),
		pConverted->GetBitsPerPixel());

	// Prepare image writer
	//    The image writer requires 3 arguments to save an image: the image's
	//    parameters, a specified file name or pattern, and the image data to
	//    save. Providing these should result in a successfully saved file on the
	//    disk. Because an image's parameters and file name pattern may repeat,
	//    they can be passed into the image writer's constructor.
	//std::cout << TAB1 << "Prepare image writer\n";

	Save::ImageWriter writer(
		params,
		filename);

	// Save image
	//    Passing image data into the image writer using the cascading I/O
	//    operator (<<) triggers a save. Notice that the << operator accepts the
	//    image data as a constant unsigned 8-bit integer pointer (const
	//    uint8_t*) and the file name as a character string (const char*).
	//std::cout << TAB1 << "Save image\n";

	writer << pConverted->GetData();

	// destroy converted image
	Arena::ImageFactory::Destroy(pConverted);
}

static void callbackFromConfig(const autoware_config_msgs::ConfigArenaSDKConstPtr& config)
{
	degree_ = config->degree;
	gain_ = config->gain;
	save_img_ = config->save;

}

int main(int argc, char **argv)
{
	////ROS STUFF////
	ros::init(argc, argv, "lucid_camera");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	signal(SIGTERM, signal_handler);//detect closing

	int fps, timeout;
	GenICam::gcstring str_pixel_format;
	double current_gain;
	bool reverseX, reverseY;

	ros_get_params(private_nh, fps, str_pixel_format, timeout, gain_, reverseX, reverseY);

	degree_ = IMAGE_ALL;
	save_img_ = false;

	//init cameras
	ISystem* pSystem = OpenSystem();
	pSystem->UpdateDevices(timeout);
	std::vector<DeviceInfo> deviceInfos = pSystem->GetDevices();
	int camera_num = deviceInfos.size();
	if (camera_num == 0) {
		std::cout << "Lucid camera is not found... " << std::endl;
		return -1;
	}

	// setup subscriber
	ros::Subscriber sub_config;
	sub_config = n.subscribe("/config/arena_sdk", 1, callbackFromConfig);

	// setup publisher
	ros::Publisher pub[camera_num];
	ros::Publisher camera_info_pub;

	std::vector<IDevice *> devices;
	for (int i = 0; i < 1; i++){//i < camera_num; i++) {
		std::cout << deviceInfos[i].SerialNumber() << std::endl;
		IDevice* pDevice = pSystem->CreateDevice(deviceInfos[i]);
		devices.push_back(pDevice);

		std::string topic(std::string("image_raw"));

		//if (camera_num > 1) {
		//	topic = "camera" + std::to_string(i) + "/" + topic;
		//} 
		pub[i] = n.advertise<sensor_msgs::Image>(topic, 100);
		ROS_INFO("Publishing.. %s", topic.c_str());
	}

	set_params(devices, str_pixel_format, gain_, reverseX, reverseY);
	current_gain = gain_;

	start_streams(devices);

	std::cout << "Capturing by " << camera_num << " cameras..." << std::endl;

	int count = 0;
	ros::Rate loop_rate(fps); // Hz
	while (running && ros::ok())
	{
		int i = 0;
		for (auto *device : devices)
		{
			IImage *pImage;
			try {
				if (current_gain != gain_) {
					SetNodeValue<double>(device->GetNodeMap(), "Gain", gain_);
					current_gain = gain_;
				}

				pImage = device->GetImage(timeout);
				if (!pImage) {
					continue;
				}

			} catch (GenICam::TimeoutException &e) {
				std::cout << "GenICam::TimeoutException: " << e.what() << std::endl;
				continue;
			}

			size_t dataWidth = pImage->GetWidth();
			size_t dataHeight = pImage->GetHeight();
			size_t payload_size = pImage->GetPayloadSize();
			uint64_t pixelFormat = pImage->GetPixelFormat();
#if 0	// 偏光処理はとりあえずしない(重くなる可能性があるので)
			// RAW画像から偏光データを抽出
			std::unique_ptr<uint16_t[]> deg0 = std::make_unique<uint16_t[]>( dataWidth * dataHeight / 4 );
			std::unique_ptr<uint16_t[]> deg45 = std::make_unique<uint16_t[]>( dataWidth * dataHeight / 4 );
			std::unique_ptr<uint16_t[]> deg90 = std::make_unique<uint16_t[]>( dataWidth * dataHeight / 4 );
			std::unique_ptr<uint16_t[]> deg135 = std::make_unique<uint16_t[]>( dataWidth * dataHeight / 4 );

			GetPolarizeData( pImage, dataWidth, dataHeight, deg0.get(), deg45.get(), deg90.get(), deg135.get() );

			// デモザイキングのためにベイヤーイメージをセット
			IImage* pBayer0 = ImageFactory::Create( ( const uint8_t* )deg0.get(), payload_size / 4, dataWidth / 2, dataHeight / 2, pixelFormat );
			IImage* pBayer45 = ImageFactory::Create( ( const uint8_t* )deg45.get(), payload_size / 4, dataWidth / 2, dataHeight / 2, pixelFormat );
			IImage* pBayer90 = ImageFactory::Create( ( const uint8_t* )deg90.get(), payload_size / 4, dataWidth / 2, dataHeight / 2, pixelFormat );
			IImage* pBayer135 = ImageFactory::Create( ( const uint8_t* )deg135.get(), payload_size / 4, dataWidth / 2, dataHeight / 2, pixelFormat );

			// デモザイキング
			IImage* pConverted0 = ImageFactory::Convert( pBayer0, BGR8 );
			IImage* pConverted45 = ImageFactory::Convert( pBayer45, BGR8 );
			IImage* pConverted90 = ImageFactory::Convert( pBayer90, BGR8 );
			IImage* pConverted135 = ImageFactory::Convert( pBayer135, BGR8 );
#endif
#if 0
			size_t dataBpp = pImage->GetBitsPerPixel();
			size_t image_size = pImage->GetSizeFilled();
			std::ofstream fout;
			// そのままファイルに保存
			std::cout << "width:" << dataWidth << " height:" << dataHeight << " bpp:" << dataBpp <<"\n";
			std::cout << "GetSizeFilled:" << image_size << " GetPayloadSize:" << payload_size <<"\n";
			std::cout << "RAWファイル保存 -> ファイル名 : " << FILE_NAME << "\n";
			fout.open( FILE_NAME, std::ios::binary );
			fout.write( reinterpret_cast< const char* >( pImage->GetData() ), payload_size );
			fout.close();
#endif

			sensor_msgs::Image msg;
			//publish*******************

			msg.header.seq = count;
			std::string frame = "camera" + std::to_string(i);
			msg.header.frame_id = frame;
			msg.header.stamp.sec = ros::Time::now().sec;
			msg.header.stamp.nsec = ros::Time::now().nsec;

			if (degree_ == IMAGE_ALL) {
				if (save_img_) {
					std::string fname = "img/img" + std::to_string(count) + ".jpg";
					SaveImage(pImage, fname.c_str());
				}

#if 0
				// 画像を縮小せずに送信
				msg.height = dataHeight;
				msg.width  = dataWidth;
				msg.encoding = "bayer_rggb8";
				msg.step = dataWidth;

				msg.data.resize(payload_size);
				memcpy(msg.data.data(), reinterpret_cast<const char*>(pImage->GetData()), payload_size);
#else
				// 画像をRGB変換+縮小して送信
				unsigned char *imagebuffer = (unsigned char *)pImage->GetData();
				cv::Mat bayer_image(cv::Size(dataWidth, dataHeight), CV_8UC1, imagebuffer);
				cv::Mat color_image(cv::Size(dataWidth, dataHeight), CV_8UC3);
				cv::cvtColor(bayer_image, color_image, CV_BayerBG2RGB);
				cv::Mat resized_color_image;
				cv::resize(color_image, resized_color_image, cv::Size(), RESIZE_WIDTH / color_image.cols , RESIZE_HEIGHT / color_image.rows);

				msg.height = resized_color_image.size().height;
				msg.width  = resized_color_image.size().width;
				msg.encoding = "rgb8";
				msg.step = resized_color_image.cols * resized_color_image.elemSize();

				size_t image_size = resized_color_image.rows * resized_color_image.cols * resized_color_image.elemSize();

				msg.data.resize(image_size);
				memcpy(msg.data.data(), resized_color_image.data, image_size);
#endif
			} else {
#if 0	// 偏光処理はとりあえずしない(重くなる可能性があるので)
				if (save_img_) {
					std::string fname = "img/img_deg000_" + std::to_string(count) + ".jpg";
					SaveImage(pConverted0, fname.c_str());
					fname = "img/img_deg045_" + std::to_string(count) + ".jpg";
					SaveImage(pConverted45, fname.c_str());
					fname = "img/img_deg090_" + std::to_string(count) + ".jpg";
					SaveImage(pConverted90, fname.c_str());
					fname = "img/img_deg135_" + std::to_string(count) + ".jpg";
					SaveImage(pConverted135, fname.c_str());
				}
				IImage *pImg;
				if (degree_ == IMAGE_DEG0) pImg = pConverted0;
				else if (degree_ == IMAGE_DEG45) pImg = pConverted45;
				else if (degree_ == IMAGE_DEG90) pImg = pConverted90;
				else pImg = pConverted135;

				dataWidth = pImg->GetWidth();
				dataHeight = pImg->GetHeight();
				payload_size = pImg->GetPayloadSize();

				msg.height = dataHeight;
				msg.width  = dataWidth;
				msg.encoding = "bgr8";
				msg.step = dataWidth * 3;

				msg.data.resize(payload_size);
				memcpy(msg.data.data(), reinterpret_cast<const char*>(pImg->GetData()), payload_size);
#endif
			}
			pub[i].publish(msg);
			i++;
#if 0	// 偏光処理はとりあえずしない(重くなる可能性があるので)
			ImageFactory::Destroy(pConverted0);
			ImageFactory::Destroy(pConverted45);
			ImageFactory::Destroy(pConverted90);
			ImageFactory::Destroy(pConverted135);
			ImageFactory::Destroy(pBayer0);
			ImageFactory::Destroy(pBayer45);
			ImageFactory::Destroy(pBayer90);
			ImageFactory::Destroy(pBayer135);
#endif
			device->RequeueBuffer(pImage);
			loop_rate.sleep();
		}
		ros::spinOnce();
		//loop_rate.sleep();
		count++;
	}

	// stop streams
	stop_streams(devices);

	// destroy devices
	destroy_devices(pSystem, devices);

	//close cameras
	CloseSystem(pSystem);

	ROS_INFO("Camera node closed correctly");
	std::cout << "Camera node closed correctly" << std::endl;

	return 0;
}
