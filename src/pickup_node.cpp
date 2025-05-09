//ros related includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>


#include <vector>
#include <unistd.h>
#include <stdlib.h>
#include <uldaq.h>
#include <signal.h>
#include "utility.h"

//ros::Publisher pickup_publisher;
rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pickup_publisher;

#define MAX_DEV_COUNT  100
#define MAX_STR_LENGTH 64
#define MAX_SCAN_OPTIONS_LENGTH 256
const std::string PUBLISH_TOPIC="/pickup_node/voltage_frames";
const std::string NODE_NAME="pickup_node";

DaqDeviceHandle daqDeviceHandle = 0;
double* buffer = nullptr;
double* buffer_copy = nullptr;
UlError err = ERR_NO_ERROR;
ScanStatus status;
TransferStatus transferStatus;

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Shutdown using SIGINT");
	//wait until system is idle again
	while(1){
		// get the initial status of the acquisition
		ulAInScanStatus(daqDeviceHandle, &status, &transferStatus);
		if(status==SS_IDLE)
			break;
		usleep(100);
	}

  	// disconnect from the DAQ device
	ulDisconnectDaqDevice(daqDeviceHandle);

	exit(1);
}

//main
int main(int argc,char** argv){
    rclcpp::init(argc, argv);

	auto floatbuffer_msg=std_msgs::msg::Float32MultiArray();

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
		"starting pickup node");

    signal(SIGINT, mySigintHandler);

	auto nh=std::make_shared<rclcpp::Node>(NODE_NAME);

	//publisher node for voltage_frames
	pickup_publisher=nh->create_publisher<std_msgs::msg::Float32MultiArray>(PUBLISH_TOPIC,1);

	//------------------------------------------
	// initialize communication to DAQ card
	//------------------------------------------

	int descriptorIndex = 0;
	DaqDeviceDescriptor devDescriptors[MAX_DEV_COUNT];
	DaqDeviceInterface interfaceType = ANY_IFC;
	unsigned int numDevs = MAX_DEV_COUNT;

	// set some variables that are used to acquire data
	int lowChan = 0;
	int highChan = 3;
	AiInputMode inputMode;
	Range range;
	int samplesPerChannel = 2000;
	double rate = 200000;
	ScanOption scanOptions = (ScanOption) (SO_DEFAULTIO | SO_EXTTRIGGER);
	AInScanFlag flags = AINSCAN_FF_DEFAULT;

	int hasAI = 0;
	int hasPacer = 0;
	int numberOfChannels = 0;
	int index = 0;

	char inputModeStr[MAX_STR_LENGTH];
	char rangeStr[MAX_STR_LENGTH];
	char scanOptionsStr[MAX_SCAN_OPTIONS_LENGTH];

	int chanCount = 0;

	int i = 0;
	int __attribute__((unused)) ret;
	char c;

	// Get descriptors for all of the available DAQ devices
	err = ulGetDaqDeviceInventory(interfaceType, devDescriptors, &numDevs);

	// verify at least one DAQ device is detected
	if (numDevs == 0)


	printf("Found %d DAQ device(s)\n", numDevs);
	for (i = 0; i < (int) numDevs; i++)
		printf("  [%d] %s: (%s)\n", i, devDescriptors[i].productName, devDescriptors[i].uniqueId);

	if(numDevs > 1)
		descriptorIndex = selectDAQDevice(numDevs);

	// get a handle to the DAQ device associated with the first descriptor
	daqDeviceHandle = ulCreateDaqDevice(devDescriptors[descriptorIndex]);

	if (daqDeviceHandle == 0)

	// verify the specified device supports analog input
	err = getDevInfoHasAi(daqDeviceHandle, &hasAI);

	// verify the specified device supports hardware pacing for analog input
	err = getAiInfoHasPacer(daqDeviceHandle, &hasPacer);

	printf("\nConnecting to device %s - please wait ...\n", devDescriptors[descriptorIndex].devString);

	// establish a connection to the DAQ device
	err = ulConnectDaqDevice(daqDeviceHandle);

	// get the first supported analog input mode
	err = getAiInfoFirstSupportedInputMode(daqDeviceHandle, &numberOfChannels, &inputMode, inputModeStr);
	inputMode=AI_DIFFERENTIAL;

	if (highChan >= numberOfChannels)
		highChan = numberOfChannels - 1;

	chanCount = highChan - lowChan + 1;

	// allocate a buffer to receive the data
    unsigned int buffercount=chanCount * samplesPerChannel;		//total number of samples in buffer
	size_t buffersize=buffercount * sizeof(double);				//size of buffer in bytes
	buffer = (double*) malloc(buffersize);
	floatbuffer_msg.data = std::vector<float>(buffercount,0);
	floatbuffer_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
	floatbuffer_msg.layout.dim[0].size=5000;
	floatbuffer_msg.layout.dim[0].stride=1;
	floatbuffer_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
	floatbuffer_msg.layout.dim[1].size=4;
	floatbuffer_msg.layout.dim[1].stride=1;

	//set trigger type
	err = ulAInSetTrigger(daqDeviceHandle, TRIG_POS_EDGE, 0, 0.0, 0.0, 0);


	// get the first supported analog input range
	err = getAiInfoFirstSupportedRange(daqDeviceHandle, inputMode, &range, rangeStr);

	ConvertScanOptionsToString(scanOptions, scanOptionsStr);


	//------------------------------------------
	// read data continusously and publish it
	//------------------------------------------
	while(1){
		//read data from the DAQ card
		err = ulAInScan(daqDeviceHandle, lowChan, highChan, inputMode, range, samplesPerChannel, &rate, scanOptions, flags, buffer);
		//wait until system is idle again
		while(1){
			// get the initial status of the acquisition
			ulAInScanStatus(daqDeviceHandle, &status, &transferStatus);
			if(status==SS_IDLE)
				break;
			usleep(100);
		}

		//copy data to uffer
		floatbuffer_msg.data.assign(buffer,buffer+buffercount);
		
		//publish the data
		pickup_publisher->publish(floatbuffer_msg);
		RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "publish voltage frames");
	}
}