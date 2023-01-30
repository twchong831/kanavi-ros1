#include "carnavicomProcessor.h"
#include <time.h>
#define CHECK_processingTime    1

carnavicomLidarProcessor::carnavicomLidarProcessor()
{
    m_datagram = new carnaviDatagram();
    m_dataParser = new lidarParser();

	g_lidarModel  = 0;
}

carnavicomLidarProcessor::~carnavicomLidarProcessor()
{

}

/**
 * @brief to convert raw data to LiDAR protocol structure
 * 
 * @param data 				datagram of LiDAR raw data
 * @return carnaviDatagram 	protocol structure of Carnavicom LiDAR sensor
 */
carnaviDatagram carnavicomLidarProcessor::process(const std::vector<u_char> &data)
{
	carnaviDatagram datagram;

	if(m_dataParser->setData(data))
	{
		// printf("Parse ENd..get datagram...\n");
		m_dataParser->getLiDARdatagram(datagram);
	}

	return datagram;
}


/**
 * @brief return LiDAR Model
 * 
 * @return int ref. CARNAVICOM::MODEL::LiDAR
 */
int carnavicomLidarProcessor::getLiDARModel()
{
    return m_dataParser->getLidarModel();
}