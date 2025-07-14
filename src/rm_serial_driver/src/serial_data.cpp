#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include <iostream>

using namespace crc16;
using namespace rm_serial_driver;
int main(int argc, char *argv[])
{
    if (argc != 7)
    {
        std::cout << "Usage: " << argv[0] << " <x> <y> <z> <roll> <pitch> <yaw>" << std::endl;
        return 1;
    }

    ReceivePacket packet;
    packet.reserved = 1;
    packet.x = std::stof(argv[1]);
    packet.y = std::stof(argv[2]);
    packet.z = std::stof(argv[3]);
    packet.roll = std::stof(argv[4]);
    packet.pitch = std::stof(argv[5]);
    packet.yaw = std::stof(argv[6]);

    packet.checksum = Get_CRC16_Check_Sum((uint8_t *)&packet, sizeof(packet) - 2);
    for (uint8_t i = 0; i < sizeof(packet); i++)
    {
        printf("%02X ", ((uint8_t *)&packet)[i]);
    }
    printf("\n");
    return 0;
}
