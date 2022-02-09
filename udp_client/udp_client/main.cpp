#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010

#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 15 //ms

#include <chrono>
#include <stdio.h>
#include <iostream>
#include <stdlib.h> 
#include <ws2tcpip.h>
#pragma comment (lib, "ws2_32.lib")


#define SERVER "192.168.72.184"	//ip address of udp server
#define BUFLEN 512	//Max length of buffer
#define PORT 4210	//The port on which to listen for incoming data


using namespace std;

float map(float value, float istart, float istop, float ostart, float ostop) {
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}
void sbusPreparePacket(unsigned char packet[], int channels[], bool isSignalLoss, bool isFailsafe) {

    static int output[SBUS_CHANNEL_NUMBER] = { 0 };

    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
     */
    for (unsigned char i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
    }

    unsigned char stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS;
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;
    }
    packet[0] = SBUS_FRAME_HEADER; //Header

    packet[1] = (unsigned char)(output[0] & 0x07FF);
    packet[2] = (unsigned char)((output[0] & 0x07FF) >> 8 | (output[1] & 0x07FF) << 3);
    packet[3] = (unsigned char)((output[1] & 0x07FF) >> 5 | (output[2] & 0x07FF) << 6);
    packet[4] = (unsigned char)((output[2] & 0x07FF) >> 2);
    packet[5] = (unsigned char)((output[2] & 0x07FF) >> 10 | (output[3] & 0x07FF) << 1);
    packet[6] = (unsigned char)((output[3] & 0x07FF) >> 7 | (output[4] & 0x07FF) << 4);
    packet[7] = (unsigned char)((output[4] & 0x07FF) >> 4 | (output[5] & 0x07FF) << 7);
    packet[8] = (unsigned char)((output[5] & 0x07FF) >> 1);
    packet[9] = (unsigned char)((output[5] & 0x07FF) >> 9 | (output[6] & 0x07FF) << 2);
    packet[10] = (unsigned char)((output[6] & 0x07FF) >> 6 | (output[7] & 0x07FF) << 5);
    packet[11] = (unsigned char)((output[7] & 0x07FF) >> 3);
    packet[12] = (unsigned char)((output[8] & 0x07FF));
    packet[13] = (unsigned char)((output[8] & 0x07FF) >> 8 | (output[9] & 0x07FF) << 3);
    packet[14] = (unsigned char)((output[9] & 0x07FF) >> 5 | (output[10] & 0x07FF) << 6);
    packet[15] = (unsigned char)((output[10] & 0x07FF) >> 2);
    packet[16] = (unsigned char)((output[10] & 0x07FF) >> 10 | (output[11] & 0x07FF) << 1);
    packet[17] = (unsigned char)((output[11] & 0x07FF) >> 7 | (output[12] & 0x07FF) << 4);
    packet[18] = (unsigned char)((output[12] & 0x07FF) >> 4 | (output[13] & 0x07FF) << 7);
    packet[19] = (unsigned char)((output[13] & 0x07FF) >> 1);
    packet[20] = (unsigned char)((output[13] & 0x07FF) >> 9 | (output[14] & 0x07FF) << 2);
    packet[21] = (unsigned char)((output[14] & 0x07FF) >> 6 | (output[15] & 0x07FF) << 5);
    packet[22] = (unsigned char)((output[15] & 0x07FF) >> 3);

    packet[23] = stateByte; //Flags byte
    packet[24] = SBUS_FRAME_FOOTER; //Footer
}

unsigned char sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
unsigned char sbusTime = 0;


void loop() {
    using namespace std::chrono;
    milliseconds mil(1000);
    mil = mil * 60;
    unsigned char currentMillis = mil.count();
    for (int i = 0; i < 15; i++)
        rcChannels[i] = 1500;
    /*
     * Modify channel values(1-16) between 1000,2000
     */
    if (currentMillis > sbusTime) {
        sbusPreparePacket(sbusPacket, rcChannels, false, false);
        //        Serial.write(sbusPacket, SBUS_PACKET_LENGTH);

        sbusTime = currentMillis + SBUS_UPDATE_RATE;
    }
}
int main() {
    struct sockaddr_in si_other;
    int s, slen = sizeof(si_other);
    char buf[BUFLEN];
    char message[BUFLEN];
    WSADATA wsa;

    //Initialise winsock
    printf("\nInitialising Winsock...");
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        printf("Failed. Error Code : %d", WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    printf("Initialised.\n");

    //create socket
    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
    {
        printf("socket() failed with error code : %d", WSAGetLastError());
        exit(EXIT_FAILURE);
    }

    memset((char*)&si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
   // si_other.sin_addr.S_un.S_addr = inet_addr(SERVER);
    inet_pton(AF_INET, SERVER, &si_other.sin_addr);




    while (1)
    {
        loop();
      //printf("Enter message : ");
      //gets_s(message);


        //send the message
        int sendOk = sendto(s, (char*)sbusPacket,32,0, (struct sockaddr*)&si_other, slen);
        for (int i = 0; i < 25; i++)
            cout << sbusPacket[i] << endl;

        if (sendOk == SOCKET_ERROR)
        {
            cout << "That didn't work! " << WSAGetLastError() << endl;
        }
    }
    closesocket(s);

    WSACleanup();
    //    Serial.begin(100000, SERIAL_8E2);
    return 0;
}
