#include <iostream>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "../src/pid.h"
#include <chrono>
#include <cmath>

int main(int argc, char *argv[])
{

    if (argc < 2)
    {
        std::cout << "Usage: ./main <ip>" << std::endl;
        return 1;
    }

    // Set PID constants
    const double kp = 100.0F;
    const double ki = 50.0F;
    const double kd = 10.0F;

    double time_offset;

    PID *c_ptr = new PID();
    c_ptr->Init(kp, ki, kd);

    // port to start the server on
    int SERVER_PORT = 1337;

    // socket address used for the server
    struct sockaddr_in server_address;
    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;

    // htons: host to network short: transforms a value in host byte
    // ordering format to a short value in network byte ordering format
    server_address.sin_port = htons(SERVER_PORT);

    // htons: host to network long: same as htons but to long
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);

    // create a UDP socket, creation returns -1 on failure
    int sock;
    if ((sock = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
    {
        printf("could not create socket\n");
        return 1;
    }

    // bind it to listen to the incoming connections on the created server
    // address, will return -1 on error
    if ((bind(sock, (struct sockaddr *)&server_address,
              sizeof(server_address))) < 0)
    {
        printf("could not bind socket\n");
        return 1;
    }

    // socket address used to store client address
    struct sockaddr_in client_address;
    socklen_t client_address_len = 0;

    // run indefinitely
    while (true)
    {
        char buffer[500];

        // read content into buffer from an incoming client
        int len = recvfrom(sock, buffer, sizeof(buffer), 0,
                           (struct sockaddr *)&client_address,
                           &client_address_len);

        static int initialized = 0;
        auto tp = std::chrono::system_clock::now();
        if (initialized == 0)
        {
            time_offset = tp.time_since_epoch().count() - (3.5e-05 * 1000000000.0F);
            initialized = 1;
        }

        double time = (tp.time_since_epoch().count() - time_offset) / 1000000000.0F;

        // inet_ntoa prints user friendly representation of the
        // ip address
        buffer[len] = '\0';
        double angle;
        memcpy(&angle, buffer, sizeof(angle));
        // printf("received: '%s' from client %s\n", buffer, inet_ntoa(client_address.sin_addr));

        double error = 0.0F - angle;
        c_ptr->UpdateError(time, error);
        double u = c_ptr->TotalError();

        std::cout << angle << " " << time << " " << u << std::endl;
        // send same content back to the client ("echo")
        // sendto(sock, buffer, len, 0, (struct sockaddr *)&client_address, sizeof(client_address));

        sockaddr_in servaddr;

        bzero(&servaddr, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = inet_addr(argv[1]);
        servaddr.sin_port = htons(1338);
        if (sendto(sock, (char *)&u, sizeof(u), 0, (sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        {
            printf("cannot send message");
        }
    }
}
