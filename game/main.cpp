#include <SFML/Graphics.hpp>
#include <iostream>

#include "Eigen/Dense"
#include "inverted_pendulum.h"
#include "tools.h"
#include "pid.h"
#include "lqr.h"

#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <strings.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>

#include <thread>

int running = 1;
double remote_u = 0;

bool udpSend(const char *msg, int len, int fd, const char *ip)
{
  sockaddr_in servaddr;

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(ip);
  servaddr.sin_port = htons(1337);
  if (sendto(fd, msg, len, 0, // +1 to include terminator
             (sockaddr *)&servaddr, sizeof(servaddr)) < 0)
  {
    perror("cannot send message");
    return false;
  }
  return true;
}

int udpReceive()
{

  std::cout << "starting receive" << std::endl;

  int SERVER_PORT = 1338;

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

  struct timeval tv;
  tv.tv_sec = 5;  // 30 Secs Timeout
  tv.tv_usec = 0; // Not init'ing this can cause strange errors

  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

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
  while (running)
  {
    char buffer[500];

    // read content into buffer from an incoming client
    int len = recvfrom(sock, buffer, sizeof(buffer), 0,
                       (struct sockaddr *)&client_address,
                       &client_address_len);

    if (len < 0)
    {
      printf("client disconnected\n");
      return 1;
    }

    // inet_ntoa prints user friendly representation of the
    // ip address
    buffer[len] = '\0';

    memcpy(&remote_u, buffer, sizeof(remote_u));

    // std::cout << remote_u << std::endl;
    //  send same content back to the client ("echo")
    //  sendto(sock, buffer, len, 0, (struct sockaddr *)&client_address, sizeof(client_address));
  }

  return 0;
}

int main(int argc,char* argv[])
{

  if (argc < 2)
  {
    std::cout << "Usage: ./main <ip> [running_time]" << std::endl;
    return 1;
  }

  int running_time = 0;
  if (argc == 3)
  {
    running_time = atoi(argv[2]);
  }


  sf::RenderWindow window(sf::VideoMode(640, 480), "Inverted Pendulum");

  // Set initial conditions
  const double p_0 = 0;
  const double theta_0 = -0.1;
  Eigen::VectorXd x_0(4);
  x_0 << p_0, to_radians(theta_0), 0, 0;

  // Set PID constants
  const double kp = 100.0F;
  const double ki = 50.0F;
  const double kd = 10.0F;

  std::vector<std::vector<double>> data;

  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
  {
    perror("cannot open socket");
    exit(-1);
  }

  std::thread t1(udpReceive);

  // Create a model with default parameters
  InvertedPendulum *ptr = new InvertedPendulum(x_0);
  PID *c_ptr = new PID();
  c_ptr->Init(kp, ki, kd);

  // Design LQR controller
  LQR optimal;
  ptr->Linearize();
  optimal.A_ = ptr->A_;
  optimal.B_ = ptr->B_;
  optimal.Q_ = Eigen::MatrixXd::Identity(4, 4);
  optimal.Q_(0, 0) = 10;
  optimal.R_ = Eigen::MatrixXd::Identity(1, 1);
  optimal.Compute();

  bool pid = true;

  // Load font
  sf::Font font;
  if (!font.loadFromFile("Roboto-Regular.ttf"))
  {
    std::cout << "Failed to load font!\n";
  }

  // Create text to display simulation time
  sf::Text text;
  text.setFont(font);
  text.setCharacterSize(24);
  const sf::Color grey = sf::Color(0x7E, 0x7E, 0x7E);
  text.setFillColor(grey);
  text.setPosition(480.0F, 360.0F);

  // Create text to display controller type
  sf::Text type;
  type.setFont(font);
  type.setCharacterSize(24);
  const sf::Color turquoise = sf::Color(0x06, 0xC2, 0xAC);
  type.setFillColor(turquoise);
  type.setPosition(480.0F, 384.0F);

  // Create a track for the cart
  sf::RectangleShape track(sf::Vector2f(640.0F, 2.0F));
  track.setOrigin(320.0F, 1.0F);
  track.setPosition(320.0F, 240.0F);
  const sf::Color light_grey = sf::Color(0xAA, 0xAA, 0xAA);
  track.setFillColor(light_grey);

  // Create the cart of the inverted pendulum
  sf::RectangleShape cart(sf::Vector2f(100.0F, 100.0F));
  cart.setOrigin(50.0F, 50.0F);
  cart.setPosition(320.0F, 240.0F);
  cart.setFillColor(sf::Color::Black);

  // Create the pole of the inverted pendulum
  sf::RectangleShape pole(sf::Vector2f(20.0F, 200.0F));
  pole.setOrigin(10.0F, 200.0F);
  pole.setPosition(320.0F, 240.0F);
  pole.setRotation(-theta_0);
  const sf::Color brown = sf::Color(0xCC, 0x99, 0x66);
  pole.setFillColor(brown);

  // Create a clock to run the simulation
  sf::Clock clock;

  float last_update = -1.0F;
  double u = 0;

  while (window.isOpen() && running)
  {
    sf::Event event;
    while (window.pollEvent(event))
    {
      switch (event.type)
      {
      case sf::Event::Closed:
        running = 0;
        break;
      }
    }

    // Update the simulation
    sf::Time elapsed = clock.getElapsedTime();
    const float time = elapsed.asSeconds();
    const std::string msg = std::to_string(time);
    text.setString("Time   " + msg.substr(0, msg.find('.') + 2));
    const std::string action = pid ? "Action PID" : "Action LQR";
    type.setString(action);


    if(running_time && time > (running_time)) running = 0;

    if (time - last_update > 0.01F)
    {
      static int first = 0;
      Eigen::VectorXd x = ptr->GetState();
      double angle = x(1);
      double error = 0.0F - angle;
      c_ptr->UpdateError(time, error);
      u = c_ptr->TotalError();
      last_update = time;
      udpSend((char *)&angle, sizeof(angle), fd, argv[1]);
      data.push_back({time, x(0), x(1)});
      if (first == 0)
      {
        first = 1;
        std::cout << angle << " " << time << " " << u << " " << remote_u << std::endl;
      }
    }

    double noise = 0; //((static_cast<double>(std::rand()) / (RAND_MAX/2)) - 1)*100;
    ptr->Update(time, remote_u + noise);

    Eigen::VectorXd x = ptr->GetState();

    double position = 320.0F + 100 * x(0);
    if (position + 50.0F > 640.0F)
    {
      x(2) = -x(2);
    }
    else if (position - 50.0F < 0.0F)
    {
      x(2) = -x(2);
    }

    ptr->SetState(x);

    // std::cout << "Position: " << x(0) << " Angle: " << x(1) << " Velocity: " << x(2) << " Angular Velocity: " << x(3) << std::endl;

    // Update SFML drawings
    cart.setPosition(position, 240.0F);
    pole.setPosition(position, 240.0F);
    pole.setRotation(to_degrees(-x(1)));

    window.clear(sf::Color::White);
    window.draw(track);
    window.draw(cart);
    window.draw(pole);
    window.draw(text);
    window.draw(type);
    window.display();
  }

  if (window.isOpen()) window.close();
  close(fd);
  running = 0;
  std::cout << "Joining" << std::endl;
  t1.join();
  Export("data.csv", {"time", "position", "angle"}, data);

  return 0;
}
