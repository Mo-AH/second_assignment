#include "ros/ros.h"
#include "second_assignment/Speed_service.h"

//publisher to publish new speed in the topic /speed
ros::Publisher pub;

//client to call the service: Speed_service.h
ros::ServiceClient client;

//create req/resp of the service 
second_assignment::Speed_service service;

//variables to display speeds
float linearSpeed = 0;
float angularSpeed = 0;

void getCommand()
{
    //print message and take a char (button)
    char input;
    std::cout << "\n\n\n\n\nBUTTONS: (just first character is taken)\n\n\t\t [X] Standard moving (START)\n";
    std::cout << "[G] Decrease linear speed\t [H] Increase linear speed\n";
    std::cout << "[J] Decrease angular speed\t [K] Increase angular speed\n";
    std::cout << "[S] Stop motion\t\t\t [R] Reset position and Stop\n";
    std::cin >> input;

    //ignore all characters after first
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (input == 'G' || input == 'g' || input == 'H' || input == 'h' || input == 'J' || input == 'j'
        || input == 'S' || input == 's' || input == 'K' || input == 'k' || input == 'X' || input == 'x'
        || input == 'R' || input == 'r')
    {
        //fill the request
        service.request.input=input;

        //wait the service if it doesn't exist and send the request
        client.waitForExistence();
        client.call(service);

        //take the response values
        linearSpeed = service.response.linear;
        angularSpeed = service.response.angular;

        //print a message in function of the speed received or the input char
        if(linearSpeed < 0)
            std::cout << "\nIt's insane to go backward! I'm gonna crash for sure ..\n ";
        else if (linearSpeed > 13)
            std::cout << "\nThis is so fast! Risk of crash is high ..\n";
        else if (input == 'R' || input == 'r')
            std::cout << "\nResetting position and stopping motor\n ";
        else if (input == 'S' || input == 's')
            std::cout << "\nStopping motor\n ";
        else if (input == 'X' || input == 'x')
            std::cout << "\nStarting moving!\n ";
        
        std::cout << "\nLinear speed: "<< linearSpeed << "\t\t";
        std::cout << "Angular speed: "<< angularSpeed << "\n\n";
    }
    else
        std::cout << "Button not valid. Retry.\n\n";
}

int main(int argc, char **argv)
{
    //initialize the node
    ros::init(argc, argv, "input_node");
    ros::NodeHandle nh;

    //get the service
    client = nh.serviceClient<second_assignment::Speed_service>("/speed_service");
	
    //run the function in loop
    while(ros::ok())
	{
		getCommand();
	}
    
    return 0;
}
