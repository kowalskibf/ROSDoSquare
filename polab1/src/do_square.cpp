#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <iostream>
#include <unistd.h>

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "do_square");

    ros::NodeHandle nh;

    ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>("/key_vel", 1);
    
    const float x_k = nh.param<float>("/x", 5.0);
    const std::string kierunek = nh.param<std::string>("/kierunek", "prawo");
    const std::string typ_ruchu = nh.param<std::string>("/typ_ruchu", "przyspieszony");
    const float a = nh.param<float>("/a", 0.5);
    const float v_k = nh.param<float>("/v", 1.0);

	// Konfiguracja

    ros::Rate loopRate(10);
        
    const float T = 0.1;

	geometry_msgs::Twist msg;

	unsigned short stan = 10;
	
	unsigned short sides_count = 0;
	
	// Parametry

	const bool prawo = kierunek == "prawo";
	const bool przyspieszenie = typ_ruchu == "przyspieszony";
        
        // Stale
        
        float alfa_k = M_PI / 2;
        
        //const float v_k = 1.0;
        float omega = M_PI / 2;

        // Zmienne
        
        float x = 0.0;
        float alfa = 0.0;
        
        float v = 0.0;
        
        // Pomiar drogi przy przyspieszeniu
        
        float x_a_d;
        
        while(ros::ok() && sides_count < 4)
        {        
        	if(przyspieszenie)
        	{
        		std::cout << "[";
        		if(stan == 10)
        		{
        			std::cout << "Przyspieszanie";
        		}
        		else if(stan == 20)
        		{
        			std::cout << "Stala predkosc";
        		}
        		else if(stan == 30)
        		{
        			std::cout << "Hamowanie";
        		}
        		else if(stan == 40)
        		{
        			std::cout << "Obracanie";
        		}
        		std::cout << "] x: " << x << " v: " << v << " alfa: " << (alfa * 180 / M_PI) << std::endl;
        		if(stan == 10)
        		{
        			v += a * T;
        			x += v * T;
        			msg.linear.x = v;
        			msg.angular.z = 0.0;
        			if(x >= x_k / 3)
        			{
        				stan = 20;
        			}
        		}
        		else if(stan == 20)
        		{
        			x += v * T;
        			msg.linear.x = v;
        			msg.angular.z = 0.0;
        			if(x >= 2 * x_k / 3)
        			{
        				stan = 30;
        			}
        		}
        		else if(stan == 30)
        		{
        			v -= a * T;
        			x += v * T;
        			msg.linear.x = v;
        			msg.angular.z = 0.0;
        			if(x >= x_k || v <= 0)
        			{
					x = 0.0;
					v = 0.0;
        				stan = 40;
        			}
        		}
        		else if(stan == 40)
        		{
        			msg.linear.x = v;
        			msg.angular.z = prawo ? -omega : omega;
        			alfa += omega * T;
        			if(alfa >= alfa_k)
        			{
        				alfa = 0.0;
        				stan = 10;
        				sides_count++;
        			}
        		}
        	}
        	else
        	{
        		std::cout << "[";
        		if(stan == 10)
        		{
        			std::cout << "Ruch jednostajny";
        		}
        		else if(stan == 20)
        		{
        			std::cout << "Obracanie";
        		}
        		std::cout << "] x: " << x << " v: " << " alfa: " << (alfa * 180 / M_PI) << std::endl;
			if(stan == 10)
			{
				msg.linear.x = v_k;
				msg.angular.z = 0.0;
				x += v_k * T;
				if(x >= x_k)
				{
					x = 0.0;
					alfa = 0.0;
					stan = 20;
					msg.linear.x = 0.0;
					sleep(5);
				}
			}
			else if(stan == 20)
			{
				msg.linear.x = 0.0;
				msg.angular.z = prawo ? -omega : omega;
				alfa += omega * T;
				if(alfa >= alfa_k)
				{
					x = 0.0;
					alfa = 0.0;
					stan = 10;
        				sides_count++;
					sleep(5);
				}
			}
		}
		
		publisher.publish(msg);

		ros::spinOnce();

		loopRate.sleep();

    }

    return 0;

}
