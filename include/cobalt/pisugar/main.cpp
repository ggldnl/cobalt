
#include <iostream>
using namespace std;

// #include "pisugar.h"
#include "pisugar1.h"
#include "pisugar3.h"

#include <iostream>
#include <chrono>
#include <thread>


int main() {

	PiSugarCreator* creator1 = new PiSugar1Creator();
    // call the factory method to create a Product object.    
    PiSugar* pisugar1 = creator1 -> factory();
    pisugar1 -> start(); 

	PiSugarCreator* creator2 = new PiSugar3Creator();
    // call the factory method to create a Product object.    
    PiSugar* pisugar2 = creator2 -> factory();
	pisugar2 -> start();
	
    cout << "current thread sleeping for 2000ms" << endl;
    std::this_thread::sleep_for (std::chrono::seconds(2));
	
	
	float result1 = pisugar1 -> get_voltage();
	cout << "voltage read by PiSugar1: " << result1 << endl;
	
	float result2 = pisugar2 -> get_voltage();
	cout << "voltage read by PiSugar2: " << result2 << endl;
	
	delete creator1;
	delete creator2;

	return 0;
}