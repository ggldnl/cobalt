#include "background_thread.h"

void BackgroundThread::start (void) {
    
    running = true;

	std::thread( // lambda 
		// https://thispointer.com/c11-lambda-how-to-capture-member-variables-inside-lambda-function/
		[this]() { // [list of captured variables] (parameters)
			while (this -> running) {

				while (this -> paused) {
				    
                    std::unique_lock<std::mutex> lock(this -> mutex);
                    
                    // need the condition in here in case of spurious wake up, 
                    // wont exit condition unless condition true
                    condition_variable.wait(lock, [this]{return !this -> paused;});
                }
				
				// below version does not account for spurious wake up; need a while loop
				// condition_variable.wait(lock, [&]{return !paused; });

				update ();

				std::this_thread::sleep_for(
					std::chrono::milliseconds(this -> interval) // interval in ms
				);
			}
		}).detach(); // background

}

void BackgroundThread::stop (void) {
    
    resume(); // in case the thread is hanging on the condition
	running = false; // exit the loop
}

void BackgroundThread::pause (void) {
    
    // lock around the variable so we can modify it
    std::unique_lock<std::mutex> lock(mutex);
    
    // only this thread can modify this variable now
    paused = true;
    
    // lock automatically released when out of scope
}

void BackgroundThread::resume (void) {
    
    // lock around the variable so we can modify it
    std::unique_lock<std::mutex> lock(mutex);
    
    // only this thread can modify this variable now
    paused = false;

    // unlock to stop threads from immediately locking when notify is called.
    lock.unlock();

    // wake up the thread waiting on the condition
    condition_variable.notify_one(); // same as notify_all()
}

void BackgroundThread::set_interval (int _interval) {
	interval = _interval;
}

//void BackgroundThread::update (void) {
//	return; // default implementation, do nothing
//}