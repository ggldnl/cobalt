
#ifndef BACKGROUND_THREAD_H
#define BACKGROUND_THREAD_H

#include <mutex>
#include <thread>
#include <condition_variable>

/**
 * signal the thread to start or resume asynchronously
 * signal the thread to stop or pause asynchronously
 */
class BackgroundThread {

	private:

		// the condition_variable class is a synchronization primitive 
		// that can be used to block a thread, or multiple threads 
		// at the same time, until another thread both modifies 
		// a shared variable (the condition), and notifies 
		// the condition_variable.
		std::condition_variable condition_variable;
		std::mutex mutex;

		bool running = false;
		bool is_paused = false;
		int interval = 1000; // update interval, default 1 sec

		void update (void);

		void set_paused (bool paused) {
			std::lock_guard<std::mutex> lck(mutex);
			is_paused = paused;
			condition_variable.notify_one();
		}

	public:

		/**
		 * Start the thread
		 */
		void start (void) {

			running = true;
			std::thread( // lambda 
				// https://thispointer.com/c11-lambda-how-to-capture-member-variables-inside-lambda-function/
				[this]() { // [list of captured variables] (parameters)
					while (this -> running) {
						
						std::unique_lock<std::mutex> lck(this -> mutex);
						condition_variable.wait(lck,[]{return !this -> is_paused;});
						this -> update();

						std::this_thread::sleep_for(
							std::chrono::milliseconds(this -> interval) // interval in ms
						);
					}
				}).detach(); // background
		}

		/** 
		 * Pause the thread. Can be resumed
		 */
		void pause (void) {
			set_paused(true);
		}

		/**
		 * Resume the thread if its paused
		 */
		void resume (void) {
			set_paused(false);
		}

		/**
		 * Stop the thread if its running
		 */
		void stop (void) {
			running = false; // exit the loop
		}

};

#endif

// https://www.cplusplus.com/forum/general/107753/
