#ifndef BACKGROUND_THREAD_H
#define BACKGROUND_THREAD_H

#include <thread>
#include <mutex>
#include <condition_variable>

class BackgroundThread {
    
    public:
        
		/*
		 * start the background thread. The thread can be paused and
		 * then resumed or stopped
		 */
        void start (void);
        
		/*
		 * stop the background thread. Will exit the loop
		 */
        void stop (void);
        
		/*
		 * temporarily stops the thread
		 */
        void pause (void);
        
		/*
		 * resume the execution
		 */
        void resume (void);
        
		/*
		 * set update interval between one update() call and another
		 */
		void set_interval (int interval);

		/*
		 * this is the only method that a subclass needs to implement,
		 * the stuff that the background thread will keep doing
		 */
        virtual void update (void);

    private:
    
        bool running = true;
        bool paused = false;
        
        int interval = 1000; // update interval, default 1s
        
        /*
		 * the condition_variable class is a synchronization primitive 
		 * that can be used to block a thread, or multiple threads 
		 * at the same time, until another thread both modifies 
		 * a shared variable (the condition), and notifies 
		 * the condition_variable.
		 */
        std::condition_variable condition_variable;
    
        /*
         * provide mutual exclusion when multiple threads 
         * try to lock the same object
         */
        std::mutex mutex;
    
};

#endif