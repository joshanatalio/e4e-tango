#ifndef RECORDER_HPP
#define RECORDER_HPP

#include <tango_client_api.h>

#include <pthread.h>
#include <queue>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <semaphore.h>
template <class T> class recorder{

public:
	recorder();
	~recorder();
	int start_record(std::string record_name, void * param, int num_threads);
	int stop_record();
	int set_record_path(std::string record_path);
	int set_recorder_name(std::string recorder_name);
	int enqueue_record(T* data);
	T * dequeue_record();
	int get_queue_length();
private:
	int num_threads;
	int threads_active;
	bool state_active = false;

	pthread_t *thread_handles;
	std::queue<T*> data_queue;
	pthread_mutex_t data_mutex;
	pthread_cond_t data_cv;

	pthread_rwlock_t queue_lock;
	sem_t queue_semaphore;

	static void* dummy(void*);

	void *record_thread(void*);
	void *param;
	std::string recorder_name;
	std::string record_path;
	std::string record_name;
};

template <class T> recorder<T>::recorder(){
	state_active = false;
	thread_handles = nullptr;
	threads_active = 0;
	pthread_rwlock_init(&queue_lock, NULL);
}

template <class T> recorder<T>::~recorder(){
	if(state_active){
		LOGI("Scan was still active on deletion!");
		stop_record();
	}
	while(data_queue.size() > 0){
		delete data_queue.front();
		data_queue.pop();
	}
}

template <class T> int recorder<T>::get_queue_length(){
	return data_queue.size();

}
template <class T> int recorder<T>::enqueue_record(T* data){
	int i;
	//LOGI("Enqueueing Record at time %f", data->timestamp);
	if(!state_active){
		LOGE("Scan was not active during enqueue. Enqueue failed!");
		return -1;
	}
	pthread_rwlock_wrlock(&queue_lock);
	data_queue.push(data);
	sem_post(&queue_semaphore);
	pthread_rwlock_unlock(&queue_lock);
	return 0;
}

template <class T> T* recorder<T>::dequeue_record(){
	T * data = nullptr;
	sem_wait(&queue_semaphore);
	pthread_rwlock_rdlock(&queue_lock);
	if(data_queue.empty()){
		LOGI("No elements in Queue to remove");
		pthread_rwlock_unlock(&queue_lock);
		return nullptr;
	}
	//LOGI("Elements remaining in %s queue: %d", recorder_name.c_str(), data_queue.size());
	data = data_queue.front();
	if(data == nullptr) {
		LOGE("Removed element is null (Queue size is %d)",data_queue.size());
	}
	data_queue.pop();
	pthread_rwlock_unlock(&queue_lock);
	return data;
}

template <class T> int recorder<T>::stop_record(){
	void * args;
	int i;
	state_active = false;
	if(!state_active){
    	state_active = false;
    	LOGI("Set State to false!");
    	for( i = 0 ; i < num_threads; i++ ){
        	sem_post(&queue_semaphore);
        }
    }
	for( i = 0 ; i < num_threads; i++ ){
		pthread_join(thread_handles[i],&args);
		LOGI("Thread %d joined",i);
	}
	num_threads = 0;
	threads_active = 0;
	if(thread_handles){
		delete thread_handles;
		thread_handles = nullptr;
	}
}

template <class T> int recorder<T>::set_record_path(std::string record_path){
	LOGI("Set record path to %s",record_path.c_str());
	this->record_path = record_path;
}

template <class T> int recorder<T>::set_recorder_name(std::string recorder_name){
	LOGI("Set recorder name to %s",recorder_name.c_str());
	this->recorder_name = recorder_name;
}

template <class T> int recorder<T>::start_record(std::string record_name, void * param, int num_threads){
	this->num_threads = num_threads;
	sem_init(&queue_semaphore, 0, 0);
	state_active = true;
	LOGI("Starting scan %s", record_name.c_str());
	this->record_name = record_name;
	T::write_to_initialization(record_path, recorder_name, record_name, param);
	thread_handles = new pthread_t[num_threads];
	for(int i = 0; i < num_threads; i++){
		pthread_create(&thread_handles[i], nullptr, dummy, (void*)this);
	}
}


template <class T> void * recorder<T>::record_thread(void * ){
	T * data;
	int tid = threads_active++; // TODO: Dangerous. This is a hack.
	LOGI("%s Recording thread %d Started", recorder_name.c_str(), tid);
	while(true){
		if((data = dequeue_record()) != nullptr){
			data->write_to_file(record_path, recorder_name, record_name, tid);
			delete data;
		} else if(!state_active){
			break;
		}
	}
	return nullptr;
}

template <class T> void* recorder<T>::dummy(void *arg){
	(reinterpret_cast< recorder<T>*> (arg))->record_thread(nullptr);
}

#endif
