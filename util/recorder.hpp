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

template <class T> class recorder{

public:
	recorder();
	~recorder();
	int start_record(std::string record_name, void * param, int num_threads);
	int stop_record();
	int set_record_path(std::string record_path);
	int set_recorder_name(std::string recorder_name);
	int get_queue_length();
	int enqueue_record(T* data);
	T * dequeue_record(bool wait);

private:
	int num_threads;
	bool state_active = false;

	pthread_t *thread_handles;
	std::queue<T*> data_queue;
	pthread_mutex_t data_mutex;
	pthread_cond_t data_cv;

	static void* dummy(void*);
	void *record_thread(void*);
	void *param;
	std::string recorder_name;
	std::string record_path;
	std::string record_name;
};

template <class T> recorder<T>::recorder(){
	state_active = false;
	pthread_mutex_init(&data_mutex, NULL);
}

template <class T> int recorder<T>::get_queue_length(){
	return data_queue.size();
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
template <class T> int recorder<T>::enqueue_record(T* data){
	//LOGI("Enqueueing Record at time %f", data->timestamp);
	if(!state_active){
		LOGE("Scan was not active during enqueue. Enqueue failed!");
		return -1;
	}
	pthread_mutex_lock(&data_mutex); 
	data_queue.push(data);
	pthread_cond_signal(&data_cv);
	pthread_mutex_unlock(&data_mutex);
	return 0;
}

template <class T> T* recorder<T>::dequeue_record(bool wait){
	T * data = nullptr;
	pthread_mutex_lock(&data_mutex);
	if(wait && data_queue.empty()){
		pthread_cond_wait(&data_cv, &data_mutex);
	}
	// If the data queue is empty here, that (probably) means someone is trying to signal us to exit.
	if(data_queue.empty()){
		LOGI("No elements in Queue to remove");
		return nullptr;
	}
	LOGI("Elements remaining in %s queue: %d", recorder_name.c_str(), data_queue.size());
	data = data_queue.front();
	if(data == nullptr) {
		LOGE("Removed element is null (Queue size is %d)",data_queue.size());
	}
	data_queue.pop();
	pthread_mutex_unlock(&data_mutex);
	return data;
}

template <class T> int recorder<T>::stop_record(){
	if(state_active){
		state_active = false;
		void * args;
		for(int i = 0 ; i < num_threads; i++){
			pthread_join(thread_handles[i],&args);
			LOGI("Thread %d joined",i);
		}
		num_threads = 0;
		delete thread_handles;
	}else{
		LOGE("No scans were active!");
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
	LOGI("%s Recording thread Started", recorder_name.c_str());
	while(true){
		if(data = dequeue_record(state_active)){
			data->write_to_file(record_path, recorder_name, record_name);
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
